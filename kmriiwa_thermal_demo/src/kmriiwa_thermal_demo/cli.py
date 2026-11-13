"""Terminal interface for heater, fan and manual filament feed."""
import argparse
from decimal import Decimal, InvalidOperation
import json
import math
from pathlib import Path
import secrets
import signal
import threading
import time

from .protocol import (
    COLD_EXTRUDE_C,
    MAX_JOG_RATE,
    MIN_JOG_RATE,
    Board,
    BoardError,
    Rejected,
    UnconfirmedWrite,
    load_devices,
    probe_usb,
)


FEED_RENEW_INTERVAL = 0.35
FEED_STOP_REASONS = {
    'rtd_fault': 'RTD fault sample',
    'temperature_low': 'nozzle temperature below the feed threshold',
    'invalid_temperature': 'temperature reading outside the valid range',
    'temperature_jump': 'sudden temperature change',
    'over_temperature': 'nozzle overtemperature',
    'sensor_stale': 'no recent valid temperature reading',
    'lease_timeout': 'no renewal before the feed lease expired',
    'gate': 'external feed gate activated',
    'operator_stop': 'stop command',
    'heat_off': 'heater switched off',
    'target_low': 'temperature target below the feed threshold',
    'timed_done': 'timed jog completed',
    'heater_runaway': 'temperature did not rise under heater drive',
    'reset': 'board fault reset',
}


class TerminationRequested(Exception):
    """Leave the console on SIGTERM, including while watching status."""


HELP = '''
devices                    List devices and addresses.
status [all|DEVICE]        Show temperatures, setpoints, heater output, and fan state.
watch [all|DEVICE]         Refresh every second. Press Ctrl+C to return to the prompt.
temp DEVICE TEMP_C         Set the target temperature in °C. Example: temp bed 40
fan DEVICE PERCENT         Set the nozzle fan PWM. Example: fan nozzle 35
feed DEVICE RATE           Feed at RATE mm/s while this console renews the request.
feed DEVICE RATE SECONDS   Jog at RATE mm/s for 1–5 seconds.
feed DEVICE 0              Stop manual feed. Updated nozzle firmware is required for these feed commands.
off [all|DEVICE]           Turn off heating. The fan keeps its setting.
quit                       Stop manual feed started here, turn off heaters set here, and exit.
help                       Show this list.
'''.strip()


def number(text):
    try:
        value = Decimal(text)
        result = float(value)
    except (InvalidOperation, ValueError, OverflowError):
        raise BoardError('Enter a valid number.') from None
    if not math.isfinite(result):
        raise BoardError('Enter a finite number.')
    return result


def show(board, status):
    fields = [f'{board.device.name} ({board.device.kind})',
              f'Temperature {status.temp_c:.1f} °C', f'Target {status.target_c:.1f} °C',
              f'Heater output {status.heater_pct:.0f}%', f'State {status.state}']
    if status.fan_raw is not None:
        fields.append(f'Fan PWM {status.fan_raw * 100 / 255:.1f}% ({status.fan_raw}/255)')
    if status.gate is not None:
        fields.append(f'Feed gate {int(status.gate)}')
    if status.feed_request is not None:
        fields.append(f'Analog feed request {status.feed_request:.2f} mm/s')
    if status.jog_rate is not None:
        label = 'Manual feed request' if status.feed_lease else 'Manual jog request'
        fields.append(f'{label} {status.jog_rate:.2f} mm/s')
    if status.feed_lease:
        fields.append('Continuous feed supported')
    if status.feed_paused:
        fields.append('Manual feed paused after an RTD fault sample. Motor command is zero while paused.')
    if status.stop_reason not in (None, 'none'):
        reason = FEED_STOP_REASONS.get(status.stop_reason, status.stop_reason)
        fields.append(f'Last manual feed stop: {reason}')
    if status.rtd_error is not None:
        count, _, last_code = status.rtd_error.partition('/')
        field = f'RTD fault samples {count}'
        if last_code:
            field += f' (last code {last_code})'
        fields.append(field)
    print(' | '.join(fields), flush=True)


class FeedRenewal:
    """Renew only the firmware lease issued for this particular feed request."""

    def __init__(self, board, token):
        self.board = board
        self.token = token
        self.cancel = threading.Event()
        self.thread = threading.Thread(target=self.run, name=f'feed-{board.device.name}', daemon=True)

    def run(self):
        while not self.cancel.wait(FEED_RENEW_INTERVAL):
            try:
                self.board.keep_feed(self.token)
            except BoardError as error:
                if not self.cancel.is_set():
                    print(f'\n{self.board.device.name}: Feed renewal stopped. The board lease will expire. '
                          f'{error}', flush=True)
                return


class Console:
    def __init__(self, boards, *, control=False):
        self.boards = boards
        self.writable = control
        self.heated = set()
        self.jogged = set()
        self.renewals = {}

    def cancel_renewal(self, name):
        renewal = self.renewals.get(name)
        if renewal is not None:
            renewal.cancel.set()
            # Let any KEEP already in progress finish before sending STOP/OFF.
            if renewal.thread.ident is not None:
                renewal.thread.join()
            del self.renewals[name]

    def selection(self, name='all'):
        if name == 'all':
            return list(self.boards.values())
        if name not in self.boards:
            raise BoardError(f'Unknown device {name!r}. Available devices: {", ".join(self.boards)}.')
        return [self.boards[name]]

    def read_status(self, boards):
        for board in boards:
            name = board.device.name
            try:
                status = board.status()
                show(board, status)
                feed_inactive = status.jog_rate == 0 and not status.feed_paused
                if name in self.renewals and (
                        not status.feed_lease or status.jog_rate is None or feed_inactive
                        or status.state.lower() != 'ok' or status.gate is not False
                        or status.temp_c < COLD_EXTRUDE_C or status.target_c < COLD_EXTRUDE_C):
                    self.cancel_renewal(name)
                    print(f'{name}: Feed renewal stopped after the latest status check.', flush=True)
                if feed_inactive:
                    self.jogged.discard(name)
            except BoardError as error:
                if name in self.renewals:
                    self.cancel_renewal(name)
                    print(f'{name}: Feed renewal stopped. The board lease will expire.', flush=True)
                print(f'{name}: Status unavailable. {error}', flush=True)

    def require_control(self):
        if not self.writable:
            raise BoardError('This hardware session is read-only. Restart with --control to send commands.')

    def set_temp(self, board, value):
        self.require_control()
        value = round(value, 1)
        if not 0 < value <= board.device.max_c:
            raise BoardError(f'Target temperature must be above 0 °C and at most {board.device.max_c:g} °C. '
                             'Use off to turn off heating.')
        current = board.status()
        if current.state.lower() != 'ok':
            raise BoardError(f'Board state is {current.state!r}. Heating was refused.')
        if board.device.kind == 'nozzle' and current.gate is not False:
            raise BoardError('The nozzle feed gate is enabled or its state is unknown. Heating was refused.')
        name = board.device.name
        previously_heated = name in self.heated
        self.heated.add(name)
        try:
            board.set_temp(value)
        except Rejected:
            if not previously_heated:
                self.heated.discard(name)
            raise
        status = board.status()
        show(board, status)
        if abs(status.target_c - value) > 0.051:
            raise BoardError('The board acknowledged the temperature command, but the reported target did not match.')

    def set_fan(self, board, value):
        self.require_control()
        if board.device.kind != 'nozzle':
            raise BoardError('Fan PWM is only available on the nozzle board.')
        if not 0 <= value <= 100 or not value.is_integer():
            raise BoardError('Enter a whole-number fan PWM percentage from 0 to 100.')
        percent = int(value)
        board.status()  # Check firmware role before issuing the write.
        board.set_fan(percent)
        status = board.status()
        show(board, status)
        if status.fan_raw != percent * 255 // 100:
            raise BoardError('The board acknowledged the fan command, but the reported PWM did not match.')

    def feed(self, board, rate, seconds):
        self.require_control()
        if board.device.kind != 'nozzle':
            raise BoardError('Manual feed requires a nozzle board.')
        if rate == 0 and seconds is not None:
            raise BoardError('Use feed DEVICE 0 with no duration to stop manual feed.')
        if rate != 0 and not MIN_JOG_RATE <= rate <= MAX_JOG_RATE:
            raise BoardError(f'Feed speed must be {MIN_JOG_RATE:g}–{MAX_JOG_RATE:g} mm/s.')
        name = board.device.name
        if rate == 0 and (name in self.jogged or name in self.renewals):
            self.stop_feed(board)
            return
        if name in self.renewals:
            renewal = self.renewals[name]
            if renewal.thread.is_alive():
                raise BoardError(f'Manual feed is already active here. Use feed {name} 0 first.')
            self.cancel_renewal(name)
        current = board.status()
        if current.jog_rate is None:
            raise BoardError('This nozzle firmware uses an external gate and analog input for feed. '
                             'Install the firmware with Ethernet jog support to use this command.')
        if rate == 0:
            self.stop_feed(board)
            return
        if seconds is None and not current.feed_lease:
            raise BoardError('This nozzle firmware supports timed jogs. Supply 1 to 5 seconds or update '
                             'the nozzle firmware for continuous feed.')
        if current.state.lower() != 'ok' or current.gate is not False:
            raise BoardError('Manual feed requires a healthy nozzle with the external feed gate inactive.')
        if current.jog_rate != 0 or current.feed_paused or current.feed_request != 0:
            raise BoardError('The nozzle already reports a feed request.')
        if current.temp_c < COLD_EXTRUDE_C or current.target_c < COLD_EXTRUDE_C:
            raise BoardError(f'Manual feed requires the nozzle temperature and target to be at least '
                             f'{COLD_EXTRUDE_C:g} °C.')
        token = f'{secrets.randbelow(0xffffffff) + 1:08X}' if seconds is None else None
        try:
            if token is None:
                board.jog(rate, seconds)
            else:
                board.start_feed(rate, token)
        except (KeyboardInterrupt, TerminationRequested):
            self.jogged.add(name)
            raise
        except UnconfirmedWrite as error:
            self.jogged.add(name)
            raise BoardError(f'{error} Check status or use feed {name} 0 before trying again.') from error
        self.jogged.add(name)
        if token is None:
            print(f'Nozzle accepted a {seconds}-second manual jog at {rate:.3f} mm/s. '
                  f'Use feed {name} 0 to stop it.', flush=True)
        else:
            renewal = FeedRenewal(board, token)
            self.renewals[name] = renewal
            renewal.thread.start()
            print(f'Nozzle accepted manual feed at {rate:.3f} mm/s. Use feed {name} 0 to stop it.', flush=True)

    def stop_feed(self, board):
        self.cancel_renewal(board.device.name)
        try:
            board.stop_jog()
        except UnconfirmedWrite as error:
            stopped = board.status()
            show(board, stopped)
            if stopped.jog_rate != 0 or stopped.feed_paused:
                raise error
            self.jogged.discard(board.device.name)
            print('The stop reply was lost. The nozzle now reports no manual feed.', flush=True)
            return
        stopped = board.status()
        show(board, stopped)
        if stopped.jog_rate != 0 or stopped.feed_paused:
            raise BoardError('Manual feed shutdown could not be confirmed.')
        self.jogged.discard(board.device.name)

    def turn_off(self, boards):
        self.require_control()
        for board in boards:
            renewal = self.renewals.get(board.device.name)
            if renewal is not None:
                renewal.cancel.set()
        all_confirmed = True
        for board in boards:
            name = board.device.name
            try:
                self.cancel_renewal(name)
                if name not in self.heated and name not in self.jogged:
                    board.status()  # Check role before writing to an unfamiliar port.
                board.off()
                status = board.status()
                if status.target_c != 0 or status.heater_pct != 0:
                    time.sleep(0.55)
                    status = board.status()
                show(board, status)
                if status.target_c != 0 or status.heater_pct != 0:
                    raise BoardError('The heater target or output is still above zero.')
                if status.feed_paused or (status.jog_rate is not None and status.jog_rate != 0):
                    raise BoardError('The board still reports an active or paused manual feed request.')
                if status.jog_rate == 0:
                    self.jogged.discard(name)
                self.heated.discard(name)
            except BoardError as error:
                print(f'{name}: Heater shutdown unconfirmed. Check the board locally. {error}', flush=True)
                all_confirmed = False
        return all_confirmed

    def command(self, line):
        words = line.split()
        if not words:
            return True
        command, args = words[0].lower(), words[1:]
        if command == 'help' and not args:
            print(HELP)
        elif command == 'devices' and not args:
            for board in self.boards.values():
                device = board.device
                print(f'{device.name} {device.kind} {device.endpoint} | Maximum target {device.max_c:g} °C')
        elif command == 'quit' and not args:
            return False
        elif command in ('status', 'watch', 'off') and len(args) <= 1:
            boards = self.selection(*args)
            if command == 'off':
                self.turn_off(boards)
            elif command == 'status':
                self.read_status(boards)
            else:
                try:
                    while True:
                        self.read_status(boards)
                        time.sleep(1.0)
                except KeyboardInterrupt:
                    print()
        elif command in ('temp', 'fan') and len(args) == 2 and args[0] != 'all':
            board = self.selection(args[0])[0]
            value = number(args[1])
            if command == 'temp':
                self.set_temp(board, value)
            else:
                self.set_fan(board, value)
        elif command == 'feed' and 2 <= len(args) <= 3 and args[0] != 'all':
            board = self.selection(args[0])[0]
            seconds = None
            if len(args) == 3:
                if args[2] not in ('1', '2', '3', '4', '5'):
                    raise BoardError('Feed duration must be a whole number from 1 to 5 seconds.')
                seconds = int(args[2])
            self.feed(board, number(args[1]), seconds)
        else:
            raise BoardError('Unknown command or invalid arguments. Type help for usage.')
        return True

    def run(self):
        mode = 'hardware control' if self.writable else 'hardware read-only'
        print(f'Thermal control | Mode: {mode} | Type help for commands.', flush=True)
        confirmed = True
        try:
            self.read_status(self.selection())
            while True:
                try:
                    if not self.command(input('thermal> ')):
                        break
                except BoardError as error:
                    print(str(error), flush=True)
        except (KeyboardInterrupt, TerminationRequested, EOFError):
            print()
        finally:
            try:
                for renewal in self.renewals.values():
                    renewal.cancel.set()
                for name in list(self.renewals):
                    self.cancel_renewal(name)
                for name in sorted(self.jogged):
                    try:
                        self.stop_feed(self.boards[name])
                    except BoardError as error:
                        print(f'{name}: Manual feed shutdown unconfirmed. {error}', flush=True)
                        confirmed = False
                if self.heated:
                    print('Turning off heaters set in this session.', flush=True)
                    confirmed = self.turn_off([b for n, b in self.boards.items() if n in self.heated]) and confirmed
            finally:
                for board in self.boards.values():
                    board.close()
        return 0 if confirmed else 2


def usb_boards(directory=Path('/dev/serial/by-id')):
    ports = sorted(directory.glob('usb-Arduino*0042*-if00'))
    if not ports:
        raise BoardError('No Arduino Mega 2560 USB device IDs found. Check USB or specify a port with --probe.')
    boards = {}
    try:
        for port in ports:
            board, _ = probe_usb(str(port))
            name = board.device.name
            if name in boards:
                board.close()
                raise BoardError('Multiple boards have the same role. Use --config to assign each board a name.')
            boards[name] = board
    except (BoardError, KeyboardInterrupt, TerminationRequested):
        for board in boards.values():
            board.close()
        raise
    return boards


def default_config():
    module = Path(__file__).resolve()
    relative = Path('config/devices.ethernet.json')
    source = module.parents[2] / relative
    if source.is_file():
        return source
    # Catkin installs the Python module under lib and the config under share.
    for parent in module.parents:
        installed = parent / 'share/kmriiwa_thermal_demo' / relative
        if installed.is_file():
            return installed
    raise BoardError(f'Default Ethernet configuration was not found at {source}. Use --config PATH.')


def main(argv=None):
    parser = argparse.ArgumentParser(description='Arduino heater and fan control over Ethernet by default')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--usb', action='store_true', help='discover connected Arduino Mega boards over USB')
    mode.add_argument('--probe', metavar='PORT', help='identify a board at the specified serial port')
    mode.add_argument('--config', type=Path, help='load a JSON configuration for hardware devices')
    access = parser.add_mutually_exclusive_group()
    access.add_argument('--control', action='store_true', help='enable control with an explicit --usb or --config')
    access.add_argument('--read-only', action='store_true', help='read temperatures and device status only')
    options = parser.parse_args(argv)
    if options.probe and options.control:
        parser.error('--probe only reads device status')

    def terminate(signum, frame):
        raise TerminationRequested

    signal.signal(signal.SIGTERM, terminate)
    try:
        if options.probe:
            board, status = probe_usb(options.probe)
            try:
                show(board, status)
                print(json.dumps({'devices': [vars(board.device)]}, indent=2, ensure_ascii=False))
            finally:
                board.close()
            return 0
        if options.usb:
            return Console(usb_boards(), control=options.control).run()
        config = options.config if options.config else default_config()
        boards = {name: Board(device) for name, device in load_devices(config).items()}
        control = options.control if options.config else not options.read_only
        return Console(boards, control=control).run()
    except BoardError as error:
        print(f'Thermal demo exited. {error}', flush=True)
        return 2
    except (KeyboardInterrupt, TerminationRequested):
        print('Thermal demo interrupted.', flush=True)
        return 2
