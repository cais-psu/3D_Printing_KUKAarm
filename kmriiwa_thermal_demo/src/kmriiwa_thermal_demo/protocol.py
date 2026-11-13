"""Read and control the thermal outputs of the two Arduino board types."""

from dataclasses import dataclass
import errno
import ipaddress
import json
import math
import os
from pathlib import Path
import re
import socket
from threading import Lock
import time
from typing import Dict, Optional, Tuple


FIRMWARE_MAX_C = {"bed": 110.0, "nozzle": 300.0}
MIN_JOG_RATE = 0.4
MAX_JOG_RATE = 10.0
MAX_JOG_SECONDS = 5
COLD_EXTRUDE_C = 170.0
MAX_LINE = 256
TCP_CLOSE_INTERVAL = 0.30
_DECIMAL = re.compile(r"-?[0-9]+(?:\.[0-9]+)?")
_DNS_LABEL = re.compile(r"[A-Za-z0-9](?:[A-Za-z0-9-]{0,61}[A-Za-z0-9])?")


class BoardError(RuntimeError):
    """A configuration, protocol, validation, or communication error."""


class Rejected(BoardError):
    """The board explicitly replied ERR."""


class UnconfirmedWrite(BoardError):
    """The command was attempted, but the board outcome could not be confirmed."""


@dataclass(frozen=True)
class Device:
    name: str
    kind: str
    endpoint: str
    max_c: float


@dataclass(frozen=True)
class Status:
    temp_c: float
    target_c: float
    heater_pct: float
    fan_raw: Optional[int]
    gate: Optional[bool]
    feed_request: Optional[float]
    rtd_error: Optional[str]
    state: str
    raw: str
    jog_rate: Optional[float] = None
    feed_lease: bool = False
    stop_reason: Optional[str] = None
    pause_reason: Optional[str] = None

    @property
    def feed_paused(self) -> bool:
        return self.pause_reason == "rtd_fault"


def _number(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise BoardError(f"{label} must be a finite number.")
    try:
        number = float(value)
    except OverflowError:
        raise BoardError(f"{label} must be a finite number.") from None
    if not math.isfinite(number):
        raise BoardError(f"{label} must be a finite number.")
    return number


def _endpoint(value):
    if not isinstance(value, str):
        raise BoardError("Use tcp://host:port or serial:/dev/... for the device address.")
    if value.startswith("serial:/dev/"):
        path = value[len("serial:"):]
        parts = path.split("/")[2:]
        if parts and all(re.fullmatch(r"[A-Za-z0-9_.:+-]+", part) and part not in (".", "..")
                         for part in parts):
            return "serial", path
    if value.startswith("tcp://"):
        match = re.fullmatch(r"tcp://([A-Za-z0-9.-]+):([0-9]{1,5})", value)
        if match:
            host = match[1].lower()
            if host.endswith("."):
                host = host[:-1]
            port = int(match[2])
            if re.fullmatch(r"[0-9.]+", host):
                try:
                    ipaddress.IPv4Address(host)
                except ValueError:
                    raise BoardError("Invalid device IPv4 address.") from None
            elif not host or len(host) > 253 or not all(_DNS_LABEL.fullmatch(part) for part in host.split(".")):
                raise BoardError("Invalid device hostname.")
            if 1 <= port <= 65535:
                return "tcp", host, port
    raise BoardError("Invalid device address. Use 'tcp://host:port' or 'serial:/dev/...'.")


def _validate_device(device):
    name = device.name
    if (not isinstance(name, str) or not name or name == "all" or not name.isprintable()
            or any(character.isspace() for character in name)):
        raise BoardError("Use a printable device name without spaces. The name 'all' is reserved.")
    kind = device.kind
    if not isinstance(kind, str) or kind not in FIRMWARE_MAX_C:
        raise BoardError("Device kind must be bed or nozzle.")
    maximum = _number(device.max_c, f"{name} maximum target temperature")
    if not 0 < maximum <= FIRMWARE_MAX_C[kind]:
        raise BoardError(f"{name} maximum target temperature must be above zero and at most {FIRMWARE_MAX_C[kind]:g} °C.")
    return _endpoint(device.endpoint)


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise BoardError(f"Duplicate configuration field {key!r}.")
        result[key] = value
    return result


def load_devices(path) -> Dict[str, Device]:
    try:
        document = json.loads(Path(path).read_text(encoding="utf-8"), object_pairs_hook=_unique_object)
    except json.JSONDecodeError as error:
        raise BoardError(f"Invalid configuration JSON at line {error.lineno}, column {error.colno}.") from error
    except UnicodeError as error:
        raise BoardError("Device configuration must use UTF-8.") from error
    except OSError as error:
        raise BoardError(f"Cannot read device configuration. {_io_detail(error)}") from error
    if (not isinstance(document, dict) or set(document) != {"devices"}
            or not isinstance(document["devices"], list) or not document["devices"]):
        raise BoardError("Configuration must contain a nonempty devices list.")
    devices = {}
    endpoints = set()
    for item in document["devices"]:
        if not isinstance(item, dict) or set(item) != {"name", "kind", "endpoint", "max_c"}:
            raise BoardError("Each device requires exactly four fields: name, kind, endpoint, and max_c.")
        device = Device(**item)
        endpoint = _validate_device(device)
        if device.name in devices:
            raise BoardError(f"Duplicate device name {device.name!r}.")
        if endpoint[0] == "serial":
            endpoint = "serial", os.path.realpath(endpoint[1])
        if endpoint in endpoints:
            raise BoardError(f"Duplicate device address {device.endpoint!r}.")
        endpoints.add(endpoint)
        devices[device.name] = device
    return devices


def _decimal(fields, key, low=-math.inf, high=math.inf):
    text = fields[key]
    if not _DECIMAL.fullmatch(text):
        raise BoardError(f"Invalid numeric value {text!r} for STATUS field {key}.")
    value = float(text)
    if not math.isfinite(value) or not low <= value <= high:
        raise BoardError(f"STATUS field {key} is out of range.")
    return value


def _integer(fields, key, high):
    text = fields[key]
    if not re.fullmatch(r"[0-9]+", text) or int(text) > high:
        raise BoardError(f"Invalid integer value {text!r} for STATUS field {key}.")
    return int(text)


def parse_status(line: str, kind: str) -> Status:
    if not isinstance(kind, str) or kind not in FIRMWARE_MAX_C:
        raise BoardError("STATUS kind must be bed or nozzle.")
    if not isinstance(line, str):
        raise BoardError("STATUS reply must be one line of ASCII text.")
    raw = line.rstrip("\r\n")
    if len(raw) > MAX_LINE or not raw.isascii() or not raw.isprintable():
        raise BoardError("STATUS reply contains invalid characters or exceeds the length limit.")
    prefix, marker, state = raw.partition(" STATE=")
    if not marker or not state or not prefix.startswith("T="):
        raise BoardError("STATUS reply is missing temperature or board state.")
    fields = {}
    for token in prefix.split():
        key, delimiter, value = token.partition("=")
        if not delimiter or not value or key in fields:
            raise BoardError("STATUS reply contains a malformed or duplicate field.")
        fields[key] = value
    expected = {"T", "SET", "DUTY"} if kind == "bed" else {"T", "SET", "PWM", "GATE", "FEED", "FAN"}
    optional = {"RTDERR", "JOG", "ACTIVE", "LEASE", "STOP_REASON", "PAUSE"} if kind == "nozzle" else set()
    if not expected <= fields.keys() or fields.keys() - expected - optional:
        raise BoardError(f"STATUS reply does not match the configured {kind} board.")
    temp = _decimal(fields, "T")
    target = _decimal(fields, "SET", 0, FIRMWARE_MAX_C[kind])
    if kind == "bed":
        return Status(temp, target, _decimal(fields, "DUTY", 0, 100), None, None, None, None, state, raw)
    rtd = fields.get("RTDERR")
    if rtd is not None and (not re.fullmatch(r"[0-9]+(?:/0x[0-9A-Fa-f]{1,2})?", rtd)
                            or int(rtd.split("/")[0]) > 65535):
        raise BoardError(f"Invalid RTD fault field {rtd!r}.")
    jog_rate = None
    if "JOG" in fields or "ACTIVE" in fields:
        if fields.get("JOG") != "1" or "ACTIVE" not in fields:
            raise BoardError("Incomplete nozzle jog status.")
        jog_rate = _decimal(fields, "ACTIVE", 0, MAX_JOG_RATE)
    feed_lease = "LEASE" in fields
    if feed_lease and (fields["LEASE"] != "1" or jog_rate is None):
        raise BoardError("Incomplete continuous feed status.")
    stop_reason = fields.get("STOP_REASON")
    if stop_reason is not None and not re.fullmatch(r"[a-z][a-z0-9_]{0,31}", stop_reason):
        raise BoardError("Invalid manual feed stop reason.")
    pause_reason = fields.get("PAUSE")
    if pause_reason is not None and (pause_reason not in ("none", "rtd_fault") or not feed_lease):
        raise BoardError("Invalid continuous feed pause status.")
    if pause_reason == "rtd_fault" and (jog_rate != 0 or stop_reason not in (None, "none")):
        raise BoardError("Paused continuous feed must report zero speed and no stop reason.")
    return Status(temp, target, _integer(fields, "PWM", 255) * 100.0 / 255.0,
                  _integer(fields, "FAN", 255), bool(_integer(fields, "GATE", 1)),
                  _decimal(fields, "FEED", 0, 10), rtd, state, raw, jog_rate, feed_lease, stop_reason, pause_reason)


def _remaining(deadline):
    remaining = deadline - time.monotonic()
    if remaining <= 0:
        raise TimeoutError()
    return remaining


def _io_detail(error):
    if isinstance(error, socket.gaierror):
        return "Could not resolve the hostname."
    if isinstance(error, (TimeoutError, socket.timeout)) or type(error).__name__ == "SerialTimeoutException":
        return "Operation timed out."
    messages = {
        errno.EACCES: "Access denied.",
        errno.EPERM: "Operation not permitted.",
        errno.ENOENT: "Path or device not found.",
        errno.ENODEV: "Device disconnected or not found.",
        errno.EBUSY: "Device is in use.",
        errno.EAGAIN: "Device temporarily unavailable.",
        errno.ECONNREFUSED: "Connection refused.",
        errno.ECONNRESET: "Peer reset the connection.",
        errno.ETIMEDOUT: "Operation timed out.",
        errno.EHOSTUNREACH: "Host unreachable.",
        errno.ENETUNREACH: "Network unreachable.",
        errno.ENETDOWN: "Network interface is down.",
        errno.EPIPE: "Connection closed.",
        errno.EIO: "Device I/O failed.",
    }
    code = getattr(error, "errno", None)
    if code in messages:
        return messages[code]
    return f"I/O failed (system error {code})." if code is not None else "I/O failed."


def _write_failure(error, expect_status):
    kind = BoardError if expect_status else UnconfirmedWrite
    return kind(str(error))


def _decode(line):
    try:
        text = bytes(line).decode("ascii").rstrip("\r")
    except UnicodeDecodeError as error:
        raise BoardError("Board reply contains non-ASCII characters.") from error
    if text and not text.isprintable():
        raise BoardError("Board reply contains control characters.")
    return text


class Board:
    """One board; TCP per request, or a persistent optional pyserial connection."""

    def __init__(self, device: Device, timeout: float = 1.5):
        self.device = device
        self._address = _validate_device(device)
        self._timeout = _number(timeout, "Timeout")
        if not 0 < self._timeout <= 60:
            raise BoardError("Timeout must be above zero and at most 60 seconds.")
        self._serial = None
        self._tcp_last_close = None
        self._lock = Lock()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def close(self):
        with self._lock:
            port, self._serial = self._serial, None
            if port is not None:
                try:
                    port.close()
                except OSError as error:
                    raise BoardError(f"{self.device.name}: Could not close the serial port. {_io_detail(error)}") from error

    def status(self) -> Status:
        return self._request("STATUS", expect_status=True)

    def set_temp(self, c):
        value = _number(c, "Temperature")
        if not 0 <= value <= self.device.max_c:
            raise BoardError(f"{self.device.name}: Temperature must be between 0 and {self.device.max_c:g} °C.")
        self._request(f"SET {value}")

    def set_fan(self, percent):
        if self.device.kind != "nozzle":
            raise BoardError("Fan control requires a nozzle board.")
        if isinstance(percent, bool) or not isinstance(percent, int) or not 0 <= percent <= 100:
            raise BoardError("Fan setting must be an integer between 0 and 100 percent.")
        self._request(f"FAN {percent}")

    def off(self):
        self._request("OFF")

    def jog(self, rate, seconds):
        speed = self._feed_speed(rate)
        if type(seconds) is not int or not 1 <= seconds <= MAX_JOG_SECONDS:
            raise BoardError(f"Feed duration must be a whole number from 1 to {MAX_JOG_SECONDS} seconds.")
        self._request(f"JOG {speed:.3f} {seconds * 1000}")

    def start_feed(self, rate, token):
        speed = self._feed_speed(rate)
        self._check_feed_token(token)
        self._request(f"FEED {speed:.3f} {token}")

    def keep_feed(self, token):
        if self.device.kind != "nozzle":
            raise BoardError("Manual feed requires a nozzle board.")
        self._check_feed_token(token)
        self._request(f"KEEP {token}")

    def _feed_speed(self, rate):
        if self.device.kind != "nozzle":
            raise BoardError("Manual feed requires a nozzle board.")
        speed = _number(rate, "Feed speed")
        if not MIN_JOG_RATE <= speed <= MAX_JOG_RATE:
            raise BoardError(f"Feed speed must be {MIN_JOG_RATE:g}–{MAX_JOG_RATE:g} mm/s.")
        return speed

    @staticmethod
    def _check_feed_token(token):
        if not isinstance(token, str) or not re.fullmatch(r"[0-9A-F]{8}", token) or token == "00000000":
            raise BoardError("Invalid continuous feed token.")

    def stop_jog(self):
        if self.device.kind != "nozzle":
            raise BoardError("Manual feed requires a nozzle board.")
        self._request("STOP")

    def _request(self, command, expect_status=False):
        payload = command.encode("ascii") + b"\n"
        operation = command.split()[0]
        label = f"{self.device.name} {operation}"
        try:
            with self._lock:
                if self._address[0] == "tcp":
                    return self._tcp_reply(payload, expect_status)
                deadline = time.monotonic() + self._timeout
                return self._serial_reply(payload, deadline, expect_status)
        except Rejected as error:
            raise Rejected(f"{label}: {error}") from error
        except UnconfirmedWrite as error:
            raise UnconfirmedWrite(f"{label}: Command outcome unknown. {error}") from error
        except BoardError as error:
            raise BoardError(f"{label}: {error}") from error
        except OSError as error:
            raise BoardError(f"{label}: {_io_detail(error)}") from error

    def _interpret(self, line, expect_status):
        if line == "ERR" or line.startswith("ERR "):
            raise Rejected(f"Board rejected the command ({line}).")
        if line.startswith("ABORT:"):
            raise BoardError(f"Board reported a fault ({line}).")
        if expect_status:
            return parse_status(line, self.device.kind)
        if line != "OK":
            raise BoardError(f"Expected an OK reply, received {line!r}.")
        return None

    def _tcp_reply(self, payload, expect_status):
        if self._tcp_last_close is not None:
            pause = self._tcp_last_close + TCP_CLOSE_INTERVAL - time.monotonic()
            if pause > 0:
                time.sleep(pause)
        deadline = time.monotonic() + self._timeout
        try:
            client = socket.create_connection(self._address[1:], timeout=_remaining(deadline))
        except OSError as error:
            raise BoardError(f"Could not establish a TCP connection. {_io_detail(error)}") from error
        try:
            with client:
                try:
                    client.settimeout(_remaining(deadline))
                except OSError as error:
                    raise BoardError(f"Could not prepare the TCP send. {_io_detail(error)}") from error
                try:
                    client.sendall(payload)
                except OSError as error:
                    failure = _write_failure(f"TCP send failed. {_io_detail(error)}", expect_status)
                    raise failure from error
                try:
                    line = self._read_tcp_line(client, deadline)
                    return self._interpret(line, expect_status)
                except Rejected:
                    raise
                except OSError as error:
                    failure = _write_failure(f"TCP reply could not be read. {_io_detail(error)}", expect_status)
                    raise failure from error
                except BoardError as error:
                    raise _write_failure(error, expect_status) from error
        finally:
            self._tcp_last_close = time.monotonic()

    def _read_tcp_line(self, client, deadline):
        line = bytearray()
        while True:
            client.settimeout(_remaining(deadline))
            chunk = client.recv(MAX_LINE + 2 - len(line))
            if not chunk:
                raise BoardError("TCP connection closed before a complete reply arrived.")
            before, newline, _ = chunk.partition(b"\n")
            line.extend(before)
            if len(line) > MAX_LINE + (1 if line.endswith(b"\r") else 0):
                raise BoardError("Board reply exceeds the length limit.")
            if newline:
                return _decode(line)

    def _open_serial(self, deadline):
        try:
            import serial
        except ImportError as error:
            raise BoardError("Serial transport requires pyserial (python3 -m pip install pyserial)") from error
        options = {"baudrate": 115200, "timeout": min(0.05, _remaining(deadline)),
                   "write_timeout": _remaining(deadline)}
        if os.name == "posix":
            options["exclusive"] = True
        try:
            port = serial.Serial(port=None, **options)
            port.dtr = False
            port.port = self._address[1]
            port.open()
            return port
        except (AttributeError, TypeError, ValueError) as error:
            raise BoardError("Could not configure the serial port with pyserial.") from error
        except OSError as error:
            raise BoardError(f"Could not open the serial port. {_io_detail(error)}") from error

    def _serial_reply(self, payload, deadline, expect_status, detect_kind=False):
        if self._serial is None:
            self._serial = self._open_serial(deadline)
        port = self._serial
        try:
            port.reset_input_buffer()
            port.timeout = min(0.05, _remaining(deadline))
            port.write_timeout = _remaining(deadline)
        except OSError as error:
            raise BoardError(f"Could not prepare the serial send. {_io_detail(error)}") from error
        try:
            if port.write(payload) != len(payload):
                raise BoardError("Serial send did not write the complete command.")
        except OSError as error:
            raise _write_failure(f"Serial send failed. {_io_detail(error)}", expect_status) from error
        except BoardError as error:
            raise _write_failure(error, expect_status) from error
        try:
            return self._read_serial_reply(port, deadline, expect_status, detect_kind)
        except Rejected:
            raise
        except OSError as error:
            if detect_kind and isinstance(error, (TimeoutError, socket.timeout)):
                raise
            raise _write_failure(f"Serial reply could not be read. {_io_detail(error)}", expect_status) from error
        except BoardError as error:
            raise _write_failure(error, expect_status) from error

    def _read_serial_reply(self, port, deadline, expect_status, detect_kind):
        line = bytearray()
        while True:
            remaining = _remaining(deadline)
            if port.timeout > remaining:
                port.timeout = remaining
            character = port.read(1)
            if not character:
                continue
            if character != b"\n":
                line.extend(character)
                if len(line) > MAX_LINE + (1 if line.endswith(b"\r") else 0):
                    raise BoardError("Board reply exceeds line length limit")
                continue
            reply = _decode(line)
            line.clear()
            if not reply or reply.startswith(("listening on ", "*")):
                continue
            if reply in ("bed node starting", "extruder node starting"):
                expected = "bed node starting" if self.device.kind == "bed" else "extruder node starting"
                if not detect_kind and reply != expected:
                    raise BoardError(f"Boot banner does not match the configured {self.device.kind} board")
                continue
            # Clearing the receive buffer can leave the tail of a periodic
            # STATUS line on the wire. Wait for a complete line on reads.
            if expect_status and not reply.startswith("T=") and (
                    " STATE=" in reply or re.match(r"^(?:STATE|TATE|ATE|TE|E)?=", reply)
                    or reply in ("ok", "k")):
                continue
            if detect_kind and reply.startswith("T="):
                fields = {token.partition("=")[0] for token in reply.partition(" STATE=")[0].split()}
                kind = "bed" if "DUTY" in fields else "nozzle"
                return kind, parse_status(reply, kind)
            if not expect_status and reply.startswith("T="):
                parse_status(reply, self.device.kind)
                continue
            return self._interpret(reply, expect_status)


def probe_usb(path, timeout: float = 3.0) -> Tuple[Board, Status]:
    """Identify the board with STATUS and return the already open serial session."""
    endpoint = "serial:" + str(path)
    board = Board(Device("usb_probe", "bed", endpoint, 60), timeout)
    started = time.monotonic()
    deadline = started + board._timeout
    try:
        try:
            kind, status = board._serial_reply(b"STATUS\n", started + board._timeout / 2,
                                               expect_status=True, detect_kind=True)
        except (TimeoutError, socket.timeout):
            # A Mega can reboot when the serial device first opens. STATUS is safe to repeat.
            kind, status = board._serial_reply(b"STATUS\n", deadline, expect_status=True, detect_kind=True)
        board.device = Device(kind, kind, endpoint, 60 if kind == "bed" else 80)
        return board, status
    except (OSError, BoardError) as error:
        try:
            board.close()
        except BoardError:
            pass
        detail = _io_detail(error) if isinstance(error, OSError) else str(error)
        raise BoardError(f"{path} STATUS probe: {detail}") from error
