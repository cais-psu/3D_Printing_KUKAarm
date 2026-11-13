# Heated Bed and Nozzle Control

The Python console reads two Arduino controllers and sets heater targets, fan PWM, and filament feed. It runs on Ubuntu with Python 3. Ethernet uses the Python standard library. USB requires `python3-serial`. The ROS workspace is optional.

## Flash the Arduino boards

The sketches target the **Arduino Mega 2560** with a **W5100 Ethernet shield**. The bed uses a 100 kΩ NTC thermistor. The nozzle sketch uses a two-wire PT1000 and a MAX31865 breakout with a 4.3 kΩ reference resistor. The sketch headers list the pin assignments. Confirm that the sensor, heater outputs, independent thermal cutoffs, and external gate match the hardware before energizing the heaters.

1. Install the **Arduino AVR Boards** core and the **Ethernet** and **Adafruit MAX31865 library** libraries in Arduino IDE. Install Adafruit BusIO if prompted.
2. Set the heater targets to zero, verify zero output, close the thermal console, and turn off the external extrusion gate before uploading.
3. Open [the updated nozzle sketch](firmware/extruder_node/extruder_node.ino). Select **Arduino Mega or Mega 2560** and the USB port for the nozzle board, then upload. For the bed, copy [bed_node.ino](../bed_node.ino) into a folder named `bed_node`, open the copy in Arduino IDE, and select the bed board's port before uploading.
4. Close the IDE serial monitor, start the console, and verify that both boards report plausible temperatures and zero heater targets. The updated nozzle reports `Continuous feed supported` in `status nozzle`.

For a build check with Arduino CLI and the same core and libraries installed, run this from `kmriiwa_thermal_demo/`:

```bash
arduino-cli compile --fqbn arduino:avr:mega:cpu=atmega2560 firmware/extruder_node
```

[Arduino CLI setup](https://arduino.github.io/arduino-cli/latest/getting-started/)

## Connect from Ubuntu

The default configuration uses the addresses set by the sketches. Put Ubuntu and both boards on the same Ethernet network. With the defaults below, use a free host address on `192.168.1.0/24`, such as `192.168.1.2/24`. If you change a board address, update the sketch and the console configuration together.

| Device | Default endpoint | Console target limit |
| --- | --- | --- |
| `bed` | `tcp://192.168.1.11:8888` | 60 °C |
| `nozzle` | `tcp://192.168.1.10:8888` | 230 °C |

The console target limits come from [devices.ethernet.json](config/devices.ethernet.json). Set limits appropriate for the installed hardware and use the filament manufacturer's temperature guidance.

Run these commands from the repository root. Start with the read-only console. After verifying the board roles and temperature feedback, use the second command to open a control session.

```bash
python3 kmriiwa_thermal_demo/scripts/thermal_demo.py --read-only
python3 kmriiwa_thermal_demo/scripts/thermal_demo.py
```

Use `--config PATH` to select an alternate configuration. Custom configurations are read-only unless you add `--control`. For USB, install `python3-serial` and use `--usb` to discover compatible Mega boards. Add `--control` to enable writes. The [serial example](config/devices.example.json) uses placeholder USB IDs and an 80 °C nozzle limit for initial heating checks. Set the correct port IDs and verify the limits for your equipment before use. Opening a serial port can reset the board.

## Console commands

| Command | What it does |
| --- | --- |
| `devices` | Shows configured devices, addresses, and target limits. |
| `status all` | Shows temperatures, targets, heater output, and board state. Use `status nozzle` for feed and RTD details. |
| `watch all` | Refreshes status each second. Press Ctrl+C to return to the prompt. |
| `temp bed 40` | Sets the bed target to 40 °C. |
| `temp nozzle 130` | Sets the nozzle target to 130 °C for a heating check. |
| `fan nozzle 35` | Sets nozzle fan PWM to 35%. The console displays PWM, with no RPM measurement. |
| `feed nozzle 1.5` | Requests continuous filament feed at 1.5 mm/s. |
| `feed nozzle 1.5 2` | Requests a two-second jog at 1.5 mm/s. |
| `feed nozzle 0` | Stops manual feed and checks the board response. |
| `off all` | Turns off heating and stops manual feed. The fan keeps its current setting. |
| `quit` | Stops manual feed and turns off the heaters set in this session before exiting. |

Manual feed accepts 0.4–10 mm/s, with an optional whole-number duration of 1–5 seconds. It requires an inactive external gate, valid temperature feedback, and a measured nozzle temperature and target of at least 170 °C. The displayed feed rate is the controller's commanded rate. Use a filament travel measurement to verify actual feed. Keep the console open for continuous feed. If the board goes 1.5 seconds without a valid keepalive, it stops manual feed.

The nozzle sketch retains the original cell's active-low gate on D18 and analog rate input on A0. The KMR iiwa circle demo does not drive these inputs.

When finished, run `feed nozzle 0`, `off all`, and `status all` to verify zero heater targets and output, then enter `quit`. If a command times out, check the board state before retrying. Check the equipment directly if the console cannot confirm shutdown.

## RTD fault response

| Condition | Firmware response |
| --- | --- |
| First reported RTD fault | Immediately pauses continuous manual feed, stops timed jogs, and sets heater PWM to zero. |
| Fresh valid sample before 500 ms | Resumes continuous feed only if the measured temperature and target are each at least 170 °C, the external gate is off, and the console lease is active. |
| Valid sample at or after 500 ms | Ends the paused feed session with `sensor_stale`. Starting again requires a new feed command. |
| Five consecutive RTD faults | Ends the manual feed session with `rtd_fault`. |
| Ten consecutive RTD faults | Clears the nozzle target and latches heating off until reset. |

Heater output stays at zero during faulted samples. With a valid reading, heater control can resume if the board has not latched an abort. The firmware also checks out-of-range temperatures, sudden temperature jumps, overtemperature, and failure to heat under high output. The external analog feed is inhibited during invalid readings and follows its gate again once temperature feedback passes the checks. Use `status nozzle` to see the cumulative RTD fault count and the last manual feed stop reason. Investigate repeated faults before resetting the controller.

See [COMMANDS.txt](COMMANDS.txt) for a copyable console reference.
