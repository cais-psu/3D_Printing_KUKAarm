# 3D Printing with a KUKA Robotic Arm

This repository contains the heater and filament feed controls for a robotic additive manufacturing cell, plus a separate circle-path demo for the KUKA KMR iiwa. The operator has confirmed that the nozzle motor can feed continuously from the Ubuntu console. The arm path and extrusion currently run independently. Synchronized deposition and printed-part quality are the next milestones.

| Component | Purpose |
| --- | --- |
| [Heated bed firmware](bed_node.ino) | Arduino Mega 2560 temperature sensing and heater control for the bed. |
| [Updated nozzle firmware](kmriiwa_thermal_demo/firmware/extruder_node/extruder_node.ino) | Hotend and fan control, timed and continuous filament feed, and RTD fault handling. Use this sketch with the Python console. |
| [Thermal console](kmriiwa_thermal_demo/README.md) | Ubuntu commands for temperatures, heater output, fan PWM, and filament feed. The console runs directly with Python 3. |
| [KMR iiwa circle demo](kmriiwa_circle_demo/README.md) | Offline path generation and ROS Noetic/MoveIt planning for a three-layer arm dry run. |

The [original nozzle sketch](extruder_node.ino) is kept for reference to the external gate and analog-rate interface. The updated sketch also accepts feed commands from the console and stops manual feed if the console stops renewing its lease for 1.5 seconds.

## Start here

For the heaters and extruder, follow the [thermal setup and flashing guide](kmriiwa_thermal_demo/README.md). With the boards connected over Ethernet, launch the console in read-only mode from the repository root:

```bash
python3 kmriiwa_thermal_demo/scripts/thermal_demo.py --read-only
```

At the prompt, use `status all` to check the two boards and `status nozzle` to see the feed state, RTD fault count, and last manual feed stop reason. For arm path work, start with the [offline circle preview](kmriiwa_circle_demo/README.md#offline-preview).

## What has been verified

- **On the equipment:** Ubuntu received status from both Arduino boards. The operator confirmed sustained motor feed from the console.
- **In software:** The nozzle firmware compiled for the Arduino Mega 2560. A 20-scenario simulation covered the feed lease, recovery window, RTD shutdown thresholds, and heater output. The circle demo has offline geometry and ROS adapter tests.
- **Still to verify:** Physical RTD fault recovery, the measured filament feed rate, and execution of the arm path on the robot. A calibrated nozzle tool center point (TCP) and extrusion synchronized to arm motion are needed before a printing trial.

The added packages include their own BSD 3-Clause notices. The nozzle update builds on the repository’s [original Arduino implementation](https://github.com/cais-psu/3D_Printing_KUKAarm/commit/0933da52dc42983a0212b5a4731fdb199820c6cf).
