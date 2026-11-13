# KMR iiwa circular dry run

This ROS Noetic package plans stacked circles for a KMR iiwa arm through the existing MoveIt and Sunrise Java interfaces. The default launch previews the plan in RViz. The optional execution mode sends a single arm trajectory. Bed heating, nozzle heating, filament feed, and mobile base control are outside the dry run. The package passed its offline geometry and ROS adapter tests and built on Ubuntu with ROS Noetic. Physical arm execution is still pending.

## Offline preview

From this package directory, use Python 3 to generate the default path and run the tests. Neither command requires ROS or a hardware connection.

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$PWD/src" python3 scripts/preview_circle.py --output /tmp/kmriiwa_circle_preview
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$PWD/src" python3 -m unittest discover -s tests -v
```

The preview writes `offline_preview.svg`, `offline_path.csv`, and `offline_summary.json`. It starts at the origin with identity orientation. The offline result verifies geometry. MoveIt performs inverse kinematics and collision checking against the robot model and planning scene in the ROS workflow.

## Preview the measured bed

The [bed preview configuration](config/bed_preview.json) uses the four estimated nozzle tip positions in KUKA World, based on the Rev F bracket CAD offset. The center is **X −339.525 mm, Y −425.000 mm**. Three 40 mm diameter circles follow the fitted plane at 20, 25, and 30 mm along its upward normal. The footprint gives about 121 mm of nominal edge clearance. The fit gives about 32.05 mm of World Z at the center and 0.29 mm of height variation across the circle. These are estimates based on the taught points and CAD offset. The layer spacing and clearance are for preview.

Generate the bed outline, TCP path CSV, and numerical summary from the package directory:

```bash
PYTHONPATH="$PWD/src" python3 scripts/preview_circle.py --bed config/bed_preview.json --output /tmp/kmriiwa_bed_circle
```

With ROS configured as below, use the dedicated launch to preview in MoveIt. The TCP path shows the circles and layer changes. The commanded end-effector and joint paths also show the Cartesian approach from the current pose. A partial or invalid plan is rejected. Bed preview execution is disabled in both the launch file and the node.

```bash
roslaunch kmriiwa_circle_demo bed_circle_preview.launch
```

This preview assumes smartPAD World matches `kmriiwa_link_0` and physical Flange matches `kmriiwa_link_ee`. Offline forward kinematics of the supplied joint readings agrees with the four taught flange poses to about 0.1 mm using the KMR iiwa 14 model. Verify this against the loaded robot description and a stationary smartPAD/ROS pose pair before using measured targets for motion. The planning scene must also represent the plate, nozzle assembly, and nearby fixtures.

For a print, record the calibrated flange-to-nozzle TCP and print orientation. With that tool selected, measure three non-collinear surface points in World at the same known gauge gap, or provide a calibrated bed Base pose in World. Use reachable points around the intended print area and check the gap near the circle center. Record gauge thickness and bed temperature. First-layer gap, layer height, nozzle diameter, and filament diameter are needed for deposition. This preview does not command heaters or extrusion. The current Java driver's joint spline timing also cannot be used to schedule extrusion against MoveIt's timestamps. [KUKA Sunrise.OS calibration manual](https://www.oir.caltech.edu/twiki_oir/pub/Palomar/ZTF/KUKARoboticArmMaterial/KUKA_SunriseOS_111_SI_en.pdf)

## Default path

| Parameter | Default |
| --- | --- |
| Reference frame | `kmriiwa_link_0` |
| End-effector link | `kmriiwa_link_ee` |
| Radius | 20 mm |
| Plane | `xy` |
| Layers | 3, with 5 mm between layers along `+Z` |
| Samples | 60 per circle |
| Tool center point (TCP) offset | Zero, so the end-effector link serves as the TCP |

The live plan starts at the current TCP, which is the point of maximum `X` on the first circle. It preserves the current tool orientation. The default path spans `X = −40…0 mm`, `Y = −20…+20 mm`, and `Z = 0…+10 mm` relative to the start, and ends 10 mm directly above it. The 5 mm spacing is for visualizing the dry run.

Pass `radius:=0.015 layers:=2` to change the radius and layer count. Launch arguments `radius`, `layers`, `layer_step`, and `plane` take precedence over their YAML values. The other settings, including `tcp_offset_xyz` and `tcp_offset_rpy`, are in [config/demo.yaml](config/demo.yaml). The TCP offset describes the end-effector-to-TCP transform in the end-effector frame. Translation is in metres and roll, pitch, and yaw are in radians using `Rz(yaw) Ry(pitch) Rx(roll)`. A physical nozzle requires a measured TCP and appropriate collision geometry.

## Build and preview with ROS

Place this package under `src/` in a Catkin workspace containing the KMR iiwa ROS stack and MoveIt configuration. From the workspace root, build the package and source its environment.

```bash
source /opt/ros/noetic/setup.bash
catkin_make --pkg kmriiwa_circle_demo
source devel/setup.bash
```

Use the ROS master configured for your robot and Sunrise application. The preview requires the robot description, joint feedback, and the MoveIt forward kinematics and Cartesian planning services. In the standard KMR iiwa setup, `kmriiwa_bringup.launch` and `move_group.launch` provide these interfaces alongside the existing Sunrise application. [KMR iiwa ROS stack](https://github.com/stoic-roboticist/kmriiwa_ros_stack) · [KMR iiwa Sunrise application](https://github.com/stoic-roboticist/kmriiwa_ros_java)

```bash
roslaunch kmriiwa_circle_demo circle_demo.launch
```

In RViz, set the fixed frame to `kmriiwa_link_0` and add the two Path topics `/kmriiwa_circle_demo/tcp_path` and `/kmriiwa_circle_demo/ee_path`. Set the MotionPlanning display's planned trajectory topic to `/kmriiwa_circle_demo/display_planned_path`. The two Path displays coincide with the default zero TCP offset. Press Ctrl+C to close the preview.

## Optional on-site dry run

After reviewing the full plan and confirming clearance, close the preview. The on-site operator can enable execution explicitly. This command replans from the current arm state and sends one trajectory.

```bash
roslaunch kmriiwa_circle_demo circle_demo.launch execute:=true keep_alive:=false
```

The node requires complete Cartesian planning and rejects trajectories above the Java backend's limit of 500 actual joint trajectory points. The default maximum step between consecutive joint positions is 0.25 rad. Execution also requires fresh seven-joint feedback, a ready robot state, and start drift no greater than 0.02 rad per joint.

The upstream Java backend converts joint positions to a `SplineJP` and applies a relative joint speed of 0.15. MoveIt timing and scaling describe the plan displayed in RViz. Set the appropriate robot speed at the controller and use the native robot stop if needed. Physical stopping through Ctrl+C, MoveIt stop, or ROS action cancellation remains unverified. The node records an unknown execution outcome on errors after submission and leaves any retry to the operator. [Java trajectory execution](https://github.com/stoic-roboticist/kmriiwa_ros_java/blob/c0547d5db79d8c2b22ff0a8957ed2f4949c5500c/src/uk/ac/liverpool/lrcfmd/kmriiwa/robot/LBRCommander.java) · [Java action cancellation](https://github.com/stoic-roboticist/kmriiwa_ros_java/blob/c0547d5db79d8c2b22ff0a8957ed2f4949c5500c/src/uk/ac/liverpool/lrcfmd/kmriiwa/nodes/ActionServerNode.java)

ROS runs write `report.json`, desired TCP and end-effector CSV files, and `joint_trajectory.json` to a timestamped directory under `~/.ros/kmriiwa_circle_demo/`. The report field `planned_duration_s` describes MoveIt timing. Physical trajectory duration, speed, and path accuracy remain to be measured.
