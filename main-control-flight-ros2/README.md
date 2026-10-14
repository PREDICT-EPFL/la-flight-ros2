# main-control-flight-ros2

Main control node for the flight systems: the fixed-wing controller for (multi-kite) airborne wind energy (AWE) systems, including an automated pumping mission. Joystick commands are used for manual flight and to activate the controller.

## Build dependencies

- ROS 2 packages: `rclcpp`, `geometry_msgs`, `sensor_msgs`, `diagnostic_msgs`
- Libraries: Eigen3
- `la-common`: `main-control-common` (linked into `lib/main-control-flight/lib/`)

## Code structure

| Path | Description |
|---|---|
| `src/`, `include/` | **ROS 2 wrapper**: the node `fixedwing_control_node2` with parameters, topics, and callbacks around the controller. |
| `lib/main-control-flight/` | **ROS-free** controller: the fixed-wing controller with its control modes and the mission commander (`MissionCommander.hpp`). |
| `lib/main-control-flight/lib/awe-control` | **ROS-free** AWE guidance and control library (vendored). |
| `lib/main-control-flight/lib/main-control-common` | **ROS-free** base classes and utilities (symlink to `la-common/main-control-common`). |
| `tests/` | Small test executables: ROS 2 node (`tests/`) and ROS-free controller (`lib/main-control-flight/tests/`). |

## Usage

```
ros2 run main_control_flight_ros2 fixedwing_control_node2 --ros-args -p control_mode:=mawe_latlon
```

The node reads the vehicle states from the simulator (`simulation-flight-ros2`). For manual commands, start `ros2 run joy joy_node` as well.

Note: the node is currently built for the simulator interface (`/sim/...` topics). The MAVROS/Pixhawk interface classes exist in the code, but the node does not select them.

Parameters of `fixedwing_control_node2` (see [FixedwingControlRos2.cpp](src/FixedwingControlRos2.cpp)):

| Parameter | Default | Description |
|---|---|---|
| `node_rate` | `100.0` | Control loop rate in Hz. |
| `control_mode` | `mawe_manual` | Control mode, see below. Only read if no mission is selected. |
| `mission_mode` | empty | `mawe` runs the automated multi-AWE mission (see below). Empty runs no mission, and `control_mode` is used. |
| `control_kites` | `2` | Number of kites to control, up to 3. |
| `windFrom_deg` | `160.0` | Nominal direction the wind is coming from, in degrees. |
| `activate` | `false` | Start with the controller active, as if the activation button was pressed. |

## Control modes

Selected with the parameter `control_mode` if no mission is set. If the name is not recognized, `manual` is used.

| Mode | Description |
|---|---|
| `mawe_manual` | Manual AWE mode: the joystick sets the lift direction of the kites (with trim on the directional pad), the collective lift and the throttle, and the tether lengths. |
| `mawe_climb` | AWE climb. |
| `mawe_latlon` | Position control on the wind-dome sphere by latitude and longitude, used for the pumping cycle. |
| `mawe_descent` | Not implemented yet, behaves like `mawe_climb`. |
| `position`, `velocity`, `attitude`, `rates`, `manual` | General fixed-wing modes with decreasing level of automation: from position reference down to direct manual commands. |

## Mission

With `mission_mode:=mawe`, the mission commander switches between the control modes automatically. The phases, published as the mission phase number on `/status/main_control`, are:

| Phase | Description |
|---|---|
| `0` | AWE climb |
| `1` | Prepare reel-out |
| `2` | Pumping cycle: reel-out |
| `3` | Prepare reel-in |
| `4` | Reel-in |
| `5` | Prepare descent |
| `6` | Descent |
| `7` | Landing |
| `8` | Mission end |

## Communication interface

| Topic | Type | Direction |
|---|---|---|
| `/sim/state`, `/sim/state2`, `/sim/state3` | `sensor_msgs/MultiDOFJointState` | in (kites 1 to 3) |
| `/sim/state_conpoint` | `sensor_msgs/MultiDOFJointState` | in (tether connection point) |
| `/joy` | `sensor_msgs/Joy` | in |
| `/sim/set/control`, `/sim/set/control2`, `/sim/set/control3` | `sensor_msgs/Joy` | out (kites 1 to 3: throttle, elevator, rudder, aileron, aux0, aux1) |
| `/status/main_control` | `diagnostic_msgs/DiagnosticArray` | out (mission phase) |
| `/diag/LFc`, `/diag/LFc_cond`, `/diag/diag3` | `geometry_msgs/Vector3Stamped` | out (diagnostics) |
