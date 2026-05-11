# boat_control

ROS 2 package for testing a baseline surface-boat controller in Gazebo.

The current working version includes:
- a cascade controller for waypoint tracking
- a waypoint publisher
- a disturbance generator
- a disturbance applier for simulator testing
- a simple Gazebo planar boat model

## Current Behavior

The controller uses:
- outer PI loop on position
- inner PI loop on forward speed
- inner PID-like loop on yaw
- optional disturbance feedforward from `/disturbance/current`


## Package Contents

- [boat_control/boat_controller_node.py](/home/valeria/ros2_ws/src/boat_control/boat_control/boat_controller_node.py:1): main controller
- [boat_control/waypoint_publisher_node.py](/home/valeria/ros2_ws/src/boat_control/boat_control/waypoint_publisher_node.py:1): publishes waypoint missions
- [boat_control/disturbance_generator_node.py](/home/valeria/ros2_ws/src/boat_control/boat_control/disturbance_generator_node.py:1): publishes synthetic current disturbance
- [boat_control/disturbance_applier_node.py](/home/valeria/ros2_ws/src/boat_control/boat_control/disturbance_applier_node.py:1): applies disturbance to simulator command path
- [launch/sim_boat_controller.launch.py](/home/valeria/ros2_ws/src/boat_control/launch/sim_boat_controller.launch.py:1): main simulation launch
- [urdf/simple_boat.urdf](/home/valeria/ros2_ws/src/boat_control/urdf/simple_boat.urdf:1): simple planar boat model

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select boat_control
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/ros2_ws/install/setup.bash
```

## Launch

### 1. Baseline simulation

Runs Gazebo, spawns the simple boat, starts the controller, and publishes waypoints.

```bash
ros2 launch boat_control sim_boat_controller.launch.py
```

### 2. Disturbance enabled, no compensation

This tests how the controller behaves when disturbance is present but not compensated.

```bash
ros2 launch boat_control sim_boat_controller.launch.py enable_disturbance:=true use_disturbance_feedforward:=false
```

### 3. Disturbance enabled, with compensation

This tests disturbance feedforward compensation in the controller.

```bash
ros2 launch boat_control sim_boat_controller.launch.py enable_disturbance:=true use_disturbance_feedforward:=true
```

## Current Simulation Setup

### Boat model

The simulator uses a simple planar boat proxy with:
- `/cmd_vel` as the motion command input
- `/odom` as the odometry output

### Waypoints

The current launch publishes a smooth rectangle-like loop in the `odom` frame:

```text
(3.0, 0.0)
(5.0, 1.0)
(5.0, 4.0)
(3.0, 5.0)
(1.0, 5.0)
(0.0, 4.0)
(0.0, 1.0)
(1.0, 0.0)
```

### Current controller parameters in sim launch

```text
kp_pos = 0.6
ki_pos = 0.02
kp_u = 1.2
ki_u = 0.1
kp_yaw = 1.8
ki_yaw = 0.02
kd_yaw = 0.2
goal_tolerance = 0.8
slowdown_radius = 2.0
```

### Current disturbance parameters

Used when `enable_disturbance:=true`:

```text
ax = 0.25
ay = 0.18
wx = 0.35
wy = 0.25
```

The disturbance generator publishes:

```text
vx = ax * sin(wx * t)
vy = ay * cos(wy * t)
```

## Useful Checks

See available nodes and topics:

```bash
ros2 node list
ros2 topic list
```

Check odometry:

```bash
timeout 3 ros2 topic echo /odom
```

Check controller output:

```bash
timeout 3 ros2 topic echo /cmd_vel_controller
timeout 3 ros2 topic echo /cmd_vel
```

Check waypoints:

```bash
timeout 3 ros2 topic echo /mission/waypoints
```

Check disturbance:

```bash
timeout 3 ros2 topic echo /disturbance/current
```

## Metrics and Plots

The current workflow is:
- launch the simulation
- topic data is written to CSV automatically during the run
- after the run, read the CSV files and generate metrics plus plots

### 1. Build

```bash
cd ~/ros2_ws
colcon build --packages-select boat_control
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/ros2_ws/install/setup.bash
```

### 2. Launch with CSV logging

By default, `sim_boat_controller.launch.py` starts a metrics logger node and writes CSV files under:

```text
~/ros2_ws/src/boat_control/metrics_runs
```

Example:

```bash
ros2 launch boat_control sim_boat_controller.launch.py
```

Or with disturbance:

```bash
ros2 launch boat_control sim_boat_controller.launch.py enable_disturbance:=true use_disturbance_feedforward:=true
```

You can choose another directory:

```bash
ros2 launch boat_control sim_boat_controller.launch.py metrics_output_dir:=/tmp/my_metrics
```

The logger creates a timestamped run directory such as:

```text
~/ros2_ws/src/boat_control/metrics_runs/run_metrics
```

### 3. Generate metrics and graphics from CSV

```bash
ros2 run boat_control metrics_report ~/ros2_ws/src/boat_control/metrics_runs/run_metrics
```

This creates:
- `metrics_summary.json`
- `metrics_overview.png`
- `angular_oscillation.png`

inside:

```text
~/ros2_ws/src/boat_control/metrics_runs/run_metrics/analysis
```

### Computed metrics

The analyzer currently reports:
- trajectory RMSE to the planned path polyline
- trajectory MAE and max path error
- final position error
- planned and actual path length
- control effort using both raw sample sums and time-weighted L1 integrals
- separate effort for `/cmd_vel` and `/cmd_vel_controller`
- angular oscillation metrics: variance, RMS, sign changes
- mean and max forward speed
- disturbance magnitude statistics

## Notes

- `sim_boat_controller.launch.py` is the main launch file for testing.
- `control_baseline.launch.py` is a simpler launch without the full simulation stack.
- `control_gazebo.launch.py` is an additional Gazebo-oriented controller launch.
- The current simulator is a proxy model for controller testing, not a full hydrodynamic boat model.
