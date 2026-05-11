# Seabed Mapping Experiment

This package contains the seabed mapping part of the AMR project. The experiment tests whether a small surface boat can build a bathymetric point-cloud map from a simulated downward multibeam sonar while following a planned survey route.

## Goal

Build a local seabed map from:

- boat odometry: `/odom`
- simulated multibeam-like sonar cloud: `/sonar/downward/points`

The mapper publishes the accumulated map as:

- `/seabed/map_points` (`sensor_msgs/msg/PointCloud2`, frame `odom`)

The current implementation is an odometry-based mapper, not loop-closing SLAM. This is intentional for the first experiment: it isolates the sonar mapping pipeline before adding fused odometry or seabed localization.

## World And Sensor

The world uses real USGS Missouri River I-64 bathymetry:

- source CSV: `data/missouri_i64/site-23_MissouriRiver_I-64_2020-08_CUBE-uncert.csv`
- world: `worlds/missouri_i64_featured_seabed.world`
- mesh: `models/missouri_i64_featured/meshes/missouri_i64_featured.dae`

The generated mesh is scaled by `0.5 0.5 0.5` in Gazebo so the map size better matches a small boat. The sensor is attached to `sonar_link` in `boat_control/urdf/simple_boat.urdf` and publishes a downward fan as `PointCloud2`.

## Experiment Variants

### Variant A: GT Odometry Mapping

Use Gazebo odometry directly:

- mapper input: `/odom`
- controller input: `/odom`
- purpose: baseline map quality when localization is nearly perfect

### Variant B: Fused Odometry Mapping

After the CV module is available, replace `/odom` with `/odometry/filtered`:

- mapper input: `/odometry/filtered`
- controller input: `/odometry/filtered`
- purpose: measure how localization drift affects the seabed map

This variant should use the same world, route, controller gains, and sonar settings as Variant A.

## Run

Inside the ROS container:

```bash
cd /home/fabian/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select boat_control seabed_slam --symlink-install
source install/setup.bash
ros2 launch seabed_slam seabed_mapping_experiment.launch.py
```

With current-like disturbance enabled:

```bash
ros2 launch seabed_slam seabed_mapping_experiment.launch.py enable_disturbance:=true
```

For a headless run without Gazebo client:

```bash
ros2 launch seabed_slam seabed_mapping_experiment.launch.py gui:=false
```

## Straight-Line RMSE Experiment

Use this experiment for the first quantitative check. The boat starts near the middle of the map, drives one straight line to `(22, 0)`, stops at the final waypoint, and the evaluator prints RMSE against the source bathymetry.

```bash
ros2 launch seabed_slam seabed_line_rmse_experiment.launch.py
```

Headless run without Gazebo GUI and RViz:

```bash
ros2 launch seabed_slam seabed_line_rmse_experiment.launch.py gui:=false rviz:=false
```

The evaluator prints lines like:

```text
seabed_map_metrics points=... valid_points=... rmse_m=... mae_m=... bias_m=... coverage_cells=... coverage_percent=...
```

For the straight-line run, use the last printed `seabed_map_metrics` line after the boat reaches the final waypoint.

## Seabed Monte Carlo Localization

The localization experiment is split into two stages.

First, record a reference patch inside the useful map area:

```bash
ros2 launch seabed_slam seabed_reference_record.launch.py
```

Headless:

```bash
ros2 launch seabed_slam seabed_reference_record.launch.py gui:=false
```

This writes:

```text
src/seabed_slam/data/missouri_i64/recorded_reference_map.csv
```

Then run Monte Carlo localization. By default it uses the GT bathymetry as an ideal reference:

```bash
ros2 launch seabed_slam seabed_mcl_experiment.launch.py
```

To localize against the recorded reference map:

```bash
ros2 launch seabed_slam seabed_mcl_experiment.launch.py \
  reference_csv:=/home/fabian/ros2_ws/src/seabed_slam/data/missouri_i64/recorded_reference_map.csv
```

The MCL node subscribes to `/sonar/downward/points`, publishes `/seabed_localization/pose` and `/seabed_localization/particles`, and prints:

```text
seabed_mcl_metrics x=... y=... yaw=... pos_error_m=... yaw_error_rad=... neff=...
```

`/odom` is used only for experiment metrics, not for the localization estimate.

Stop Gazebo after the experiment:

```bash
pkill gzclient || true
pkill gzserver || true
pkill gazebo || true
```

## Route

The default route is a lawnmower survey pattern over the scaled bathymetry:

```text
(-22, -14) -> (22, -14)
(22,  -8) -> (-22, -8)
(-22, -2) -> (22,  -2)
(22,   4) -> (-22,  4)
(-22, 10) -> (22,  10)
(22,  14) -> (-22, 14)
```

This route is designed to maximize sonar coverage while keeping the boat inside the useful map area.

## What To Record

Record these topics into a rosbag:

```bash
ros2 bag record \
  /odom \
  /sonar/downward/points \
  /seabed/map_points \
  /mission/waypoints \
  /cmd_vel \
  /tf \
  /tf_static
```

For the fused-odometry variant, also record:

```bash
ros2 bag record /odometry/filtered
```

## Metrics

Report at least these metrics:

- `map_points`: number of accumulated points in `/seabed/map_points`
- `survey_time_s`: time from first movement to final waypoint
- `route_length_m`: integrated boat trajectory length from odometry
- `coverage_area_m2`: XY area covered by occupied map voxels
- `mean_point_density`: map points per covered square meter
- `odom_source`: `gt` or `fused`

For the final report, add ground-truth map comparison:

- `height_rmse_m`: RMSE between reconstructed map height and source bathymetry grid
- `height_mae_m`: MAE between reconstructed map height and source bathymetry grid
- `coverage_percent`: fraction of source grid cells observed by sonar

## Visualization

In RViz:

- fixed frame: `odom`
- add `PointCloud2` for `/sonar/downward/points`
- add `PointCloud2` for `/seabed/map_points`
- add `TF`
- add `Odometry` for `/odom`

The expected result is a growing gray point-cloud strip map following the lawnmower trajectory.
