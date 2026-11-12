# Stretch 4 Rtabmap

Lifelong mapping and navigation for Stretch 4: GLIM for lidar-inertial odometry, RTAB-Map for the map and localization,
and Nav2 for driving.

To run GLIM and RTAB-Map on a Jetson (GPU) instead of the robot computer, see
[README_GPU.md](README_GPU.md).

## Install

Once per robot, on Ubuntu 24.04 with ROS 2 Jazzy. The commands use `$WORKSPACE` for your colcon
workspace; set it first, e.g. `export WORKSPACE=~/ament_ws`.

**1. Get the sources.** GTSAM, GLIM and the patched RTAB-Map are built from source:

```bash
sudo apt install python3-vcstool
cd $WORKSPACE/src
git clone https://github.com/hello-robot/stretch4_rtabmap.git
cd $WORKSPACE
vcs import src < src/stretch4_rtabmap/stretch4_rtabmap.repos
```

**2. Install the dependencies:**

```bash
sudo apt install libomp-dev libboost-all-dev libmetis-dev libfmt-dev libspdlog-dev \
    libglm-dev libglfw3-dev libpng-dev libjpeg-dev ros-jazzy-rviz-2d-overlay-plugins
pip3 install ssmach                                  # mapped_navigator's state machine
rosdep install --from-paths src --ignore-src -r -y
```

**3. Build.** `colcon.meta` is used to declare dependencies. Link it into the workspace
root once and build from there (colcon reads `./colcon.meta`):

```bash
ln -sf $WORKSPACE/src/stretch4_rtabmap/colcon.meta $WORKSPACE/colcon.meta
cd $WORKSPACE
colcon build --symlink-install --cmake-args \
    -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DBUILD_TESTING=OFF
source install/setup.bash
```


## 1. Make a map

Stow the arm first. slam starts a new map and deletes any database already at that path;
replace `<my_map>` with your map's name here and in the commands below:

```bash
ros2 launch stretch4_rtabmap navigation_mppi_glim_rtabmap.launch.py \
    mode:=slam database_path:=$HOME/stretch_maps/<my_map>.db use_rviz:=true
```

Drive the robot around the space. Stop with Ctrl+C and wait: RTAB-Map saves the database on
exit, which can take a minute for a large map.

## 2. Navigate in the map

```bash
ros2 launch stretch4_rtabmap navigation_mppi_glim_rtabmap.launch.py \
    mode:=localization database_path:=$HOME/stretch_maps/<my_map>.db use_rviz:=true \
    mapped_navigator:=true
```

1. In RViz, set the robot's pose with **2D Pose Estimate**.
2. Send goals with **2D Goal Pose**.

The robot can drive right away. As it moves, RTAB-Map confirms the pose and then starts updating
the map on its own (`enable_mapping:=false` keeps the map read-only). It maps only in the
light: when the head camera is dark it stays in localization, which keeps working on the lidars.
The thresholds are in `config/rtabmap_supervisor.yaml`.

## 3. Get the map as an image

```bash
ros2 run stretch4_rtabmap export_map ~/stretch_maps/<my_map>.db
```

Writes `<my_map>.pgm` and `<my_map>.yaml` (a normal map_server map) next to the database;
`-o <path without extension>` writes them elsewhere. This is the map as RTAB-Map saved it at the
end of the last run. To save the live map while the launch is running instead:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/stretch_maps/<my_map>_live
```

## 4. Keepout and speed zones (optional)

Export the map (step 3), draw Nav2 keepout and speed masks over it with any editor (for example
[stretch4_nav_webapp](https://github.com/hello-robot/stretch4_nav_webapp)), and pass them to the
navigation launch:

```bash
ros2 launch stretch4_rtabmap navigation_mppi_glim_rtabmap.launch.py \
    mode:=localization database_path:=$HOME/stretch_maps/<my_map>.db use_rviz:=true \
    mapped_navigator:=true \
    keepout_mask:=<keepout.yaml> speed_mask:=<speed.yaml>
```

## 5. Check the localization status

```bash
ros2 topic echo /localization_status
```

The status is also shown in the top-left corner of RViz.

| Status | Meaning |
|---|---|
| `NO_DATA` | RTAB-Map is not running yet. Goals are cancelled. |
| `NEED_POSE` | Set a 2D Pose Estimate. Goals are cancelled. |
| `UNVERIFIED` | Pose given, not confirmed yet. Keep driving. |
| `LOCALIZED` | Pose confirmed. |
| `JOINING` | Starting to update the map. |
| `MAPPING` | Driving now updates the map. |
| `LOST` | Pose is not trusted. Drive through mapped space or set a new 2D Pose Estimate. |

## Optional: goals that update the map

If a goal has no path because the map is out of date (moved furniture, unmapped space),
`mapped_navigator` explores toward the goal and tries again.

The command in step 2 starts it with `mapped_navigator:=true`. To add it to a navigation launch
that is already up:

```bash
ros2 launch stretch4_rtabmap mapped_navigator.launch.py
```

Send goals to it instead of to Nav2:

- **RViz:** with `mapped_navigator:=true`, *Nav2 Goal* and the *Navigation 2* panel (Cancel
  included) use `navigate_mapped`. Without it, they go to Nav2.
- **Command line:**

  ```bash
  ros2 action send_goal /navigate_mapped nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: 3.0, y: 2.0}, orientation: {w: 1.0}}}}"
  ```

- **Code:** any `NavigateToPose` client works; use the action name `navigate_mapped`.

## Launch files

`navigation_mppi_glim_rtabmap.launch.py` is the one to run. It is put together from:

| File | Starts |
|---|---|
| `robot.launch.py` | driver, head stereo cameras, lidars, footprint, IMU, GLIM odometry |
| `rtabmap_glim.launch.py` | RTAB-Map and its image inputs (robot self-mask, stereo sync) |
| `nav2.launch.py` | Nav2, and the zone mask servers when masks are given |

Others: `mapped_navigator.launch.py` (above), `robot_self_mask_preview.launch.py` (tune how the
robot's body is painted out of the camera images), and `navigation_jetson.launch.py` with
`jetson.launch.py` ([README_GPU.md](README_GPU.md)).

Arguments of the three part files can be given to the main launch too. The common ones:

| Argument | Default | Meaning |
|---|---|---|
| `mode` | `slam` | `slam` makes a new map, `localization` uses an existing one |
| `database_path` | `~/.ros/rtabmap.db` | Map database |
| `use_rviz` | `false` | Open RViz |
| `initial_pose` | empty | Starting pose `"x y z roll pitch yaw"`; empty uses the last saved pose |
| `enable_mapping` | `true` | `false` keeps the map read-only in localization mode |
| `keepout_mask`, `speed_mask` | empty | Zone masks |
| `mask_robot` | `true` | Hide the robot's own body from the camera images |
| `rtabmap_viz` | `false` | RTAB-Map's own UI |
| `start_supervisor` | `true` | Localization status and automatic mapping switch |

`ros2 launch stretch4_rtabmap <file> --show-args` lists every argument of a file.

## Troubleshooting

```bash
ros2 run stretch4_rtabmap rtabmap_loop_monitor    # what RTAB-Map is doing, step by step
ros2 topic echo /mapped_navigator/state           # what the navigator is doing
ros2 run stretch4_rtabmap memory_monitor          # RAM use per process
```

You can also inspect a saved database (nodes, loop closures, 3D map, etc.) with RTAB-Map's database viewer. By default RTAB-Map writes to `~/.ros/rtabmap.db`. If you set `database_path`, pass that path instead:

```bash
rtabmap-databaseViewer ~/.ros/rtabmap.db
```

Settings live in `config/`: `rtabmap_params_*.yaml` (RTAB-Map), `rtabmap_supervisor.yaml`
(localization status), `mapped_navigator.yaml` (navigator) and `glim_overrides.yaml` (Nav2
changes for GLIM).
