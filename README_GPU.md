# stretch4_rtabmap on a Jetson (GPU)

Runs GLIM (with its CUDA odometry) and RTAB-Map on the Jetson instead of the robot computer.
Everything else stays on the robot computer.

All commands run on the robot computer, with `$WORKSPACE` set as in [README.md](README.md). The
Jetson defaults to `jetson1@192.168.1.101`; for a different one, add
`export STRETCH_JETSON=<user>@<address>` to `~/.bashrc`.

## Setup (once)

Install `stretch4_rtabmap` as [README.md](README.md) says, then:

```bash
cd $WORKSPACE && source install/setup.bash
ros2 run stretch4_rtabmap jetson deploy
ros2 run stretch4_rtabmap jetson push-map ~/stretch_maps/<my_map>.db
ros2 run stretch4_rtabmap jetson check
```

`jetson deploy` builds on the Jetson; the first time takes 1-2 h. `jetson check` should print
`[ok]` on every line.

## Run

```bash
ros2 launch stretch4_rtabmap navigation_jetson.launch.py mode:=localization database_path:=$HOME/stretch_maps/<my_map>.db use_rviz:=true mapped_navigator:=true
```

Then set the pose with **2D Pose Estimate** in RViz. Ctrl+C stops both computers.

## After changing code or parameters

```bash
ros2 run stretch4_rtabmap jetson deploy
```

## Get the map back from the Jetson

Mapping updates the Jetson's copy of the database.

```bash
ros2 run stretch4_rtabmap jetson stop
ros2 run stretch4_rtabmap jetson pull-map <my_map>.db --dest ~/stretch_maps/<my_map>.db
```
