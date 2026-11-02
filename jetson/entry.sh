#!/bin/bash
# Runs before every command in the container: sources ROS, then the workspace. Nodes use the
# Jetson's Zenoh router (started by jetson.launch.py), not the base image's setting.
unset ZENOH_CONFIG_OVERRIDE
source /opt/ros/jazzy/setup.bash
if [ -f "$WORKSPACE/install/setup.bash" ]; then
    source "$WORKSPACE/install/setup.bash"
fi
exec "$@"
