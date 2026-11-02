# Sourced by run.sh and build.sh: the container's Docker arguments. ~/stretch_gpu/env is written
# by `ros2 run stretch4_rtabmap jetson deploy`.
source "$HOME/stretch_gpu/env"     # IMAGE, CONTAINER_HOME, WORKSPACE, NUC_ROUTER
HOST_HOME="$HOME/stretch_gpu/home"  # mounted as CONTAINER_HOME, the robot computer's home path
# WORKSPACE is the robot computer's workspace path. The Jetson keeps its copy under
# ~/stretch_gpu/home and mounts it at that same path in the container.
ENTRY="$WORKSPACE/src/stretch4_rtabmap/jetson/entry.sh"
DOCKER_ARGS=(
    --runtime nvidia --network host
    --user "$(id -u):$(id -g)"
    --group-add "$(getent group video | cut -d: -f3)"
    --group-add "$(getent group render | cut -d: -f3)"
    -e HOME="$CONTAINER_HOME" -e WORKSPACE="$WORKSPACE" -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
    -v "$HOST_HOME:$CONTAINER_HOME"
)
