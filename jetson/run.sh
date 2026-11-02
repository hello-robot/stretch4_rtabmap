#!/bin/bash
# Start GLIM and RTAB-Map in the container; arguments go to jetson.launch.py:
#
#   ~/stretch_gpu/run.sh mode:=localization database_path:=~/stretch_maps/x.db
#
# Ctrl-C, a closed ssh session or `jetson stop` send SIGINT to the container and give RTAB-Map up
# to 150 s to save the database.
set -u
source "$(dirname "$(readlink -f "$0")")/common.sh"
NAME=stretch_gpu

stop() {
    docker stop --signal SIGINT --time 150 "$NAME" >/dev/null 2>&1
}

if [ -n "$(docker ps -aq --filter "name=^/${NAME}$")" ]; then
    echo "run.sh: stopping the $NAME container left from an earlier run"
    stop
    docker rm -f "$NAME" >/dev/null 2>&1
fi

trap 'stop; exit 0' HUP INT TERM
docker run --rm --init --sig-proxy=false --name "$NAME" "${DOCKER_ARGS[@]}" "$IMAGE" \
    bash "$ENTRY" ros2 launch stretch4_rtabmap jetson.launch.py \
    nuc_router:="$NUC_ROUTER" "$@" &
wait $!
