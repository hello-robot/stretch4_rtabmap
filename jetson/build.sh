#!/bin/bash
# Build the Jetson workspace in the container: everything stretch4_rtabmap needs, or a colcon build
# with the given arguments (e.g. --packages-select stretch4_rtabmap).
# JOBS: parallel compile jobs (default 4).
# CUDA_ARCH: the GPU's compute capability (default 87, every Orin).
set -eu
source "$(dirname "$(readlink -f "$0")")/common.sh"
JOBS=${JOBS:-4}
CUDA_ARCH=${CUDA_ARCH:-87}

build() {
    docker run --rm -i "${DOCKER_ARGS[@]}" -w "$WORKSPACE" \
        -e MAKEFLAGS="-j$JOBS" -e CMAKE_BUILD_PARALLEL_LEVEL="$JOBS" "$IMAGE" \
        bash "$ENTRY" colcon build --symlink-install --executor sequential \
        --event-handlers console_direct- console_cohesion- console_start_end+ \
        --metas "$WORKSPACE/src/stretch4_rtabmap/colcon.meta" "$@" \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
            -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_BUILD_UNSTABLE=ON \
            -DGTSAM_WITH_TBB=ON -DGTSAM_USE_SYSTEM_EIGEN=OFF -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
            -DGTSAM_POSE3_EXPMAP=ON -DGTSAM_ROT3_EXPMAP=ON -DGTSAM_ALLOW_DEPRECATED_SINCE_V43=ON \
            -DGTSAM_ENABLE_BOOST_SERIALIZATION=ON -DGTSAM_COMPILE_OPTIONS_PRIVATE_COMMON=-Wall \
            -DBUILD_WITH_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES="$CUDA_ARCH" -DBUILD_WITH_MARCH_NATIVE=OFF \
            -DBUILD_WITH_VIEWER=OFF \
            -DWITH_QT=ON -DBUILD_APP=OFF -DBUILD_EXAMPLES=OFF
}

if [ $# -gt 0 ]; then
    build "$@"
else
    build --packages-up-to stretch4_rtabmap
fi
