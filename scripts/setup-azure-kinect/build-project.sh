#!/usr/bin/env bash

# run.sh:
#     Runs the Azure-Kinect-Sensor-SDK
#     cmake project using with -GNinja.
#
# author: Everett
# created: 2020-11-06 13:41
# Github: https://github.com/antiqueeverett/

PROJECT_DIR=$(dirname $(dirname $(dirname $(readlink -f "$0"))))
echo "$PROJECT_DIR"

# -- build directory
build="$PROJECT_DIR/external/Azure-Kinect-Sensor-SDK/build"

# -- clean old build files
if [ -d "$build" ]; then rm -Rf "$build"; fi

# -- build with CMake -GNinja
mkdir "$build" && cd "$build" || return
cmake .. -GNinja
ninja
cd - || return
