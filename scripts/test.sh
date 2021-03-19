#!/usr/bin/env bash

# test:
#     runs vim diff on two *ply files
#
# author: Everett
# created: 2020-11-18 17:48
# Github: https://github.com/antiqueeverett/

PROJECT_DIR=$(dirname $(dirname $(readlink -f "$0")))

cd "$PROJECT_DIR/build/bin" || return

./test 'test.ply'
./main

vim -d "test.ply" "pointcloud.ply"

cd - || return
