#!/usr/bin/env bash

# run_renderexample.sh:
#   executes toolkit
#
# author: Everett
# created: 2021-04-29 13:09
# Github: https://github.com/antiqueeverett/

PROJECT_DIR=$(dirname $(dirname $(readlink -f "$0")))

# -- BIN directory
cd "$PROJECT_DIR" || return

mkdir -p ./output
rm -rf ./output/render
./build/bin/render --logtostderr=1
#cloudcompare.CloudCompare ./output/context.ply >/dev/null 2>&1 &
