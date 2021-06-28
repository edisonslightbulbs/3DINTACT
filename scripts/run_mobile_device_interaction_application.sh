#!/usr/bin/env bash

# run_mobile_device_interaction_application.sh:
#  a mobile device interaction application that
#  lends itself to 3DINTACT

# author: Everett
# created: 2021-06-28 16:25
# Github: https://github.com/antiqueeverett/

PROJECT_DIR=$(dirname $(dirname $(readlink -f "$0")))

# -- BIN directory
cd "$PROJECT_DIR" || return

./build/bin/mobile-device-interaction --logtostderr=1
