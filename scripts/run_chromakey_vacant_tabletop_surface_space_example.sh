#!/usr/bin/env bash

# run_chromakey_vacant_tabletop_surface_space_example.sh
#   example application of how chromakey tabletop surface
#
# author: Everett
# created: 2021-06-24 14:09
# Github: https://github.com/antiqueeverett/

PROJECT_DIR=$(dirname $(dirname $(readlink -f "$0")))

# -- BIN directory
cd "$PROJECT_DIR" || return

./build/bin/chromakey-vacant-tabletop-surface-space --logtostderr=1
