#!/bin/bash
# Build one Uno sketch for the PC (the simulator runs the real .ino, with Arduino stubs).
# usage: bash build.sh <sketch.ino> <output-binary> [stubs-variant-folder]
#   bash build.sh ../firmware/mavp_uno/mavp_uno.ino sim_v76c
set -e
if [ -z "$1" ] || [ -z "$2" ]; then echo "usage: bash build.sh <sketch.ino> <output-binary> [stubs-variant-folder]"; exit 1; fi
EXTRA=""
if [ -n "$3" ]; then EXTRA="-I$3"; fi
g++ -std=c++17 -O1 -I. $EXTRA -Istubs -DFW_PATH="\"$1\"" sim_fw.cpp -o "$2"
ls -la "$2" | awk '{print $9, $5, "bytes"}'
