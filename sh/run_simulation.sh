#!/bin/bash

set -e
BASE_DIR="$(dirname "$0")/.."
source "$BASE_DIR/sh/cfg.sh"

# GAZEBO_MODEL_PATH_EXT="$(realpath "$BASE_DIR/models")"
# export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$GAZEBO_MODEL_PATH_EXT
# echo "updated gazebo model path: $GAZEBO_MODEL_PATH"

S=$1
S=1

if [ -z "$S" ] || [ ! -f "$BASE_DIR/open_worlds/$S.world" ]; then
	echo "Please, provide a world name. Available worlds:"
	echo "$(ls "$BASE_DIR/open_worlds" | while read line; do echo $line | cut -d'.' -f1,3;done)"
	exit 1
fi

WORLD="$(realpath "$BASE_DIR/open_worlds/$S.world")"

echo "using $WORLD"

# script accepts one argument, name of world(field) in open_world folder

# all paths should be changed wrt virtual machine

cd ~/catkin_ws # can be replaced with something like 'cd ~/catkin_ws'
catkin build
. devel/setup.bash
# python3 ~/artag-detection/main.py
roslaunch pioneer3dx-robot-core simulation_w.launch "world_filename:=$WORLD"
