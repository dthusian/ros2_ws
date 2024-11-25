#!/bin/bash

set -e

source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.sh

sudo chmod 777 /dev/ttyACM0

ros2 launch launch.py