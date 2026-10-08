#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/.."
source /opt/ros/lyrical/setup.bash
source install/setup.bash
bag_dir="${1:-results/week1/manual_bag_$(date +%Y%m%d_%H%M%S)}"
exec ros2 bag record --use-sim-time --include-hidden-topics -o "$bag_dir" --topics \
 /ares/odom /ares/imu /ares/scan /odometry/filtered /amcl_pose /tf /tf_static \
 /cmd_vel /ares/cmd_vel /plan /clock /diagnostics \
 /navigate_to_pose/_action/status /navigate_to_pose/_action/feedback
