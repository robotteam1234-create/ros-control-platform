#!/bin/bash
# drive_both.sh - drive BOTH Pinky robots forward with ONE command via domain bridge
#   pinky_764e (.201) on domain 12, pinky_1e3e (.202) on domain 13, operator domain 5
# Usage: ./drive_both.sh [speed_mps] [seconds]   (defaults: 0.12 3)
# Example: ./drive_both.sh            # both forward ~0.36m and stop
#          ./drive_both.sh 0.1 5      # both forward ~0.5m and stop
# Requires: bridge running (pinky_dual_bridge.yaml), both stacks up.
set -u
SPEED="${1:-0.12}"
SECS="${2:-3}"

set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source /home/ai/domain_bridge_ws/install/setup.bash
set -u
export ROS_DOMAIN_ID=5

echo "=== driving BOTH: speed=${SPEED} m/s for ${SECS}s ==="
timeout "$((SECS + 3))" ros2 topic pub /pinky_764e/cmd_vel geometry_msgs/msg/Twist "{linear: {x: ${SPEED}}}" > /tmp/drive_both_764e.log 2>&1 &
P1=$!
timeout "$((SECS + 3))" ros2 topic pub /pinky_1e3e/cmd_vel geometry_msgs/msg/Twist "{linear: {x: ${SPEED}}}" > /tmp/drive_both_1e3e.log 2>&1 &
P2=$!
sleep "$SECS"
kill "$P1" "$P2" 2>/dev/null
wait 2>/dev/null
echo "=== STOP both ==="
timeout 5 ros2 topic pub --once /pinky_764e/cmd_vel geometry_msgs/msg/Twist "{}" > /dev/null 2>&1
timeout 5 ros2 topic pub --once /pinky_1e3e/cmd_vel geometry_msgs/msg/Twist "{}" > /dev/null 2>&1
echo "done."
