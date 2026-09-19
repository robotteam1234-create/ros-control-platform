#!/usr/bin/env bash

# Hardware bringup for one Pinky Pro robot (vendor pinky_bringup launch).
# Runs under pinky-bringup@.service; all inputs come from that unit's
# EnvironmentFile (~/.config/pinky-control/<instance>.env).
# The control session (rosbridge/watchdog/Nav2) is a SEPARATE unit.

set -Eeo pipefail

fail() {
  echo "ERROR: $*" >&2
  exit 1
}

PINKY_PRO_WS="${PINKY_PRO_WS:-/home/pinky/pinky_pro}"
PINKY_CONTROL_WS="${PINKY_CONTROL_WS:-/home/pinky/dev_ws/wj}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/wait_for.sh
source "$SCRIPT_DIR/lib/wait_for.sh"

if [[ -z "${ROS_DOMAIN_ID:-}" ]]; then
  fail "ROS_DOMAIN_ID must be set by the unit EnvironmentFile"
fi
[[ "$ROS_DOMAIN_ID" == "12" || "$ROS_DOMAIN_ID" == "13" ]] \
  || fail "ROS_DOMAIN_ID must be 12 (robot_1) or 13 (robot_2), got '$ROS_DOMAIN_ID'"

[[ -f /opt/ros/jazzy/setup.bash ]] || fail "missing ROS Jazzy setup"
[[ -f "$PINKY_PRO_WS/install/setup.bash" ]] || fail "missing Pinky Pro workspace setup ($PINKY_PRO_WS)"
[[ -f "$PINKY_CONTROL_WS/install/setup.bash" ]] || fail "missing control workspace setup ($PINKY_CONTROL_WS)"
source /opt/ros/jazzy/setup.bash
source "$PINKY_PRO_WS/install/setup.bash"
source "$PINKY_CONTROL_WS/install/setup.bash"
# ROS setup scripts read variables that may not exist. Enable nounset only
# after sourcing the underlay and both overlays.
set -u

unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS2CLI_NO_DAEMON=1
export ROS_DOMAIN_ID
export PATH="/usr/bin:/bin:$PATH"

command -v ros2 >/dev/null 2>&1 || fail "ros2 is not available after sourcing ROS Jazzy"
ros2 pkg prefix pinky_bringup >/dev/null || fail "pinky_bringup is not available"

# Migration guard: legacy boot-time domain-0 services must not own the
# hardware while this unit runs.
systemctl --user stop rosy-session-bringup.service 2>/dev/null || true
systemctl --user stop rosy-session-control.service 2>/dev/null || true

echo "[pinky-bringup] launching pinky_bringup bringup_robot.launch.xml (ROS_DOMAIN_ID=$ROS_DOMAIN_ID)..."
ros2 launch pinky_bringup bringup_robot.launch.xml &
launch_pid=$!

kill_launch() {
  kill -INT -- "-$launch_pid" 2>/dev/null || kill -INT "$launch_pid" 2>/dev/null || true
}

wait_for_publisher /odom "$launch_pid" 45 || { kill_launch; fail "hardware bringup did not publish /odom"; }
wait_for_publisher /scan "$launch_pid" 45 || { kill_launch; fail "hardware bringup did not publish /scan"; }
echo "[pinky-bringup] ready: /odom and /scan are publishing."

# Foreground keepalive: if the launch dies, the unit exits non-zero and
# systemd restarts it.
wait "$launch_pid"
