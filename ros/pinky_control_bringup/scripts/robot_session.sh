#!/usr/bin/env bash

# Control session for one Pinky Pro robot: rosbridge + watchdog + optional
# Nav2. Runs under pinky-session@.service; hardware bringup is a separate
# unit (pinky-bringup@). All inputs come from the unit EnvironmentFile
# (~/.config/pinky-control/<instance>.env).

set -Eeo pipefail

fail() {
  echo "ERROR: $*" >&2
  exit 1
}

PINKY_WS="${PINKY_CONTROL_WS:-/home/pinky/dev_ws/wj}"
START_NAV2="${START_NAV2:-1}"
CONTROL_STATUS_TOPIC="/control/status"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/wait_for.sh
source "$SCRIPT_DIR/lib/wait_for.sh"

rosbridge_pid=""
watchdog_pid=""
nav2_pid=""
cleanup_done=0

cleanup() {
  [[ "$cleanup_done" -eq 0 ]] || return
  cleanup_done=1
  set +e
  for pid in "$nav2_pid" "$watchdog_pid" "$rosbridge_pid"; do
    if [[ -n "$pid" ]]; then
      # Each process is started in its own session so children (for example
      # the Python rosbridge server) stop together with it.
      kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null
    fi
  done
  wait "$nav2_pid" "$watchdog_pid" "$rosbridge_pid" 2>/dev/null || true
}

trap cleanup EXIT
trap 'exit 130' INT TERM

if [[ -z "${ROBOT_ID:-}" ]]; then
  fail "ROBOT_ID must be set by the unit EnvironmentFile"
fi
if [[ -z "${ROS_DOMAIN_ID:-}" ]]; then
  fail "ROS_DOMAIN_ID must be set by the unit EnvironmentFile"
fi
case "${ROS_DOMAIN_ID}:${ROSBRIDGE_PORT:-}" in
  12:9090 | 13:9091) ;;
  *) fail "domain/port must be the fixed pair 12:9090 (robot_1) or 13:9091 (robot_2), got '${ROS_DOMAIN_ID}:${ROSBRIDGE_PORT:-<unset>}'" ;;
esac
case "$START_NAV2" in
  0 | 1) ;;
  *) fail "START_NAV2 must be 0 or 1" ;;
esac

[[ -f /opt/ros/jazzy/setup.bash ]] || fail "missing ROS Jazzy setup"
[[ -f "$PINKY_WS/install/setup.bash" ]] || fail "missing control workspace setup ($PINKY_WS)"
source /opt/ros/jazzy/setup.bash
source "$PINKY_WS/install/setup.bash"
# Enable nounset only after sourcing the underlay and overlay.
set -u

unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_DOMAIN_ID
export ROS2CLI_NO_DAEMON=1
export PATH="/usr/bin:/bin:$PATH"

command -v ros2 >/dev/null 2>&1 || fail "ros2 is not available after sourcing ROS Jazzy"
ros2 pkg prefix rosbridge_server >/dev/null || fail "rosbridge_server package is not available"
ros2 pkg prefix pinky_control_watchdog >/dev/null || fail "pinky_control_watchdog package is not available; build the control workspace first"
if [[ "$START_NAV2" == "1" ]]; then
  ros2 pkg prefix pinky_control_navigation >/dev/null || fail "pinky_control_navigation package is not available; build the control workspace first"
fi

# Migration guard: the legacy boot-time control session must not duplicate ours.
systemctl --user stop rosy-session-control.service 2>/dev/null || true

existing_watchdog="$(pgrep -u "$(id -un)" -f '[/]pinky_control_watchdog/manual_velocity_watchdog' || true)"
[[ -z "$existing_watchdog" ]] || fail "pinky control watchdog is already running (PID(s): $existing_watchdog)"

existing_bridge="$(pgrep -u "$(id -un)" -f "[r]osbridge_websocket.*--port.*${ROSBRIDGE_PORT}" || true)"
[[ -z "$existing_bridge" ]] || fail "rosbridge is already running on port $ROSBRIDGE_PORT (PID(s): $existing_bridge)"

echo "[pinky-session] starting rosbridge on port $ROSBRIDGE_PORT (ROS_DOMAIN_ID=$ROS_DOMAIN_ID)..."
setsid ros2 run rosbridge_server rosbridge_websocket --port "$ROSBRIDGE_PORT" &
rosbridge_pid=$!

echo "[pinky-session] starting pinky_control_watchdog (robot_id=$ROBOT_ID)..."
setsid ros2 run pinky_control_watchdog manual_velocity_watchdog --ros-args \
  -p robot_id:="$ROBOT_ID" \
  -p manual_topic:=/control/manual_velocity \
  -p nav_topic:=/control/nav_velocity \
  -p navigate_action_name:=/navigate_to_pose \
  -p cmd_vel_topic:=/cmd_vel \
  -p status_topic:="$CONTROL_STATUS_TOPIC" \
  -p heartbeat_topic:=/control/heartbeat \
  -p max_linear_mps:=0.15 \
  -p max_angular_rps:=0.50 &
watchdog_pid=$!

if ! wait_for_publisher "$CONTROL_STATUS_TOPIC" "$watchdog_pid" 15; then
  fail "control watchdog did not publish $CONTROL_STATUS_TOPIC within 15 seconds"
fi

if [[ "$START_NAV2" == "1" ]]; then
  start_lidar_motor || fail "SLLidar /start_motor did not come up"
  echo "[pinky-session] starting pinky_control_navigation..."
  setsid ros2 launch pinky_control_navigation robot_nav2.launch.py use_sim_time:=false &
  nav2_pid=$!
  if ! wait_for_action_server "/navigate_to_pose" "$nav2_pid" 60; then
    fail "Nav2 did not expose /navigate_to_pose within 60 seconds"
  fi
else
  echo "[pinky-session] Nav2 is disabled (START_NAV2=0)."
fi

echo "[pinky-session] ready."
echo "  robot:      $ROBOT_ID"
echo "  domain:     $ROS_DOMAIN_ID"
echo "  rosbridge:  ws://0.0.0.0:$ROSBRIDGE_PORT"
echo "  control:    /control/manual_velocity -> /cmd_vel (watchdog, boot stop-latch ON)"
echo "  navigation: $START_NAV2 (/navigate_to_pose -> /control/nav_velocity)"

session_pids=("$rosbridge_pid" "$watchdog_pid")
[[ -n "$nav2_pid" ]] && session_pids+=("$nav2_pid")
wait -n "${session_pids[@]}"
exit $?
