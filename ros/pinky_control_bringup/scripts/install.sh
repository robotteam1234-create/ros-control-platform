#!/usr/bin/env bash

# Install/refresh Pinky boot auto-start (systemd --user) for one robot.
# Run ON the robot as the pinky user:   install.sh <robot_1|robot_2>
# No sudo required: payload lives in $HOME (units in ~/.config/systemd/user).
# Works from a repo checkout or the colcon share dir (layout mirrors).

set -Eeo pipefail

fail() {
  echo "ERROR: $*" >&2
  exit 1
}

INSTANCE="${1:-}"
[[ "$INSTANCE" == "robot_1" || "$INSTANCE" == "robot_2" ]] || fail "usage: $0 <robot_1|robot_2>"

SRC_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
[[ -f "$SRC_ROOT/systemd/pinky-bringup@.service" ]] || fail "unit templates not found under $SRC_ROOT/systemd"
ENV_SRC="$SRC_ROOT/config/${INSTANCE}.env"
[[ -f "$ENV_SRC" ]] || fail "missing env template $ENV_SRC"

# Refuse to fight running legacy foreground wrappers.
old_wrappers="$(pgrep -u "$(id -un)" -f '[s]tart-pinky-robot2-(all|session)' || true)"
old_wrappers+=" $(pgrep -u "$(id -un)" -f '[s]tart-robot2\.sh' || true)"
old_wrappers="$(echo "$old_wrappers" | xargs)"
[[ -z "$old_wrappers" ]] || fail "legacy start scripts are running (PID(s): $old_wrappers). Stop them first."

# The control workspace must already be built on THIS robot. robot_1 has no
# control workspace yet — build pinky_control_interfaces, pinky_control_watchdog
# and pinky_control_navigation in /home/pinky/dev_ws/wj first (docs/user-guide.md).
PINKY_PRO_WS="$(grep -E '^PINKY_PRO_WS=' "$ENV_SRC" | cut -d= -f2-)"
PINKY_CONTROL_WS="$(grep -E '^PINKY_CONTROL_WS=' "$ENV_SRC" | cut -d= -f2-)"
[[ -f /opt/ros/jazzy/setup.bash ]] || fail "missing ROS Jazzy"
[[ -f "$PINKY_PRO_WS/install/setup.bash" ]] || fail "missing $PINKY_PRO_WS/install/setup.bash (build pinky_pro first)"
[[ -f "$PINKY_CONTROL_WS/install/setup.bash" ]] || fail "missing $PINKY_CONTROL_WS/install/setup.bash (build the control workspace first; robot_1 see docs/user-guide.md)"
source /opt/ros/jazzy/setup.bash
source "$PINKY_PRO_WS/install/setup.bash"
source "$PINKY_CONTROL_WS/install/setup.bash"
set -u

ros2 pkg prefix pinky_bringup >/dev/null || fail "pinky_bringup not available"
ros2 pkg prefix pinky_control_watchdog >/dev/null || fail "pinky_control_watchdog not available"
ros2 pkg prefix pinky_control_navigation >/dev/null || fail "pinky_control_navigation not available"
ros2 pkg prefix rosbridge_server >/dev/null || fail "rosbridge_server not available"

BIN_DIR="$HOME/pinky/startup/bin"
ENV_DIR="$HOME/.config/pinky-control"
UNIT_DIR="$HOME/.config/systemd/user"
mkdir -p "$BIN_DIR/lib" "$ENV_DIR" "$UNIT_DIR"

install -m 0755 "$SRC_ROOT/scripts/robot_bringup.sh" "$BIN_DIR/"
install -m 0755 "$SRC_ROOT/scripts/robot_session.sh" "$BIN_DIR/"
install -m 0644 "$SRC_ROOT/scripts/lib/wait_for.sh" "$BIN_DIR/lib/"
install -m 0644 "$ENV_SRC" "$ENV_DIR/${INSTANCE}.env"
install -m 0644 "$SRC_ROOT/systemd/pinky-bringup@.service" "$UNIT_DIR/"
install -m 0644 "$SRC_ROOT/systemd/pinky-session@.service" "$UNIT_DIR/"

systemctl --user daemon-reload

# Legacy domain-0 boot services conflict with these units on hardware access.
# Disable (not delete) them.
systemctl --user disable --now rosy-session-bringup.service 2>/dev/null || true
systemctl --user disable --now rosy-session-control.service 2>/dev/null || true

# User units only start at boot with linger enabled. The legacy rosy units
# prove boot-time user services worked here, but verify instead of assuming.
if loginctl show-user "$(id -un)" --property=Linger 2>/dev/null | grep -q '^Linger=yes'; then
  echo "Linger is enabled."
elif loginctl enable-linger "$(id -un)" 2>/dev/null; then
  echo "Linger enabled."
else
  fail "could not enable linger automatically; run 'sudo loginctl enable-linger $(id -un)' on the robot, then re-run this script"
fi

systemctl --user enable --now "pinky-bringup@${INSTANCE}.service" "pinky-session@${INSTANCE}.service"

echo
echo "Installed and started pinky-bringup@${INSTANCE}.service + pinky-session@${INSTANCE}.service."
echo "Logs:   journalctl --user -u 'pinky-*' -f"
echo "Status: systemctl --user status 'pinky-*'"
echo "After every reboot: set the AMCL initial pose in the web UI before driving."
