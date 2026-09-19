# Shared health-gate helpers for Pinky startup scripts.
# Sourced by robot_bringup.sh and robot_session.sh — never executed directly.
# Requires: ros2 on PATH, an owner PID to detect early exit.

wait_for_publisher() {
  local topic="$1"
  local owner_pid="$2"
  local timeout_seconds="${3:-30}"
  local attempt
  local topic_info

  for ((attempt = 1; attempt <= timeout_seconds; attempt++)); do
    kill -0 "$owner_pid" 2>/dev/null || return 1
    topic_info="$(timeout 5s ros2 topic info "$topic" 2>/dev/null || true)"
    if grep -Eq 'Publisher count: [1-9]' <<<"$topic_info"; then
      return 0
    fi
    sleep 1
  done
  return 1
}

wait_for_action_server() {
  local action="$1"
  local owner_pid="$2"
  local timeout_seconds="${3:-30}"
  local attempt
  local action_info

  for ((attempt = 1; attempt <= timeout_seconds; attempt++)); do
    kill -0 "$owner_pid" 2>/dev/null || return 1
    action_info="$(timeout 5s ros2 action list -t 2>/dev/null || true)"
    if grep -Eq "^${action}[[:space:]]" <<<"$action_info"; then
      return 0
    fi
    sleep 1
  done
  return 1
}

# Bring the SLLidar scanning motor up. Nav2 needs /scan for AMCL; a registered
# lidar publisher alone is NOT enough (functional spec, docs/02-functional-spec.md:142).
start_lidar_motor() {
  local attempt
  local services

  echo "[pinky-session] starting SLLidar motor..."
  for ((attempt = 1; attempt <= 15; attempt++)); do
    services="$(timeout 5s ros2 service list 2>/dev/null || true)"
    if grep -Fxq "/start_motor" <<<"$services"; then
      if timeout 10s ros2 service call /start_motor std_srvs/srv/Empty "{}" >/dev/null 2>&1; then
        echo "[pinky-session] SLLidar motor is running."
        return 0
      fi
    fi
    sleep 1
  done
  echo "ERROR: SLLidar /start_motor service did not become ready; /scan and map TF cannot be produced" >&2
  return 1
}
