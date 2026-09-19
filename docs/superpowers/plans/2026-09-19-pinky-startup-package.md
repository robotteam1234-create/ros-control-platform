# Pinky Startup Package Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Both Pinky robots auto-start their full control stack (hardware bringup → rosbridge → watchdog → Nav2) at boot via systemd **user** units, parameterized per robot from env files.

**Architecture:** New ament_python package `ros/pinky_control_bringup` holding two generalized runtime scripts (bringup, session), a shared health-gate helper lib, two template units (`pinky-bringup@`, `pinky-session@`), per-robot env files, and a no-sudo `install.sh` that runs on the robot over SSH. Session unit depends on bringup unit so a session crash never re-opens serial devices.

**Tech Stack:** Bash (ROS 2 Jazzy env), systemd user units (lingered), ament_python packaging (colcon), no new runtime deps.

**Spec:** `docs/superpowers/specs/2026-09-19-pinky-startup-package-design.md`

## Global Constraints

- Watchdog is the SOLE `/cmd_vel` publisher; Nav2 keeps its `cmd_vel → /control/nav_velocity` remap (untouched, but do not regress anything that would break it).
- Watchdog boots STOP-latched and never auto-resumes — auto-start at boot is safe by design; AMCL initial pose stays a MANUAL web-UI step after every reboot.
- Fixed deployment mapping (from `docs/04-contract-clarifications.md`): `robot_1 = ROS_DOMAIN_ID 12 / rosbridge :9090`, `robot_2 = 13 / :9091`. Scripts must enforce the domain↔port pairing.
- Keep proven script idioms from `deployment/scripts/start-pinky-robot2-*.sh`: `set -Eeo pipefail`; `set -u` only AFTER sourcing ROS underlays/overlays; explicit `unset ROS_LOCALHOST_ONLY`; `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`; `PATH=/usr/bin:/bin` first; bracket-pattern `pgrep -f '[x]...'` (never self-matching); per-process `setsid` groups + INT group-kill teardown; health gates via `ros2 topic info` / `ros2 action list`.
- No sudo anywhere (PC has none; robot installs run as `pinky`).
- Commits: `git -c user.name="opencode" -c user.email="opencode@localhost" commit` from `/home/ai/robot/control-platform`.
- Verification tools available on PC: `bash -n`, `systemd-analyze --user verify`, `colcon` (ROS Jazzy at `/opt/ros/jazzy`). NO shellcheck on this PC — do not gate on it.
- Docs are Korean, matching existing `runbook.md` / `docs/user-guide.md` style.

## File Structure

```
ros/pinky_control_bringup/                     # NEW package
├── package.xml                                # Task 1
├── setup.py                                   # Task 1 (data_files for scripts/systemd/config)
├── setup.cfg                                  # Task 1
├── resource/pinky_control_bringup             # Task 1
├── scripts/
│   ├── lib/wait_for.sh                        # Task 2 (shared gates)
│   ├── robot_bringup.sh                       # Task 3 (hardware bringup unit payload)
│   ├── robot_session.sh                       # Task 4 (rosbridge+watchdog+Nav2 unit payload)
│   └── install.sh                             # Task 6 (on-robot installer)
├── systemd/
│   ├── pinky-bringup@.service                 # Task 5
│   └── pinky-session@.service                 # Task 5
├── config/
│   ├── robot_1.env                            # Task 5
│   └── robot_2.env                            # Task 5
└── AGENTS.md                                  # Task 7

ros/AGENTS.md                                  # Task 7 (add subdirectory row)
runbook.md                                     # Task 8 (§6.2 systemd flow)
docs/user-guide.md                             # Task 8 (replace manual wrapper flow)
docs/superpowers/specs/2026-09-19-pinky-startup-package-design.md  # Task 6 (sync: fixed %h path, no sed rendering)
```

---

### Task 1: Package scaffolding

**Files:**
- Create: `ros/pinky_control_bringup/package.xml`
- Create: `ros/pinky_control_bringup/setup.py`
- Create: `ros/pinky_control_bringup/setup.cfg`
- Create: `ros/pinky_control_bringup/resource/pinky_control_bringup`

**Interfaces:**
- Produces: ament_python package named `pinky_control_bringup` version 0.1.0; `data_files` entries that Tasks 2–6 fill: `share/pinky_control_bringup/scripts{,/lib}`, `share/pinky_control_bringup/systemd`, `share/pinky_control_bringup/config`. `install.sh` (Task 6) resolves `SRC_ROOT` from the scripts dir and works BOTH from a repo checkout and the colcon share dir — the layout must mirror.

- [ ] **Step 1: Create `package.xml`** (mirrors `ros/pinky_control_watchdog/package.xml` style; no new runtime deps — payload is shell)

```xml
<?xml version="1.0"?>
<package format="3">
  <name>pinky_control_bringup</name>
  <version>0.1.0</version>
  <description>Boot auto-start scripts and systemd user units for Pinky robots (bringup, rosbridge, watchdog, Nav2).</description>
  <maintainer email="operator@example.com">Pinky Control Center</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_python</buildtool_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] **Step 2: Create `setup.py`**

```python
from setuptools import setup

package_name = "pinky_control_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (
            f"share/{package_name}/scripts",
            [
                "scripts/robot_bringup.sh",
                "scripts/robot_session.sh",
                "scripts/install.sh",
            ],
        ),
        (f"share/{package_name}/scripts/lib", ["scripts/lib/wait_for.sh"]),
        (
            f"share/{package_name}/systemd",
            [
                "systemd/pinky-bringup@.service",
                "systemd/pinky-session@.service",
            ],
        ),
        (
            f"share/{package_name}/config",
            ["config/robot_1.env", "config/robot_2.env"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
)
```

- [ ] **Step 3: Create `setup.cfg`**

```ini
[develop]
script_dir=$base/lib/pinky_control_bringup
[install]
install_scripts=$base/lib/pinky_control_bringup
```

- [ ] **Step 4: Create empty `resource/pinky_control_bringup`**

```bash
mkdir -p ros/pinky_control_bringup/resource
touch ros/pinky_control_bringup/resource/pinky_control_bringup
```

- [ ] **Step 5: Syntax-check packaging**

Run: `python3 -c "import ast; ast.parse(open('ros/pinky_control_bringup/setup.py').read()); print('ok')"`
Expected: `ok`

- [ ] **Step 6: Commit**

```bash
git add ros/pinky_control_bringup/package.xml ros/pinky_control_bringup/setup.py ros/pinky_control_bringup/setup.cfg ros/pinky_control_bringup/resource/pinky_control_bringup
git -c user.name="opencode" -c user.email="opencode@localhost" commit -m "feat: pinky_control_bringup package scaffolding"
```

---

### Task 2: Shared health-gate helper `scripts/lib/wait_for.sh`

**Files:**
- Create: `ros/pinky_control_bringup/scripts/lib/wait_for.sh`

**Interfaces:**
- Produces (sourced functions, used by Tasks 3, 4, and copied verbatim from the proven implementations in `deployment/scripts/start-pinky-robot2-all.sh:63-79` and `start-pinky-robot2-session.sh:40-96`):
  - `wait_for_publisher <topic> <owner_pid> [timeout=30]` → 0 when `ros2 topic info` shows Publisher count ≥ 1; 1 if owner died or timeout
  - `wait_for_action_server <action> <owner_pid> [timeout=30]` → 0 when `ros2 action list -t` lists the action
  - `start_lidar_motor` → 0 after a successful `/start_motor` call; **returns 1** (does not exit) so callers stay generic

- [ ] **Step 1: Write the file**

```bash
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
```

- [ ] **Step 2: Syntax check**

Run: `bash -n ros/pinky_control_bringup/scripts/lib/wait_for.sh`
Expected: exit 0, no output

- [ ] **Step 3: Commit**

```bash
git add ros/pinky_control_bringup/scripts/lib/wait_for.sh
git -c user.name="opencode" -c user.email="opencode@localhost" commit -m "feat: shared wait_for health-gate helpers"
```

---

### Task 3: `scripts/robot_bringup.sh` (hardware bringup unit payload)

**Files:**
- Create: `ros/pinky_control_bringup/scripts/robot_bringup.sh`

**Interfaces:**
- Consumes: `wait_for_publisher` (Task 2); env `ROS_DOMAIN_ID`, `PINKY_PRO_WS`, `PINKY_CONTROL_WS` from the unit `EnvironmentFile`
- Produces: exit 0 only while the vendor launch keeps running; gate failure kills the launch group and exits non-zero (systemd restarts). Does NOT touch Nav2/rosbridge/watchdog (session unit's job).

- [ ] **Step 1: Write the file**

```bash
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
```

- [ ] **Step 2: Syntax check + literal audit**

Run: `bash -n ros/pinky_control_bringup/scripts/robot_bringup.sh && grep -n 'robot_2\|13' ros/pinky_control_bringup/scripts/robot_bringup.sh`
Expected: syntax OK; only the `12 (robot_1) or 13 (robot_2)` guard line mentions them — no hardcoded domain 13 anywhere else

- [ ] **Step 3: Commit**

```bash
git add ros/pinky_control_bringup/scripts/robot_bringup.sh
git -c user.name="opencode" -c user.email="opencode@localhost" commit -m "feat: generic hardware bringup unit payload"
```

---

### Task 4: `scripts/robot_session.sh` (rosbridge + watchdog + Nav2 unit payload)

**Files:**
- Create: `ros/pinky_control_bringup/scripts/robot_session.sh`

**Interfaces:**
- Consumes: `wait_for_publisher`, `wait_for_action_server`, `start_lidar_motor` (Task 2); env `ROBOT_ID`, `ROS_DOMAIN_ID`, `ROSBRIDGE_PORT`, `START_NAV2`, `PINKY_CONTROL_WS` from the unit `EnvironmentFile`
- Produces: ready session with `/control/status` publishing and (if `START_NAV2=1`) `/navigate_to_pose` action server; watchdog parameter `robot_id` now comes from env (fixes the robot_2 hardcode in `deployment/scripts/start-pinky-robot2-session.sh:139`). Enforces the fixed domain↔port pairs 12↔9090, 13↔9091.

- [ ] **Step 1: Write the file**

```bash
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
```

- [ ] **Step 2: Syntax check + safety audit**

Run: `bash -n ros/pinky_control_bringup/scripts/robot_session.sh && grep -n 'robot_2\|9091' ros/pinky_control_bringup/scripts/robot_session.sh`
Expected: syntax OK; `robot_2`/`9091` appear ONLY in the domain/port guard message and comments, never as assigned values

- [ ] **Step 3: Commit**

```bash
git add ros/pinky_control_bringup/scripts/robot_session.sh
git -c user.name="opencode" -c user.email="opencode@localhost" commit -m "feat: generic control session unit payload (env-driven robot/domain/port)"
```

---

### Task 5: systemd unit templates + per-robot env files

**Files:**
- Create: `ros/pinky_control_bringup/systemd/pinky-bringup@.service`
- Create: `ros/pinky_control_bringup/systemd/pinky-session@.service`
- Create: `ros/pinky_control_bringup/config/robot_1.env`
- Create: `ros/pinky_control_bringup/config/robot_2.env`

**Interfaces:**
- Consumes: `~/pinky/startup/bin/robot_{bringup,session}.sh` (installed by Task 6 at this fixed path), env files installed to `~/.config/pinky-control/%i.env`
- Produces: instances `pinky-bringup@robot_1|2.service`, `pinky-session@robot_1|2.service`. Session Requires+After+PartOf bringup. Both Restart=on-failure, RestartSec=5s, StartLimit 5/120s.

- [ ] **Step 1: Write `systemd/pinky-bringup@.service`**

```ini
[Unit]
Description=Pinky hardware bringup for %i (rosbridge control platform)
Wants=network-online.target
After=network-online.target
StartLimitIntervalSec=120
StartLimitBurst=5

[Service]
Type=exec
EnvironmentFile=%h/.config/pinky-control/%i.env
ExecStart=%h/pinky/startup/bin/robot_bringup.sh
Restart=on-failure
RestartSec=5s

[Install]
WantedBy=default.target
```

- [ ] **Step 2: Write `systemd/pinky-session@.service`**

```ini
[Unit]
Description=Pinky control session (rosbridge+watchdog+Nav2) for %i
Requires=pinky-bringup@%i.service
After=pinky-bringup@%i.service
PartOf=pinky-bringup@%i.service
StartLimitIntervalSec=120
StartLimitBurst=5

[Service]
Type=exec
EnvironmentFile=%h/.config/pinky-control/%i.env
ExecStart=%h/pinky/startup/bin/robot_session.sh
Restart=on-failure
RestartSec=5s

[Install]
WantedBy=default.target
```

- [ ] **Step 3: Write `config/robot_1.env`**

```bash
# Sourced by pinky-bringup@robot_1.service / pinky-session@robot_1.service
# (systemd EnvironmentFile — no shell expansion, no quoting tricks).
ROBOT_ID=robot_1
ROS_DOMAIN_ID=12
ROSBRIDGE_PORT=9090
START_NAV2=1
PINKY_PRO_WS=/home/pinky/pinky_pro
PINKY_CONTROL_WS=/home/pinky/dev_ws/wj
```

- [ ] **Step 4: Write `config/robot_2.env`**

```bash
# Sourced by pinky-bringup@robot_2.service / pinky-session@robot_2.service
# (systemd EnvironmentFile — no shell expansion, no quoting tricks).
ROBOT_ID=robot_2
ROS_DOMAIN_ID=13
ROSBRIDGE_PORT=9091
START_NAV2=1
PINKY_PRO_WS=/home/pinky/pinky_pro
PINKY_CONTROL_WS=/home/pinky/dev_ws/wj
```

- [ ] **Step 5: Verify unit syntax offline**

Run: `systemd-analyze --user verify /home/ai/robot/control-platform/ros/pinky_control_bringup/systemd/pinky-bringup@.service /home/ai/robot/control-platform/ros/pinky_control_bringup/systemd/pinky-session@.service; echo "exit=$?"`
Expected: `exit=0` (missing ExecStart payload/env-file paths are fine offline — verify reports them only as warnings; treat "cannot open" on `%h/...` as acceptable, structural errors are not)

- [ ] **Step 6: Commit**

```bash
git add ros/pinky_control_bringup/systemd/ ros/pinky_control_bringup/config/
git -c user.name="opencode" -c user.email="opencode@localhost" commit -m "feat: template user units and per-robot env files"
```

---

### Task 6: `scripts/install.sh` (on-robot installer) + spec sync

**Files:**
- Create: `ros/pinky_control_bringup/scripts/install.sh`
- Modify: `docs/superpowers/specs/2026-09-19-pinky-startup-package-design.md` (installer line: fixed `%h/pinky/startup/bin` path, drop sed rendering; bringup script keeps migration stop of both rosy units)

**Interfaces:**
- Consumes: package layout from Tasks 1–5 (works from repo checkout OR colcon share dir); `SRC_ROOT = dirname(dirname(script))`
- Produces: installed payload at `~/pinky/startup/bin/`, env at `~/.config/pinky-control/<instance>.env`, units at `~/.config/systemd/user/`, legacy `rosy-session-*` disabled, linger verified, both unit instances enabled+started

- [ ] **Step 1: Write `scripts/install.sh`**

```bash
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
```

- [ ] **Step 2: Syntax check**

Run: `bash -n ros/pinky_control_bringup/scripts/install.sh`
Expected: exit 0

- [ ] **Step 3: Sync spec** — in `docs/superpowers/specs/2026-09-19-pinky-startup-package-design.md`, replace the installer step "copies scripts/ → `~/pinky/startup/bin/`, env file → `~/.config/pinky-control/<instance>.env`, renders unit templates (sed: `@BIN_DIR@`) → `~/.config/systemd/user/`" with "copies scripts/ → `~/pinky/startup/bin/` (fixed convention also hardcoded in the unit `ExecStart=%h/pinky/startup/bin/...`, no rendering), env file → `~/.config/pinky-control/<instance>.env`, units → `~/.config/systemd/user/`"; and in the `robot_bringup.sh` bullet note it stops both legacy `rosy-session-*` units at startup (migration guard).

- [ ] **Step 4: Commit**

```bash
git add ros/pinky_control_bringup/scripts/install.sh docs/superpowers/specs/2026-09-19-pinky-startup-package-design.md
git -c user.name="opencode" -c user.email="opencode@localhost" commit -m "feat: on-robot installer for boot auto-start units"
```

---

### Task 7: Package `AGENTS.md` + `ros/AGENTS.md` row

**Files:**
- Create: `ros/pinky_control_bringup/AGENTS.md`
- Modify: `ros/AGENTS.md` (subdirectory table)

**Interfaces:**
- Consumes: all prior tasks (documents their contracts)
- Produces: agent-facing docs; `ros/AGENTS.md` gains the row `| pinky_control_bringup/ | ament_python: boot auto-start scripts, systemd user units, per-robot env files, on-robot installer (see its AGENTS.md) |`

- [ ] **Step 1: Write `ros/pinky_control_bringup/AGENTS.md`**

```markdown
# pinky_control_bringup

## Purpose
ament_python package: robot-side boot auto-start. Two generalized runtime scripts (hardware bringup, control session), template systemd **user** units instantiated per robot (`pinky-bringup@<instance>`, `pinky-session@<instance>`), per-robot env files, and a no-sudo installer run on the robot over SSH. Both robots start their full stack at boot: bringup → rosbridge → watchdog → Nav2. Watchdog boots STOP-latched (no auto-resume), so auto-start is safe; AMCL initial pose stays a manual web-UI step after every reboot.

## Key Files
| File | Description |
|------|-------------|
| `scripts/robot_bringup.sh` | Unit payload for `pinky-bringup@`: sources Jazzy + both overlays (`set -u` only after), runs vendor `pinky_bringup bringup_robot.launch.xml` in the foreground, gates `/odom`+`/scan` (45 s each), stops legacy `rosy-session-*` units as a migration guard |
| `scripts/robot_session.sh` | Unit payload for `pinky-session@`: rosbridge + watchdog (env-driven `robot_id`) + optional Nav2 (`START_NAV2`); duplicate `pgrep` guards (bracket patterns), `setsid` groups, health gates `/control/status` (15 s) + `/navigate_to_pose` (60 s), `wait -n` supervisor |
| `scripts/lib/wait_for.sh` | Shared gates: `wait_for_publisher`, `wait_for_action_server`, `start_lidar_motor` (returns, does not exit) |
| `scripts/install.sh <robot_1\|robot_2>` | On-robot installer: verifies workspaces built, copies payload → `~/pinky/startup/bin/`, env → `~/.config/pinky-control/<i>.env`, units → `~/.config/systemd/user/`, disables legacy `rosy-session-*`, verifies linger, `enable --now` both units |
| `systemd/pinky-bringup@.service` | Template unit: `EnvironmentFile=%h/.config/pinky-control/%i.env`, `Restart=on-failure` 5 s, `StartLimitBurst=5`/120 s, `WantedBy=default.target` |
| `systemd/pinky-session@.service` | Same + `Requires`/`After`/`PartOf` `pinky-bringup@%i.service` — a session crash never re-opens serial devices |
| `config/robot_1.env` | `ROBOT_ID=robot_1`, `ROS_DOMAIN_ID=12`, `ROSBRIDGE_PORT=9090`, `START_NAV2=1`, workspace paths |
| `config/robot_2.env` | `ROBOT_ID=robot_2`, `ROS_DOMAIN_ID=13`, `ROSBRIDGE_PORT=9091`, `START_NAV2=1`, workspace paths |

## For AI Agents

### Working In This Directory
- Scripts take NO positional args and NO robot literals — every input comes from the unit `EnvironmentFile`. `robot_session.sh` hard-fails unless the domain/port pair is the fixed mapping 12↔9090 or 13↔9091.
- Keep the proven idioms when editing: `set -u` only after sourcing ROS overlays, `unset ROS_LOCALHOST_ONLY`, `PATH=/usr/bin:/bin` first, bracket-pattern `pgrep -f '[x]...'`, per-process `setsid` + INT group-kill cleanup.
- The watchdog stays the SOLE `/cmd_vel` publisher; never add a motion publisher here. Nav2 keeps its `/control/nav_velocity` remap (owned by `../pinky_control_navigation/`).
- Units run WITHOUT an SSH session (linger). Payload path `~/pinky/startup/bin/` is a fixed convention shared by `install.sh` and the unit files — change all of them together or none.
- Legacy `~/pinky/stack_start.sh` and `deployment/scripts/start-pinky-robot2-*.sh` remain as manual fallbacks; `install.sh` refuses to run while those wrappers are alive.

### Testing Requirements
- No automated tests (repo convention for `ros/`). Gate per change: `bash -n` on every script, `systemd-analyze --user verify` on unit templates, `colcon build --packages-select pinky_control_bringup` smoke.
- Field gate per robot: reboot → `journalctl --user -u 'pinky-*'` shows bringup gates then session READY → `/control/status` publishing, watchdog STOP-latched → backend ONLINE → set AMCL pose in UI → low-speed drive.

## Dependencies

### External
- systemd (user units + linger), ROS Jazzy, `rosbridge_server`, Nav2 (via `pinky_control_navigation`)

### Internal
- Vendor bringup `pinky_bringup` (workspace `~/pinky_pro`, NOT this repo); `../pinky_control_watchdog/`, `../pinky_control_navigation/`, `../pinky_control_interfaces/` built in `~/dev_ws/wj`.
- Consumers: field operators (install/runbook) and the backend (starts AFTER these units — typeless `/control/status` subscription needs the watchdog up first; backend restart after robot reboot remains a manual step).
```

- [ ] **Step 2: Add row to `ros/AGENTS.md`** — in the "Subdirectories" table, add above `pinky_control_navigation/`:

```markdown
| `pinky_control_bringup/` | ament_python: boot auto-start — generic bringup/session scripts, template systemd user units, per-robot env files, on-robot installer (see its `AGENTS.md`) |
```

- [ ] **Step 3: Commit**

```bash
git add ros/pinky_control_bringup/AGENTS.md ros/AGENTS.md
git -c user.name="opencode" -c user.email="opencode@localhost" commit -m "docs: pinky_control_bringup agent docs"
```

---

### Task 8: Runbook + user guide updates (Korean)

**Files:**
- Modify: `runbook.md` §6.2 (lines ~160–176: replace wrapper paragraph with systemd flow)
- Modify: `docs/user-guide.md` (lines ~242–274: replace manual wrapper flow with install + systemd flow)

**Interfaces:**
- Consumes: `install.sh` usage from Task 6; unit names from Task 5
- Produces: field documentation matching the new boot flow

- [ ] **Step 1: Update `runbook.md` §6.2** — replace the paragraph and code block from "일상적인 재부팅 후 기동은 위 두 workspace와 기존 부팅 서비스를 통합 관리하는" through "웹에서 실제 위치·방향을 지정하고 `위치 재설정(AMCL)`을 수행해야 한다." (keep the following "아래의 터미널 A/B 절차는 ... 분리 진단 절차다." line) with:

```markdown
일상적인 재부팅 후 기동은 systemd user unit(`pinky-bringup@<robot>`, `pinky-session@<robot>`)이 담당한다. 최초 1회는 로봇마다 설치 스크립트를 실행한다. 저장소의 `ros/pinky_control_bringup`을 로봇의 `/home/pinky/dev_ws/wj/src/` 아래에 복사해 `colcon build --symlink-install --packages-select pinky_control_bringup`으로 빌드한 뒤, 다음을 실행한다.

```bash
~/dev_ws/wj/install/pinky_control_bringup/share/pinky_control_bringup/scripts/install.sh robot_2
```

설치 스크립트는 workspace 빌드 여부를 확인하고, payload를 `~/pinky/startup/bin/`에, 환경 파일을 `~/.config/pinky-control/<robot>.env`에, unit을 `~/.config/systemd/user/`에 복사한 뒤 기존 `rosy-session-*` 부팅 서비스를 비활성화하고 linger를 확인한 뒤 두 unit을 `enable --now` 한다. sudo는 필요 없다. 이후 재부팅에서는 SSH 접속 없이 두 unit이 자동으로 시작된다. robot_1은 control workspace가 아직 없으므로 `docs/user-guide.md`의 절차로 workspace를 먼저 빌드해야 설치가 통과한다.

상태와 로그는 다음으로 확인한다.

```bash
systemctl --user status 'pinky-*'
journalctl --user -u 'pinky-*' -f
```

unit이 정상이면 `[pinky-bringup] ready`와 `[pinky-session] ready`가 로그에 남는다. 재부팅 뒤에는 AMCL 위치가 사라지므로 웹에서 실제 위치·방향을 지정하고 `위치 재설정(AMCL)`을 수행해야 한다. 세션 비정상 종료 시 systemd가 5초 뒤 재시작하며(120초당 최대 5회), 반복 실패는 `journalctl`로 원인을 확인한다.
```

- [ ] **Step 2: Update `docs/user-guide.md`** — replace lines 242–274 (the three paragraphs + two code blocks describing `start-pinky-robot2-all.sh` scp/foreground flow, and the paragraph at 272–274 about running `start-robot2.sh` / standalone session script) with:

```markdown
robot_2 실물 시험의 권장 기동 방법은 systemd user unit 기반 부팅 자동 기동이다. `ros/pinky_control_bringup` 패키지가 하드웨어 bringup(`pinky-bringup@<robot>`)과 관제 세션(rosbridge·watchdog·Nav2, `pinky-session@<robot>`)을 로봇별로 관리하며, 두 unit은 부팅 때 자동으로 시작된다. 세션 unit은 bringup unit에 의존하므로 세션 재시작이 시리얼 장치를 다시 열지 않는다. 두 스크립트 모두 `ROS_LOCALHOST_ONLY`를 해제하고 `ROS_DOMAIN_ID`는 환경 파일(`robot_1=12`, `robot_2=13`)에서 고정한다.

최초 설치는 로봇에서 한 번만 실행한다. 저장소의 `ros/pinky_control_bringup`을 `/home/pinky/dev_ws/wj/src/` 아래에 복사해 빌드한 뒤 설치 스크립트를 실행한다.

```bash
cd /home/pinky/dev_ws/wj
colcon build --symlink-install --packages-select pinky_control_bringup
./install/pinky_control_bringup/share/pinky_control_bringup/scripts/install.sh robot_2
```

설치가 끝나면 로봇을 재부팅해도 SSH로 무언가를 실행할 필요가 없다. 웹을 새로고치고 `robot_2`를 선택한다. 재부팅하면 AMCL 추정 위치는 유지되지 않으므로 실제 위치와 방향을 지도에 지정하고 `위치 재설정(AMCL)`을 반드시 한 번 수행한다. 부팅 직후 watchdog는 정지 래치가 걸린 상태이므로 제어권 획득 → `robot_2 정지 해제` → `MANUAL 모드 전환` 순서로 준비한다.

로그와 상태 확인은 `systemctl --user status 'pinky-*'`와 `journalctl --user -u 'pinky-*' -f`로 한다. 수동 기동이 필요한 진단 상황에서는 기존 `deployment/scripts/start-pinky-robot2-*.sh`를 그대로 쓸 수 있지만, 이 스크립트들은 unit과 동시에 실행할 수 없다(중복 프로세스 감지로 실패한다). 실행 중인 로봇은 정지 상태에서 시험한다.
```

(Leave the workspace-build paragraph at 263–270 and the final paragraph at 276 untouched, except: line 272's `그 다음 start-robot2.sh...` sentence is part of what gets replaced above.)

- [ ] **Step 3: Commit**

```bash
git add runbook.md docs/user-guide.md
git -c user.name="opencode" -c user.email="opencode@localhost" commit -m "docs: runbook and user guide for boot auto-start units"
```

---

### Task 10: Enable robot_1 onboard control contract (master/slave switch parity)

**Files:**
- Modify: `backend/pinky_control_center/resources/config/robots.ros.yaml:27` (`control_available: false → true`)
- Modify: `deployment/robots.ros.example.yaml` (robot_1 `control_available: false → true`)

**Interfaces:**
- Consumes: startup package gives robot_1 an onboard watchdog (`/control/command` service) once field-installed
- Produces: backend treats robot_1 as a controllable robot over its onboard rosbridge (`ws://robot-1.local:9090`), enabling robot_2-MASTER / robot_1-SLAVE formation pairing once `follow` capability lands robot-side. `robots.ros.local.yaml` stays PC-gateway (pinned by `test_ros_smoke_local.py`) — do NOT touch it.

- [ ] **Step 1: Flip the two YAML values**

In `backend/pinky_control_center/resources/config/robots.ros.yaml` robot_1 services block:

```yaml
    services:
      control_command: /control/command
      control_command_type: pinky_control_interfaces/srv/ControlCommand
      control_available: true
```

Same change in `deployment/robots.ros.example.yaml` robot_1 services block.

- [ ] **Step 2: Run backend config tests**

Run: `backend/.venv/bin/python -m pytest backend/tests/test_ros_smoke_local.py backend/tests/test_contracts.py -q` (repo root)
Expected: PASS (local.yaml untouched; robots.ros.yaml not pinned by tests)

- [ ] **Step 3: Commit**

```bash
git add backend/pinky_control_center/resources/config/robots.ros.yaml deployment/robots.ros.example.yaml
git -c user.name="opencode" -c user.email="opencode@localhost" commit -m "feat: robot_1 onboard control contract for master/slave parity"
```

---

### Task 9: Final verification

**Files:** none created (verification + any fixes)

**Interfaces:**
- Consumes: Tasks 1–8 complete

- [ ] **Step 1: colcon build smoke** (proves packaging installs all data_files)

```bash
rm -rf /tmp/opencode/pcb_ws && mkdir -p /tmp/opencode/pcb_ws/src
cp -r /home/ai/robot/control-platform/ros/pinky_control_bringup /tmp/opencode/pcb_ws/src/
source /opt/ros/jazzy/setup.bash && colcon build --packages-select pinky_control_bringup --event-handlers console_direct-
find /tmp/opencode/pcb_ws/install/pinky_control_bringup/share/pinky_control_bringup -type f | sort
```
Workdir: `/tmp/opencode/pcb_ws`
Expected: build succeeds; listing shows `scripts/{install.sh,robot_bringup.sh,robot_session.sh}`, `scripts/lib/wait_for.sh`, `systemd/*.service`, `config/*.env`, `package.xml`

- [ ] **Step 2: Installer works from share dir** (layout assumption check)

```bash
bash -n /tmp/opencode/pcb_ws/install/pinky_control_bringup/share/pinky_control_bringup/scripts/install.sh
test -f /tmp/opencode/pcb_ws/install/pinky_control_bringup/share/pinky_control_bringup/systemd/pinky-bringup@.service && echo LAYOUT_OK
```
Expected: `LAYOUT_OK`

- [ ] **Step 3: Full syntax + invariant audit**

```bash
for f in /home/ai/robot/control-platform/ros/pinky_control_bringup/scripts/*.sh /home/ai/robot/control-platform/ros/pinky_control_bringup/scripts/lib/wait_for.sh; do bash -n "$f" || exit 1; done && echo SYNTAX_OK
grep -rn 'cmd_vel' /home/ai/robot/control-platform/ros/pinky_control_bringup/scripts/ | grep -v 'cmd_vel_topic\|nav_velocity'
```
Expected: `SYNTAX_OK`; the only bare `/cmd_vel` references are watchdog params/echo lines — no new publisher anywhere

- [ ] **Step 4: Clean up scratch workspace**

```bash
rm -rf /tmp/opencode/pcb_ws
```

- [ ] **Step 5: Report** — summarize commits + remaining field gates (linger verification on real robots, robot_1 workspace build, reboot test, backend restart step unchanged).
