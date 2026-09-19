# Pinky Startup Package — Robot-Side Boot Auto-Start (Design)

- Date: 2026-09-19
- Status: Approved (design discussed in session; see chat transcript)
- Scope: robot-side only (robot_1 + robot_2). PC-side backend/rosbridge lifecycle is out of scope.

## Goals

1. Both robots automatically bring up their full control stack at boot: hardware bringup → rosbridge → watchdog → Nav2.
2. Survive crashes (`Restart=on-failure`) and reboots (lingered systemd **user** units) without an SSH session holding anything alive.
3. One parameterized package instead of robot_2-named one-off scripts: `ROBOT_ID` / `ROS_DOMAIN_ID` / `ROSBRIDGE_PORT` come from per-robot env files.
4. Bringup and session are separate units so a session crash never re-opens the Dynamixel/lidar serial devices.

## Non-Goals

- No PC-side systemd changes (rosbridge gateway pair stays manual).
- No AMCL initial-pose auto-restore — after every reboot the operator sets the pose in the web UI (existing, intentional).
- No auto-resume of motion after boot: the watchdog boots STOP-latched and never auto-resumes; auto-start is therefore safe by design.
- No changes to `pinky_bringup` (vendor repo `~/pinky_pro`), the watchdog, or the Nav2 launch file.

## Current State (verified)

- Only the PC backend has a systemd unit. Robot startup = manual SSH + foreground `start-pinky-robot2-all.sh` (robot_2 only, domain 13 hardcoded; SSH drop kills the session).
- Robots boot with legacy domain-0 `rosy-session-bringup/-control.service` user units that the new flow stops every time. `~/pinky/stack_start.sh <domain>` is the manual fallback (domain fixed per robot: 764e→12, 1e3e→13).
- Ordering invariant: watchdog (`/control/status` publisher) must be up before the backend connects, else the typeless subscription never resolves; backend restart remains a manual remediation.

## Design

### Package

New ament_python package `ros/pinky_control_bringup/` (builds into the robots' control workspace `~/dev_ws/wj`):

```
pinky_control_bringup/
├── package.xml                     # exec_dep: none new beyond rclpy-free shell scripts; ament index only
├── setup.py                        # data_files: scripts/, systemd/, config/
├── setup.cfg
├── resource/pinky_control_bringup
├── scripts/
│   ├── robot_bringup.sh            # hardware bringup: vendor launch, gates /odom + /scan
│   ├── robot_session.sh            # rosbridge + watchdog + optional Nav2, gates /control/status + /navigate_to_pose
│   ├── install.sh                  # deploy-to-robot installer (run on robot over SSH)
│   └── lib/wait_for.sh             # shared wait_for_publisher / wait_for_action_server / start_lidar_motor
├── systemd/
│   ├── pinky-bringup@.service      # template unit
│   └── pinky-session@.service      # template unit
├── config/
│   ├── robot_1.env                 # ROBOT_ID=robot_1, ROS_DOMAIN_ID=12, ROSBRIDGE_PORT=9090, START_NAV2=1
│   └── robot_2.env                 # ROBOT_ID=robot_2, ROS_DOMAIN_ID=13, ROSBRIDGE_PORT=9091, START_NAV2=1
└── AGENTS.md
```

### Scripts

- Both runtime scripts source `/opt/ros/jazzy`, `$PINKY_PRO_WS` (default `/home/pinky/pinky_pro`), `$PINKY_CONTROL_WS` (default `/home/pinky/dev_ws/wj`); enable `set -u` only after sourcing; `unset ROS_LOCALHOST_ONLY`; `export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`, `ROS2CLI_NO_DAEMON=1`; `PATH=/usr/bin:/bin` first. All of this is copied verbatim from the proven `start-pinky-robot2-*.sh` behavior.
- `robot_session.sh` generalizes `start-pinky-robot2-session.sh`: watchdog gets `-p robot_id:="${ROBOT_ID}"`; everything else (clamp 0.15/0.50, topic names, duplicate `pgrep` guards with bracket patterns, `setsid` process groups, Ctrl+C group teardown, `wait -n` supervisor, lidar `/start_motor` before Nav2) unchanged. Duplicate-domain guard: fail if `ROS_DOMAIN_ID` is not 12 or 13 (typo protection, mirrors the robot_2=13 hard-fail).
- `robot_bringup.sh` runs `ros2 launch pinky_bringup bringup_robot.launch.xml` in the foreground (launch process = unit process, so a bringup crash is visible to systemd), gates on `/odom` + `/scan` publishers before reporting success, and stops both legacy `rosy-session-*` units at startup as a migration guard.

### Units

- `pinky-bringup@.service`: `EnvironmentFile=%h/.config/pinky-control/%i.env`, `ExecStart=%h/pinky/startup/bin/robot_bringup.sh`, `Restart=on-failure`, `RestartSec=5s`, `WantedBy=default.target`.
- `pinky-session@.service`: same env file; `Requires=pinky-bringup@%i.service` + `After=pinky-bringup@%i.service` + `PartOf=pinky-bringup@%i.service`; `Restart=on-failure`, `RestartSec=5s`.
- Both: `StartLimitIntervalSec=120`, `StartLimitBurst=5` (no serial-port hammering on repeated failure).
- Session script's health-gate failure exits non-zero ⇒ systemd restarts it; bringup keeps running (hardware untouched).

### Installer (`install.sh <robot_1|robot_2>`)

Runs **on the robot** (no sudo anywhere):
1. Verifies: workspaces built (`pinky_bringup`, `pinky_control_watchdog`, `pinky_control_navigation` prefixes resolvable), rosbridge present.
2. Copies `scripts/` → `~/pinky/startup/bin/` (fixed convention, also hardcoded in the unit `ExecStart=%h/pinky/startup/bin/...` — no rendering), env file → `~/.config/pinky-control/<instance>.env`, unit templates → `~/.config/systemd/user/`.
3. `systemctl --user daemon-reload`; disables (not deletes) `rosy-session-bringup.service` and `rosy-session-control.service`.
4. Checks linger (`loginctl show-user`); attempts `loginctl enable-linger` for the current user, warns with exact remediation if it fails.
5. `systemctl --user enable --now pinky-bringup@<i>.service pinky-session@<i>.service`.

## Invariants Preserved

- Watchdog remains the sole `/cmd_vel` publisher; Nav2 keeps the `cmd_vel → /control/nav_velocity` remap; `set_initial_pose: False` + lifecycle gate untouched.
- Boot-stop latch: robots come up STOPPED; operator resets in the UI.
- Domain isolation: robot_1 d12/:9090, robot_2 d13/:9091 — enforced by env files + script guard.

## Legacy & Migration

- `install.sh` disables the legacy `rosy-session-*` units (they conflict on hardware access).
- Existing `start-pinky-robot2-*.sh` stay in `deployment/scripts/` until field validation, then a follow-up removes them.

## Verification

- Repo-level: `bash -n` on every script; `shellcheck` where available (no CI gate today); unit templates validated with `systemd-analyze --user verify` on a robot or locally.
- Field gate per robot (documented in runbook): reboot → `journalctl --user -u 'pinky-*'` shows bringup gates passing then session READY → `/control/status` publishing → backend sees robot ONLINE, STOP-latched → operator sets AMCL pose → drive.
- Docs updated: `docs/runbook.md`, `docs/user-guide.md` (replace `/home/pinky/start-robot2.sh` flow), `ros/AGENTS.md` + new package `AGENTS.md`.

## Risks

- Linger may already be enabled (legacy rosy units prove boot-time user services work on these robots) — installer verifies rather than assumes.
- Repeated bringup crash loops on serial faults are bounded by StartLimitBurst; operator intervention via `journalctl` is the documented remediation.
