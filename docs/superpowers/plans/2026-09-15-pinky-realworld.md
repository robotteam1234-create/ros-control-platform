# Pinky Real-World Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Run control-platform against real Pinkys via PC rosbridge gateway with full missions + Nav2 and both cameras.

**Architecture:** Robots on DDS domains 12 (764e robot_1 MASTER) and 13 (1e3e robot_2 SLAVE) stream to this PC; PC runs 2x rosbridge_websocket (9090/9091); existing RosbridgeAdapter connects via ws://127.0.0.1; robot-side watchdog + navigation + camera republisher provide /control/* and Nav2.

**Tech Stack:** ROS 2 Jazzy, rosbridge_server, Python FastAPI backend, rclpy onboard packages, image_transport republisher, Vite frontend (unchanged).

**Spec:** docs/superpowers/specs/2026-09-15-pinky-realworld-design.md

## Global Constraints

- robot_1 domain_id must be 12 and robot_2 domain_id must be 13 (config.py fixed_deployment_contract, validate-config.py).
- CONTROL_PLATFORM_WORKERS must be 1 (single-worker settings application).
- Max teleop 0.15 m/s linear / 0.50 rad/s angular, manual deadman 0.35 s, nav deadman 0.50 s (watchdog defaults).
- Boot latched STOP with STARTUP_STOP_LATCH, no auto-resume after restart.
- Map goal tolerance 0.08 m planar / 0.17 rad yaw.
- Secrets via env only, never in YAML/git/logs.
- Unrun real-world checks stay NOT_RUN in acceptance-report.md, never PASS by default.

---

### Task 1: PC rosbridge gateway up

**Files:**
- Modify: `deployment/launch/control_center.launch.py:25-39` (no change expected, verify ports/domains)
- Test: shell verify only (no new test file)

**Interfaces:**
- Consumes: ROS 2 Jazzy on PC, domains 12/13 DDS traffic from robots.
- Produces: `ws://127.0.0.1:9090` (domain 12), `ws://127.0.0.1:9091` (domain 13) for Task 5.

- [ ] **Step 1: Install rosbridge_server on PC**

```bash
source /opt/ros/jazzy/setup.bash
sudo apt-get update
sudo apt-get install -y ros-jazzy-rosbridge-server
ros2 pkg prefix rosbridge_server
```

- [ ] **Step 2: Run it and verify it fails before robots are realigned (expected)**

```bash
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 deployment/launch/control_center.launch.py
```

Expected: two rosbridge processes start; backend not yet connected so ports listen idle.

- [ ] **Step 3: Verify both ports listen**

```bash
ss -tlnp | grep -E '9090|9091'
```

Expected: LISTEN on 127.0.0.1:9090 and 127.0.0.1:9091 (or 0.0.0.0). If missing, stop here and fix install.

- [ ] **Step 4: Commit if launch file changed, else skip**

```bash
git status --short deployment/launch/
```

Expected: clean (launch file already correct). Only commit if you edited it.

### Task 2: Robot domain realignment 12/13 + retire domain_bridge

**Files:**
- Modify (on robots via SSH, not in this repo): `~/pinky/stack_start.sh` or systemd unit `ROS_DOMAIN_ID`, 764e 0->12, 1e3e 12->13
- Modify: `deployment/robots.ros.local.yaml:10,36` (domain_id already 12/13, verify)

**Interfaces:**
- Consumes: SSH access pinky@192.168.1.201 (764e), pinky@192.168.1.202 (1e3e).
- Produces: isolated domains 12/13 observable from PC for Tasks 3-5.

- [ ] **Step 1: Confirm current identity over SSH**

```bash
ssh pinky@192.168.1.201 'hostname; cat /etc/hostname 2>/dev/null; echo ---; env | grep ROS_DOMAIN || true'
ssh pinky@192.168.1.202 'hostname; cat /etc/hostname 2>/dev/null; echo ---; env | grep ROS_DOMAIN || true'
```

Expected: identify which host is 764e vs 1e3e before changing anything.

- [ ] **Step 2: Set 764e to domain 12 and 1e3e to domain 13 (persistent)**

```bash
ssh pinky@192.168.1.201 'grep -rn ROS_DOMAIN_ID ~/pinky/stack_start.sh ~/.config/systemd/user/ 2>/dev/null | head'
```

Expected: locate the file that sets the domain, edit it to `export ROS_DOMAIN_ID=12` on .201 and `export ROS_DOMAIN_ID=13` on .202, restart bringup. Exact file path varies by robot; do not guess, read it first.

- [ ] **Step 3: Verify isolation from PC**

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=12
timeout 10 ros2 topic list | grep -E '/odom|/scan|/battery'
export ROS_DOMAIN_ID=13
timeout 10 ros2 topic list | grep -E '/odom|/scan|/battery'
```

Expected: both domains list /odom /scan /battery/percent. If domain 13 is empty, the 1e3e remap did not take.

- [ ] **Step 4: Stop domain_bridge control path to avoid dual cmd_vel writers**

```bash
ps aux | grep domain_bridge | grep -v grep
kill -INT 11759 2>/dev/null || true
```

Expected: domain_bridge processes exit; keep `~/domain_bridge_ws/pinky_dual_bridge.yaml` file untouched. Backend will own /cmd_vel via watchdog after Task 3.

### Task 3: Robot control packages (status/command/watchdog)

**Files:**
- Create (on robots): `/home/pinky/dev_ws/wj/src/pinky_control_interfaces`, `/home/pinky/dev_ws/wj/src/pinky_control_watchdog` (copied from `ros/`)
- Test: `ros2 topic info /control/status -v`, `ros2 service type /control/command`

**Interfaces:**
- Consumes: `ros/pinky_control_interfaces/msg/ControlStatus.msg`, `ros/pinky_control_interfaces/srv/ControlCommand.srv`, `ros/pinky_control_watchdog/pinky_control_watchdog/manual_velocity_watchdog.py:41-80` (params: robot_id, manual_topic /control/manual_velocity, nav_topic /control/nav_velocity, cmd_vel_topic /cmd_vel, manual_timeout_sec 0.35, nav_timeout_sec 0.50, max_linear_mps 0.15, max_angular_rps 0.50).
- Produces: `/control/status` 10 Hz, `/control/command` service, single `/cmd_vel` publisher (watchdog) for Task 5.

- [ ] **Step 1: Copy packages to both robots**

```bash
scp -r ros/pinky_control_interfaces ros/pinky_control_watchdog pinky@192.168.1.201:/home/pinky/dev_ws/wj/src/
scp -r ros/pinky_control_interfaces ros/pinky_control_watchdog pinky@192.168.1.202:/home/pinky/dev_ws/wj/src/
```

- [ ] **Step 2: Build on 764e (domain 12)**

```bash
ssh pinky@192.168.1.201 'source /opt/ros/jazzy/setup.bash && cd /home/pinky/dev_ws/wj && colcon build --symlink-install --packages-select pinky_control_interfaces pinky_control_watchdog'
```

Expected: build PASS. If rosdep keys fail, run `rosdep install --from-paths src/pinky_control_interfaces src/pinky_control_watchdog -y --ignore-src` first.

- [ ] **Step 3: Build on 1e3e (domain 13)**

```bash
ssh pinky@192.168.1.202 'source /opt/ros/jazzy/setup.bash && cd /home/pinky/dev_ws/wj && colcon build --symlink-install --packages-select pinky_control_interfaces pinky_control_watchdog'
```

Expected: build PASS.

- [ ] **Step 4: Start watchdog no-motion and verify single cmd_vel publisher**

```bash
ssh pinky@192.168.1.201 'source /opt/ros/jazzy/setup.bash && source /home/pinky/dev_ws/wj/install/setup.bash && export ROS_DOMAIN_ID=12 && ros2 run pinky_control_watchdog manual_velocity_watchdog --ros-args -p robot_id:=robot_1' &
sleep 5
source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=12
ros2 topic info /control/status -v | head -n 20
ros2 topic info /cmd_vel -v | grep -E 'Publisher count|Subscription count'
```

Expected: `/control/status` publisher count 1, `/cmd_vel` publisher count exactly 1 (watchdog). Repeat with `ROS_DOMAIN_ID=13` and `robot_id:=robot_2` on .202. Kill test processes after verify.

### Task 4: Camera pipeline both robots

**Files:**
- Modify (on robots): camera republisher launch (image_transport republish or libcamera node -> `/camera/image_raw/compressed`)
- Modify: `deployment/robots.ros.local.yaml:28-31,57-60` (camera.enabled false->true)
- Test: backend JPEG HTTP 200 per robot in Task 5

**Interfaces:**
- Consumes: onboard camera topic (discover via `ros2 topic list | grep camera`), `sensor_msgs/msg/CompressedImage` on `/camera/image_raw/compressed`.
- Produces: compressed frames for `RosbridgeCameraOptions{enabled=True, throttle_rate_ms=100, fragment_size=65536}`.

- [ ] **Step 1: Discover real camera topics on each domain**

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=12; timeout 10 ros2 topic list | grep -i camera
export ROS_DOMAIN_ID=13; timeout 10 ros2 topic list | grep -i camera
```

Expected: at least one raw image topic per robot (e.g. /camera/front, /camera/image_raw). If none, stop: camera hardware/bringup missing, leave `camera.enabled=false` and record NOT_RUN.

- [ ] **Step 2: Start compressed republisher per robot (example for image_transport)**

```bash
export ROS_DOMAIN_ID=12
ros2 run image_transport republish raw in:=/camera/image_raw raw out:=/camera/image_raw --ros-args -p qos:=sensor_data &
```

Expected: `/camera/image_raw/compressed` appears in `ros2 topic list`. Adjust `in:=` to the real topic from Step 1. Repeat for domain 13.

- [ ] **Step 3: Enable cameras in local YAML (PC repo file)**

```yaml
# deployment/robots.ros.local.yaml, both robots:
    camera:
      enabled: true
      throttle_rate_ms: 100
      fragment_size: 65536
```

- [ ] **Step 4: Run backend config loader to verify it passes**

```bash
backend/.venv/bin/python -c "from pathlib import Path; from pinky_control_center.config import load_ros_config; c=load_ros_config(Path('deployment/robots.ros.local.yaml')); print([(r.robot_id, r.domain_id, r.bridge_url, r.camera.enabled) for r in c.robots])"
```

Expected: `[('robot_1', 12, 'ws://127.0.0.1:9090', False), ('robot_2', 13, 'ws://172.20.10.8:9091', False)]` (pre-fix values proving the loader works; Task 5 changes URLs/flags).

### Task 5: Backend YAML + ROS smoke (no motion)

**Files:**
- Modify: `deployment/robots.ros.local.yaml:11,37,27,53-56,28-31,57-60`
- Test: `backend/tests/test_ros_smoke_local.py` (new)

**Interfaces:**
- Consumes: Task 1 bridges, Task 3 /control/*, Task 4 compressed camera.
- Produces: backend `--mode ros` with both robots ONLINE for Task 6.

- [ ] **Step 1: Write the failing test**

```python
from pathlib import Path
from pinky_control_center.config import load_ros_config

def test_local_ros_yaml_is_pc_gateway():
    cfg = load_ros_config(Path("deployment/robots.ros.local.yaml"))
    by_id = {r.robot_id: r for r in cfg.robots}
    assert by_id["robot_1"].domain_id == 12
    assert by_id["robot_2"].domain_id == 13
    assert by_id["robot_1"].bridge_url == "ws://127.0.0.1:9090"
    assert by_id["robot_2"].bridge_url == "ws://127.0.0.1:9091"
    assert by_id["robot_1"].camera.enabled is True
    assert by_id["robot_2"].camera.enabled is True
    assert by_id["robot_1"].services.control_available is True
    assert by_id["robot_2"].services.control_available is True
```

Save as `backend/tests/test_ros_smoke_local.py`.

- [ ] **Step 2: Run test to verify it fails**

Run: `backend/.venv/bin/python -m pytest backend/tests/test_ros_smoke_local.py -v`
Expected: FAIL (robot_2 URL is ws://172.20.10.8:9091, cameras disabled, robot_1 control_available false).

- [ ] **Step 3: Write minimal YAML fix**

```yaml
robots:
  - robot_id: robot_1
    name: Pinky Master
    role: MASTER
    namespace: /robot_1
    domain_id: 12
    bridge_url: ws://127.0.0.1:9090
    topics:
      odom: /odom
      battery_percent: /battery/percent
      battery_voltage: /battery/voltage
      camera_compressed: /camera/image_raw/compressed
      control_status: /control/status
      scan: /scan
      path: /plan
      manual_velocity: /control/manual_velocity
      initial_pose: /initialpose
      tf: /tf
      tf_static: /tf_static
    services:
      control_command: /control/command
      control_command_type: pinky_control_interfaces/srv/ControlCommand
      control_available: true
    camera:
      enabled: true
      throttle_rate_ms: 100
      fragment_size: 65536
  - robot_id: robot_2
    name: Pinky Slave
    role: SLAVE
    namespace: /robot_2
    domain_id: 13
    bridge_url: ws://127.0.0.1:9091
    topics:
      odom: /odom
      battery_percent: /battery/percent
      battery_voltage: /battery/voltage
      camera_compressed: /camera/image_raw/compressed
      control_status: /control/status
      scan: /scan
      path: /plan
      manual_velocity: /control/manual_velocity
      initial_pose: /initialpose
      tf: /tf
      tf_static: /tf_static
    services:
      control_command: /control/command
      control_command_type: pinky_control_interfaces/srv/ControlCommand
      control_available: true
      follow_command: /follow/command
      follow_command_type: pinky_control_interfaces/srv/FollowCommand
      follow_available: false
    camera:
      enabled: true
      throttle_rate_ms: 100
      fragment_size: 65536
```

Keep `follow_available: false` (FollowCommand.srv does not exist yet).

- [ ] **Step 4: Run test to verify it passes**

Run: `backend/.venv/bin/python -m pytest backend/tests/test_ros_smoke_local.py -v`
Expected: PASS.

- [ ] **Step 5: No-motion ROS smoke with real backend**

```bash
kill 19107 2>/dev/null || true
sleep 2
CONTROL_PLATFORM_WORKERS=1 backend/.venv/bin/python -m pinky_control_center.main --mode ros --config deployment/robots.ros.local.yaml --host 127.0.0.1 --port 8081 --database ~/.local/state/control-platform/control.db --allowed-origin http://localhost:5173 &
sleep 3
curl -s http://127.0.0.1:8081/health
```

Expected: `{"status":"ok","mode":"ros"}` (mock backend on 8081 must be stopped first — it conflicts on the same port). Then login and check `/api/v1/state` shows both ONLINE/FRESH, cameras HTTP 200, stop latched true. Do not command motion in this task.

- [ ] **Step 6: Commit**

```bash
git add deployment/robots.ros.local.yaml backend/tests/test_ros_smoke_local.py
git commit -m "feat: pc rosbridge gateway config with both cameras"
```

### Task 6: Nav2 + missions gate + acceptance update

**Files:**
- Create (on robots): `/home/pinky/dev_ws/wj/src/pinky_control_navigation` (from `ros/pinky_control_navigation/`, map `map_260905.pgm/yaml`, params `nav2_params.yaml`, launch `robot_nav2.launch.py`)
- Modify: `acceptance-report.md:32-46` (ROS/Gazebo gate table)
- Test: supervised single goal via UI, `ros2 action list -t | grep navigate_to_pose`

**Interfaces:**
- Consumes: Task 5 ONLINE backend, `ros/pinky_control_navigation/launch/robot_nav2.launch.py`, `ros/pinky_control_navigation/params/nav2_params.yaml` (goal tolerance 0.08/0.17), `nav2_lifecycle_gate.py`.
- Produces: map goals, AMCL reset, patrol/formation evidence.

- [ ] **Step 1: Deploy navigation package to both robots**

```bash
scp -r ros/pinky_control_navigation pinky@192.168.1.201:/home/pinky/dev_ws/wj/src/
scp -r ros/pinky_control_navigation pinky@192.168.1.202:/home/pinky/dev_ws/wj/src/
ssh pinky@192.168.1.201 'source /opt/ros/jazzy/setup.bash && cd /home/pinky/dev_ws/wj && colcon build --symlink-install --packages-select pinky_control_navigation'
ssh pinky@192.168.1.202 'source /opt/ros/jazzy/setup.bash && cd /home/pinky/dev_ws/wj && colcon build --symlink-install --packages-select pinky_control_navigation'
```

Expected: build PASS both. If Nav2 packages missing, `sudo apt-get install -y ros-jazzy-navigation2 ros-jazzy-nav2-bringup` on robots first.

- [ ] **Step 2: Verify Nav2 action + lifecycle with no motion**

```bash
export ROS_DOMAIN_ID=12
timeout 10 ros2 action list -t | grep navigate_to_pose
timeout 10 ros2 lifecycle get /map_server
timeout 10 ros2 lifecycle get /amcl
```

Expected: one navigate_to_pose server, map_server/amcl active or activatable. Repeat for domain 13. If TF `map->base_footprint` absent, use UI `위치 재설정(AMCL)` with a free-space start point first.

- [ ] **Step 3: Supervised single goal (wheels supervised, low speed)**

```bash
curl -s http://127.0.0.1:8081/api/v1/state -b /tmp/cookies.txt | backend/.venv/bin/python -c "import json,sys; d=json.load(sys.stdin); print([(r['robot_id'], r['connection'], r['tf_valid']) for r in d['robots']])"
```

Expected: both ONLINE, tf_valid true after AMCL reset. Then in UI: set start -> `위치 재설정(AMCL)` -> set goal -> move. Operator hand on stop. Record click goal vs final pose distance/yaw error.

- [ ] **Step 4: Update acceptance-report.md ROS gate rows**

```markdown
| robot_1 domain 12 / bridge 9090 | PASS | <date> 192.168.1.201 DDS + PC rosbridge 9090, ONLINE/odom FRESH |
| robot_2 domain 13 / bridge 9091 | PASS | <date> 192.168.1.202 DDS + PC rosbridge 9091, ONLINE/odom FRESH |
| 지도 시작점→AMCL→AUTO→목표 저속 주행 | PASS/PARTIAL | goal vs pose error m/rad, action status |
```

Only mark PASS with log evidence; else PARTIAL/NOT_RUN.

- [ ] **Step 5: Run full backend + frontend regression**

```bash
backend/.venv/bin/python -m pytest backend/tests/ -q 2>&1 | tail -n 5
cd frontend && npm run build 2>&1 | tail -n 5
```

Expected: pytest green (113+1 tests), vite build green.

- [ ] **Step 6: Commit**

```bash
git add acceptance-report.md
git commit -m "test: real-world nav2 gate evidence"
```
