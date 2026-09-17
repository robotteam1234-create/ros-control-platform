# Line-follow design — white or black tape, control-platform first

Date: 2026-09-17 · Status: approved design (architectural path) · Scope: backend MVP for `robot_1`

## 1. Goal
Pinky Pro `robot_1` (MASTER, domain 12) follows a floor tape line — white or black —
with a forward-facing camera. First step lives entirely in control-platform:
backend OpenCV service + UI Start/Stop. No robot-side build in MVP.

Success: `robot_1` tracks tape slowly, stops cleanly (software-only stop) on line loss.

## 2. Architecture
```
robot_1 camera (Picamera2/V4L2)
  -> ros/pinky_camera_pub.py --topic /camera/image_raw/compressed --fps 10
  -> rosbridge :9090 (domain 12) sensor_msgs/CompressedImage
  -> backend RosbridgeAdapter._decode_camera -> adapter.frame(robot_1)
  -> CameraService -> LineFollowService.detect() @10Hz
  -> adapter.publish_manual_velocity (TwistStamped /control/manual_velocity)
  -> ros/pinky_control_watchdog (SOLE /cmd_vel publisher, clamp 0.15/0.50)
```

Browser never touches ROS. Backend holds the rosbridge websocket.
`CONTROL_PLATFORM_WORKERS=1` unchanged.

## 3. Components
- `backend/pinky_control_center/line_detector.py` (new, pure OpenCV):
  input JPEG bytes -> output `{offset, area, polarity, found}`.
  - Decode grayscale, Gaussian blur (5x5).
  - ROI: bottom 50% (forward floor), optional perspective crop later.
  - Auto polarity: bright-pixel ratio in ROI. If ROI mean < 127 assume dark floor /
    white line -> threshold `>200`; else (bright floor) invert for black line (`<60`).
    Manual override `mode: auto|white|black` in op params + UI selector.
  - Morph open 5x5, find contours, keep largest by area.
  - Moments `cx`; `offset = (cx - w/2)/(w/2)` in [-1,1]. `found=false` if
    area < `min_area` (initial 0.5% ROI, field-tuned) or no contour.
  - Thresholds (200/60), gains (kP/kD), and speeds are initial test values;
    final numbers come from field tuning, not fixed by this spec.
- `backend/pinky_control_center/line_follow_service.py` (new):
  async 10 Hz loop per active robot (MVP: `robot_1` only).
  - Pulls `adapter.frame(robot_id)`; skips stale (>2.0 s -> CAMERA_STALLED).
  - P-control: `linear = 0.10 m/s`, `angular = -kP*offset - kD*dOffset`,
    clamp `±0.50 rad/s`, linear clamp `≤0.15` to respect watchdog.
  - Loss policy: 3 consecutive `found=false` -> publish (0,0) + protective-stop
    path + state `LOST`. No auto-resume (watchdog boot-latch semantics).
  - Emits AdapterEvent `line_follow` (offset, polarity, state) for WS + alerts.
- `main.py` wiring: new ops `mission_line_start`, `mission_line_stop`,
  `mission_line_status`. MUST NOT start with `follow_` (diverts to follow
  service). Register in `mission_*` block via `command_service.handlers`.
  All mutations need `request_id` + lease + `X-CSRF-Token` + `Origin`.
- Frontend (minimal MVP): robot card Start/Stop line-follow button, status
  pill (TRACKING white/black / LOST / STALLED), offset bar, camera-grid
  overlay (center line + detected cx). Stop copy says software-only.
- Config: enable `camera.enabled=true` for `robot_1` in ROS field YAML +
  `camera_compressed` topic mapping; `camera.enabled=false` stays for `robot_2`
  until validated. 503 `CAMERA_STALLED` remains the stale signal.

## 4. Data flow
1. Operator holds lease for `robot_1`, clicks Start (mode auto default).
2. `command_service` validates, calls `LineFollowService.start(robot_1, mode)`.
3. Loop: frame -> detect -> `publish_manual_velocity(lin, ang)` ->
   watchdog -> `/cmd_vel`. State snapshot + camera WS update.
4. Stop / loss / lease-expiry / teleop close -> zero velocity + `protective_stop`.

## 5. Error handling / safety
- Watchdog is SOLE `/cmd_vel` publisher; backend never publishes `/cmd_vel`
  directly (uses `TwistStamped` shim, same as lap585 scripts).
- Stale camera, rosbridge offline, stop-latched, lease conflict -> refuse start
  (`CAMERA_STALLED` / `ROSBRIDGE_OFFLINE` / `CONTROL_CONFLICT`), or stop if running.
- `SafetyService` CONFIRMED = latch + odom-agreement + 0.5 s stable; use
  `odom_*` fields for independent checks.
- Field gate: Nav2/AMCL/TF + low-speed run remain NOT_RUN until logged evidence
  in `acceptance-report.md`.

## 6. Testing (TDD)
- RED first: `test_line_detector.py` with synthetic images (white bar left/
  center/right, black bar left/center/right, no-line, noisy floor) asserting
  offset sign + polarity + loss.
- Service tests: mock adapter frame source -> loss-after-3 -> zero published +
  LOST event; `follow_` prefix rejection; lease enforcement.
- Full backend `pytest`, frontend `vitest` for button/status, mock acceptance
  smoke. Real-robot run recorded as field gate, never marked PASS by default.

## 7. Non-goals / follow-ups
- No robot-side OpenCV node in MVP (option B deferred; migrate hot loop to
  Pinky later if rosbridge latency ~100-200 ms proves too slow).
- SLAM, accessory devices, `robot_2` line-follow, recording sync excluded.
- Perspective warp, PID auto-tune, intersection handling are follow-ups.

## 8. Alternatives considered
- A (chosen) backend service: control-platform-first as requested, testable in
  mock, respects watchdog. Con: compressed-image latency.
- B robot-side node: lowest latency, robust; Con: robot deploy + lifecycle now.
- C USB-camera spike: fastest threshold tuning; Con: throwaway, no integration.
  Path: A now, B later (A-then-B).
