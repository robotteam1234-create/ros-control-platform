# lap585 Mapping Absorption Design (supervisory)

Date: 2026-09-16 · Status: approved design, not yet implemented
Spec: absorbs /home/ai/robot/lap585 (4-stage mapping: wall-follow,
frontier explore, wall-fill, return) into control-platform for
navigation + mapping.

## Decision
Approach A — supervisory absorption. lap585 stages stay ROS nodes
running on the PC under backend supervision. Rejected: B full port
(discards 3.6k tested lines), C shell trigger (no lease/stop
integration, unsafe).

## Architecture
lap585 stage processes (PC, domains 12/13) ↔ new MappingService
(lifecycle + tick) ↔ existing /control/command + watchdog/Nav2
velocity paths ↔ new MappingPanel (stage, progress, map health, stop).
Browser never touches ROS. Velocity never touches /cmd_vel directly:
wall-follow already routes via /control/manual_velocity shim
(lap585/pinky_wallfollow.py _WatchdogPublisher, 2026-09-16);
explore uses Nav2 action only.

## Changes
Backend (new): `mapping_service.py` (stage list instead of waypoints,
valid{} transitions validate/start/pause/resume/cancel, tick progress,
stop-on-pause with runner kill + latch), `api/mappings.py` (mirrors
api/missions.py lease/CSRF), `models.py` MappingMission
(stages, stage_index, progress).
Backend (modify): `main.py` registry + mapping_* handlers (mirrors
68-72), storage persist mapping missions (mirrors mission
idempotency, 24h request_id).
Frontend (new/modify): `MappingPanel.tsx` (mirrors MissionPanel:
list/create/start polling waitForCommand+getMapping, lease/mapId
gates), `api.ts` mapping helpers, App wiring.
Deployment: runner scripts for SLAM/Nav2/stages; slam_toolbox +
nav2_map_server source-build (no sudo on PC) is the heavy
prerequisite gating Stages 2-4.
lap585: keep source pristine except safety shims; wallfollow done.

## Data flow
start → validate map active + formation UNPAIRED + lease → launch
stage process → progress via stage heartbeat/status → pause/cancel
kills process + stop latch. Single-robot map moves reuse
navigate_from_map path; formation-coupled work stays on mission_*
path. New ops must NOT start with follow_ (would divert to follow
service).

## Safety
Lease-gated start; stop kills runner + latch, no auto-resume;
watchdog clamps 0.15 m/s / 0.5 rad/s + deadmen 0.35/0.50 s;
lap585 map-diverge detection aborts to PAUSED; stages publish only
via watchdog/Nav2 paths. All mutations need request_id + lease +
X-CSRF-Token + Origin or 422/409/403.

## Rollout
P0 mock runner (fake stages) backend TDD + frontend tests.
P1 supervised Stage 1 wall-follow on robot_1 (safety-ready).
P2 SLAM/Nav2 source-build, then Stages 2-4 supervised.
Evidence in acceptance-report.md; unrun stays NOT_RUN.
