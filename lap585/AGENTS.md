<!-- Parent: ../AGENTS.md -->
<!-- Generated: 2026-09-17 | Updated: 2026-09-17 -->

# lap585 (vendored mapping stack)

## Purpose
Vendored 4-stage autonomous mapping stack for Pinky Pro: **wall-follow → frontier explore → wall-fill → return-home**. Runs from this directory on the PC; the control platform supervises stages via the mapping missions API (design: `../docs/superpowers/specs/2026-09-16-lap585-mapping-design.md`). This is the canonical copy — a staging duplicate (without this README) sits at the outer workspace root.

## Key Files
| File | Description |
|------|-------------|
| `pinky_hybrid.py` | The 4-stage hybrid mapper: lidar-only wall-follow (no Nav2) → frontier exploration → wall-fill → return to start |
| `pinky_wallfollow.py` | Left-hand-rule wall-following mapper (track is one connected wall blob — one lap covers every corridor); publishes only via the `_WatchdogPublisher` TwistStamped shim |
| `pinky_explore.py` | Frontier exploration: classifies occupancy cells, drives via Nav2, saves the finished map |
| `pinky_wallfill.py` | Finds unobserved (-1) cells between facing walls; navigates to vantage points that can see them |
| `pinky_mapcheck.py` | Map-health checker (shared module + CLI `python3 pinky_mapcheck.py`): detects scan-matching drift (stretched cells / doubled walls) |
| `run_lap_real.sh` | Real-robot pipeline (no Gazebo/sim time): driver → SLAM → Nav2 → 4-stage mapping; auto-sources `/opt/ros/jazzy` + workspace overlays |
| `run_lap585.sh` | One-shot Gazebo simulation pipeline (`ROS_DOMAIN_ID` defaults to 42); `trap 'kill 0'` teardown on Ctrl+C |
| `mapper_params_track.yaml` | `slam_toolbox` parameters tuned for the track |
| `nav2_params_real.yaml` / `nav2_params_sim.yaml` | Nav2 + AMCL params for physical robot / simulation |
| `map_260905.world` | Gazebo world of the practice track (16 box walls — dartsim can't do mesh collisions; kept STL-free) |
| `README.md` | One-screen summary of stages, safety, and prerequisites |

## For AI Agents

### Working In This Directory
- **Safety invariant: never publish raw `/cmd_vel`.** All velocity goes through the `_WatchdogPublisher` shim (`pinky_wallfollow.py`) → `TwistStamped` on `/control/manual_velocity` → the watchdog (`../ros/pinky_control_watchdog/`), the sole `/cmd_vel` publisher.
- Stages 2–4 need `slam_toolbox` + `nav2_map_server` on the PC (source build; no sudo on this machine) plus Nav2 and a `/map` source.
- Stage names here (`wall_follow, frontier_explore, wall_fill, return_home`) must stay in sync with backend `MappingService` STAGES and `frontend/src/mappingStages.ts`.
- This is vendored code: prefer minimal, surgical changes; document deviations from upstream in the commit message.

### Testing Requirements
- No test suite of its own. Backend mapping contracts are covered by `../backend/tests/test_mapping.py` and `test_mapping_e2e.py`; `pinky_mapcheck.py` is runnable standalone as a smoke check.

## Dependencies

### External
- ROS 2 Jazzy: `slam_toolbox` (PC source build), Nav2, Gazebo (sim only)
- Stage 1 needs no Nav2 — lidar-only wall follow

### Internal
- Backend `MappingService` (`../backend/pinky_control_center/mapping_service.py`) supervises stages; `MapHealth` UI mirrors `pinky_mapcheck.health()` output shape.
- Watchdog contract: `../ros/pinky_control_watchdog/`.

<!-- MANUAL: -->
