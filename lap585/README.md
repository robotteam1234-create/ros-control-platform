# lap585 mapping stages (vendored)

4-stage autonomous mapping for Pinky Pro: wall-follow, frontier
explore, wall-fill, return-home. Run from this directory on the PC
(see `run_lap_real.sh`); the control platform supervises stages via
the mapping missions API (`docs/superpowers/specs/2026-09-16-lap585-mapping-design.md`).

Safety: `pinky_wallfollow.py` publishes through a watchdog shim
(`_WatchdogPublisher` → `/control/manual_velocity`, deadman 0.35 s,
clamp 0.15 m/s) instead of raw `/cmd_vel`. Never bypass it.
Stages 2-4 need `slam_toolbox` + `nav2_map_server` on the PC
(source build; no sudo required) plus Nav2 and a `/map` source.
