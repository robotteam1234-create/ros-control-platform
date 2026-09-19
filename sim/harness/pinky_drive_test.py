#!/usr/bin/env python
"""Headless Isaac Sim physics verification for sim/assets/pinky_pro_stage.usd.

Checks, in order:
  1. REST   — the robot spawns on the ground plane and settles (no fall-through,
     no drift) within 1 s of physics.
  2. DRIVE  — equal velocity targets on both wheels move the robot forward.
  3. SPIN   — opposite velocity targets turn it in place (diff-drive geometry).

Joint DOF state is intentionally NOT read: `SingleArticulation.get_joint_positions()`
returns None in Isaac Sim 5.1 headless even when drives act correctly; motion of
the base is the observable that matters. Run with ~/isaac-sim-5.1.0/python.sh.
"""

import os
import sys
import traceback

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

STAGE = "/home/ai/robot/control-platform/sim/assets/pinky_pro_stage.usd"
PHYSICS_DT = 1 / 120
STEPS = 120  # 1 s of physics per phase

failures = []


def check(name, cond, detail):
    print(f"[{'PASS' if cond else 'FAIL'}] {name}: {detail}", flush=True)
    if not cond:
        failures.append(name)


def run():
    import numpy as np
    from isaacsim.core.api import World
    from isaacsim.core.prims import SingleArticulation
    from isaacsim.core.utils.stage import open_stage
    from isaacsim.core.utils.types import ArticulationAction

    open_stage(STAGE)
    world = World(stage_units_in_meters=1.0, physics_dt=PHYSICS_DT, rendering_dt=1 / 60)
    world.reset()

    art = SingleArticulation(prim_path="/pinky_pro/base_footprint")
    world.reset()
    art.initialize()
    av = art._articulation_view

    def base_pose():
        import math
        poses, quats = av.get_world_poses()
        qw, qz = float(quats[0][3]), float(quats[0][2])
        return float(poses[0][0]), float(poses[0][2]), 2.0 * math.atan2(qz, qw)

    dof_names = list(art.dof_names)
    # drive wheels only — caster_wheel_joint is passive and must not be driven
    wheel_idx = [i for i, n in enumerate(dof_names) if n in ("l_wheel_joint", "r_wheel_joint")]
    check("wheel dofs found", len(wheel_idx) == 2, f"{[dof_names[i] for i in wheel_idx]}")

    # ---- 1. REST ---------------------------------------------------------
    # Isaac contract: re-initialize the articulation after every world.reset()
    world.reset()
    art.initialize()
    for _ in range(STEPS):
        world.step(render=True)
    _, z_rest, _ = base_pose()
    p0, _, _ = base_pose()
    for _ in range(STEPS):
        world.step(render=True)
    px, _, _ = base_pose()
    x_drift = abs(px - p0)
    check("rests on ground", 0.0 <= z_rest <= 0.05, f"base_footprint z={z_rest:.4f} m")
    check("no rest drift", x_drift < 0.02, f"|dx|={x_drift:.4f} m over 1 s")

    # ---- 2. DRIVE ---------------------------------------------------------
    x0, _, _ = base_pose()
    art.apply_action(
        ArticulationAction(joint_velocities=np.array([3.0, 3.0]), joint_indices=np.array(wheel_idx))
    )
    for _ in range(STEPS):
        world.step(render=True)
    px, _, _ = base_pose()
    dx = px - x0
    check("drives forward", dx > 0.02, f"dx={dx:.3f} m after 1 s @3 rad/s both wheels")

    # ---- 3. SPIN ----------------------------------------------------------
    _, _, yaw0 = base_pose()
    art.apply_action(
        ArticulationAction(joint_velocities=np.array([3.0, -3.0]), joint_indices=np.array(wheel_idx))
    )
    for _ in range(STEPS):
        world.step(render=True)
    _, _, yaw1 = base_pose()
    dyaw = abs(yaw1 - yaw0)
    check("spins in place", dyaw > 0.15, f"|dyaw|={dyaw:.2f} rad after 1 s @±3 rad/s")


try:
    run()
except Exception:
    traceback.print_exc()
    failures.append("exception")
finally:
    print(f"\nRESULT: {'ALL PASS' if not failures else 'FAILED: ' + ','.join(failures)}", flush=True)
    # app.close() hangs on this box after physics runs; the kernel reclaims GPU
    # memory on process exit, so hard-exit once results are out.
    sys.stdout.flush()
    # Kit shutdown can hang on this box; results are already printed.
    sys.stdout.flush()
    os._exit(0 if not failures else 1)
