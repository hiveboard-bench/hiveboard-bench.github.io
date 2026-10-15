#!/usr/bin/env python3
"""Assert the gate-valve grasps are real, not jams.

The judge in ``sim_trajectories`` asks only whether the module joint reached its
goal.  A trace that saws back and forth passes on its maximum, and a head that
is merely pressed against a valve reads the same as one that is held.  These two
fixtures ask what the peak cannot answer:

  * did the wheel only ever advance?  A ratchet releases and swings back on
    purpose, so a wheel that goes *backwards* is being scrubbed, not turned.
  * were the jaws outside the valve at the grasp?  The fixed jaw sitting inside
    the valve is the jam that used to read as a pass.

Both are checked against the recorded trajectory, so they run without re-solving.

    .venv/Scripts/python.exe tools/check_grasps.py
"""
import json
import sys
from pathlib import Path

import mujoco
import numpy as np

import sim_trajectories

REPO = Path(__file__).resolve().parent.parent
MODEL = REPO / "public/sim/models/spot.xml"
TRAJ = REPO / "public/sim/models/spot.traj.json"

# The wheel's body per valve -- the thing a grasp is supposed to be holding.
VALVES = {"high-valve": "high-valve_nut", "small-valve": "small-valve_eixo_trans"}

GIVE_BACK = 0.20      # the wheel may surrender at most this much of its goal
FIXED_CLEAR = -0.004  # the fixed jaw must stay outside the valve
FINGER_CLEAR = -0.020 # the closing finger may press in: that is the grasp


def body_of(model, geom):
    return mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY,
                             int(model.geom_bodyid[geom])) or ""


def monotonic_turn(entry, states, adr):
    """How much of the goal the wheel gave back on its way to the peak.

    Delegates to the solver's own measure, which carries a deadband: without
    one, the replay's per-sample jitter sums into hundreds of percent on a small
    goal, and a clean ratchet reads as a saw.
    """

    return sim_trajectories.movement_quality(states[:, adr], entry["goal"])


def jaws_at_grasp(model, data, states, wheel_body):
    """How far the fixed jaw and the closing finger reach into the valve.

    They are not the same question.  The finger is *supposed* to press into the
    object -- that is the grasp -- and the collision meshes are coarse enough to
    overlap a few millimetres at any real contact.  The fixed jaw has no such
    excuse: if it is inside the valve the grasp is a jam, which is exactly what
    used to read as a pass.
    """

    # Bodies of the robot side, taken from the geoms -- body_of() indexes geoms,
    # so handing it body indices silently compares the valve against itself.
    arm = {int(model.geom_bodyid[g]) for g in range(model.ngeom)
           if body_of(model, g).startswith(("arm", "body", "Spot"))}
    # Group 2 is the presentation shell on both sides: it bulges well past the
    # collision mesh, so counting it reports tens of millimetres of overlap
    # wherever the jaws merely touch.
    wheel = [g for g in range(model.ngeom)
             if body_of(model, g) == wheel_body and model.geom_group[g] != 2]
    worst = {"fixed": (1e9, None), "finger": (1e9, None)}
    for sample in range(len(states)):
        data.qpos[:] = states[sample]
        mujoco.mj_forward(model, data)
        for g in wheel:
            for h in range(model.ngeom):
                if model.geom_bodyid[h] not in arm or model.geom_group[h] == 2:
                    continue
                name = body_of(model, h)
                side = "finger" if name == "arm_link_fngr" else "fixed"
                gap = mujoco.mj_geomDistance(model, data, g, h, 0.3, None)
                if gap < worst[side][0]:
                    worst[side] = (float(gap), sample)
    return worst


def main():
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    traj = json.loads(TRAJ.read_text(encoding="utf-8"))
    failed = []
    for task, wheel_body in VALVES.items():
        entry = traj.get(task)
        if not entry or not entry["ok"]:
            print(f"  {task:12} not solved -- nothing to assert")
            continue
        states = np.array(entry["stateQpos"])
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT,
                                f"{task}_RevoluteJoint")
        adr = model.jnt_qposadr[jid]

        give = monotonic_turn(entry, states, adr)
        reach = jaws_at_grasp(model, data, states, wheel_body)["fixed"]

        problems = []
        if give > GIVE_BACK:
            problems.append(f"wheel gives back {give * 100:.0f}% of its goal")
        if reach[0] < FIXED_CLEAR:
            problems.append(f"fixed jaw sits {reach[0] * 1000:.1f} mm inside "
                            f"the valve at sample {reach[1]}")
        if problems:
            failed.append(task)
            print(f"  {task:12} FAIL  " + "; ".join(problems))
        else:
            print(f"  {task:12} ok    gives back {give * 100:.0f}%, "
                  f"fixed jaw {reach[0] * 1000:+.1f} mm clear")

    if failed:
        print(f"\n{len(failed)} grasp(s) not real: {', '.join(failed)}")
        return 1
    print("\nall grasps real")
    return 0


if __name__ == "__main__":
    sys.exit(main())
