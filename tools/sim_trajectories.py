#!/usr/bin/env python3
"""Author the scripted arm trajectories the simulation widget plays back.

The widget used to let visitors shove the arm around with the mouse, which
showed off the physics but never showed a HiveBoard task being done. Instead
each module now gets a fixed trajectory: approach, grasp, actuate, release,
retreat. They are solved here rather than in the browser so that playback is
deterministic -- the same motion every visit, on every machine -- and so a
trajectory that misses its target fails this build instead of shipping.

Each task is authored in task space (where the fingertips go, which way they
point, how far apart they are), sampled at RATE Hz, run through damped
least-squares IK against the `tcp` site, and then *simulated* end to end with
the same controller the widget uses. The simulation is the acceptance test: if
the ball valve does not end up turned, build() raises.

Imported by tools/build-sim-assets.py; not run directly.
"""
import json
import math
from pathlib import Path

import numpy as np

RATE = 50                     # trajectory samples per second
IK_ITERS = 200
IK_DAMPING = 1e-3

# Gripper commands live in tools/build-sim-assets.py's ROBOTS table, in
# whatever units that gripper's actuator takes -- metres of finger travel for
# the FR3's parallel jaws, radians for the hinged jaws on Spot and the SO-101.
# GRASP is deliberately far past whatever we are picking up: these are position
# actuators, so the squeeze is kp times the *unreached* travel. Commanding an
# object's own half-width leaves no error and no grip -- that is why an early
# version closed on the lamp and then slipped straight off it.

# Metres the lamp rises per radian of unscrewing. build-sim-assets.py writes
# the matching equality into the scene. Coarse, so that one wrist's worth of
# rotation lifts the bulb out of its socket rather than merely loosening it.
LAMP_PITCH = 0.0082

DOWN = np.array([0.0, 0.0, -1.0])   # default approach: straight down


def approach_for(cfg, point, base):
    """Which way this arm comes at a point on the board.

    Straight into the board's face, whichever way that faces: down onto a panel
    lying on a bench, horizontally into one stood upright on a post. `tilt`
    then leans the approach outward, away from the base -- the SO-101 is a
    desk-sized thing looking across at its board and cannot point a gripper
    straight at the bench out at arm's length.
    """
    into = -unit(cfg.get("board_normal", (0.0, 0.0, 1.0)))
    tilt = cfg.get("tilt", 0.0)
    if not tilt:
        return into
    radial = np.array([point[0] - base[0], point[1] - base[1], 0.0])
    norm = float(np.linalg.norm(radial))
    if norm < 1e-6:
        return into
    return unit(into + tilt * radial / norm)


def plane_basis(axis, hint):
    """Two unit vectors spanning the plane a joint turns in.

    The tasks used to be written against world Z, which quietly assumed the
    board was lying flat. Stood upright on its post for Spot, every hinge on it
    turns about a horizontal axis instead, so the geometry is built from each
    joint's own axis and nothing about the world.
    """
    axis = unit(axis)
    u = np.asarray(hint, float)
    u = u - axis * (u @ axis)
    if np.linalg.norm(u) < 1e-6:
        fallback = [0.0, 0.0, 1.0] if abs(axis[2]) < 0.9 else [1.0, 0.0, 0.0]
        u = np.cross(axis, fallback)
    u = unit(u)
    return u, np.cross(axis, u)


def board_out(cfg):
    """The direction that stands off the board's face -- 'up', whichever way."""
    return unit(cfg.get("board_normal", (0.0, 0.0, 1.0)))


def board_spin_for(model, data, cfg, module):
    """How far to turn the board so `module` comes round to the robot's side."""
    import mujoco

    board = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "hiveboard")
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{module}_RevoluteJoint")
    if board < 0 or jid < 0:
        return 0.0

    normal = unit(cfg.get("board_normal", (0.0, 0.0, 1.0)))
    centre = np.array(data.xpos[board])
    flatten = lambda v: v - normal * (v @ normal)

    here = flatten(np.array(data.xanchor[jid]) - centre)
    there = flatten(np.array(data.xpos[1]) - centre)
    if np.linalg.norm(here) < 1e-6 or np.linalg.norm(there) < 1e-6:
        return 0.0
    here, there = unit(here), unit(there)
    return float(math.atan2(np.cross(here, there) @ normal, here @ there))


def robot_base(model, data):
    """World position of the robot's root body, for aiming the approach."""
    return np.array(data.xpos[1])

# Every robot in the scene is position-controlled: the command is the joint
# angle. The servo gains live in each robot's own MJCF, so this file and the
# widget both just write the trajectory sample and let MuJoCo close the loop.


# ═══════════════════════════════════════════════════════════════════════════
#  Task-space path
# ═══════════════════════════════════════════════════════════════════════════
def unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def smoothstep(s):
    """Ease in and out, so the arm starts and stops without a velocity jump."""
    return s * s * (3.0 - 2.0 * s)


def frame(approach, finger):
    """Hand orientation from an approach axis and a finger-opening direction.

    The Panda hand reaches along its own +Z and its fingers slide along +/-Y,
    so those two vectors pin the pose down completely. `finger` only has to be
    roughly right -- the component along `approach` is projected out.
    """
    z = unit(approach)
    y = np.asarray(finger, float)
    y = unit(y - z * (y @ z))
    return np.column_stack([np.cross(y, z), y, z])


def lerp_dir(a, b, s):
    """Interpolate two directions through the shorter arc between them.

    Two directions do not record which way round you meant to get from one to
    the other, so this cannot express more than half a turn: asking it for the
    lamp's 5.2 rad unscrew quietly produced 1.08 rad the other way. Anything
    that really turns belongs in an `arc` key, which is given its angle.
    """
    a, b = unit(a), unit(b)
    dot = float(np.clip(a @ b, -1.0, 1.0))
    if dot > 0.9995:
        return unit(a + s * (b - a))
    theta = math.acos(dot) * s
    perp = unit(b - a * dot)
    return a * math.cos(theta) + perp * math.sin(theta)


def rotate(axis, angle):
    """Rodrigues rotation matrix about a unit axis."""
    k = unit(axis)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * (K @ K)


def sample_path(keys):
    """Expand authored keyframes into a per-tick task-space path.

    A key is a dict with `pos`, `finger`, `grip` and the `secs` taken to reach
    it from the previous key. `arc` instead sweeps the fingertips around an
    axis, which is what turning a valve lever actually is. The first key is the
    starting pose and its `secs` is ignored.
    """
    out = []
    prev = keys[0]
    out.append((np.asarray(prev["pos"], float), unit(prev["finger"]),
                unit(prev.get("approach", DOWN)), float(prev["grip"]), False))

    for key in keys[1:]:
        ticks = max(int(round(key["secs"] * RATE)), 1)
        was = unit(prev.get("approach", DOWN))
        wants = unit(key.get("approach", was))
        for i in range(1, ticks + 1):
            s = smoothstep(i / ticks)
            grip = prev["grip"] + s * (key["grip"] - prev["grip"])
            if "arc" in key:
                # Carry the hand around the joint the module actually turns on.
                # A straight-line shove works until the contact normal lines up
                # with the hinge and the moment arm vanishes -- which is exactly
                # where the circuit breaker used to stall, at 0.32 of its 0.52.
                arc = key["arc"]
                anchor = np.asarray(arc["anchor"], float)
                axis = unit(arc["axis"])
                rot = rotate(axis, s * arc["angle"])
                pos = anchor + rot @ (np.asarray(prev["pos"], float) - anchor)
                pos = pos + axis * (arc.get("rise", 0.0) * s)   # a thread climbs
                finger = rot @ unit(prev["finger"])
                # Swinging the approach round with the arc keeps the hand square
                # to what it is turning, but it also tilts the wrist by the full
                # arc angle -- which a short arm already leaning hard cannot add
                # to. Those arcs keep the approach they started with.
                approach = (unit(prev.get("approach", DOWN)) if arc.get("hold_approach")
                            else unit(rot @ unit(prev.get("approach", DOWN))))
            else:
                pos = np.asarray(prev["pos"], float) * (1 - s) + np.asarray(key["pos"], float) * s
                finger = lerp_dir(prev["finger"], key["finger"], s)
                # Swing the approach across the move as well. Snapping it to the
                # new key's direction on the first tick asks the arm to hold the
                # end orientation while still standing at the start position --
                # a pose that is often nowhere near reachable.
                approach = lerp_dir(was, wants, s)
            out.append((pos, unit(finger), approach, grip, not key.get("transit")))

        # Carry the endpoint forward as the next segment's start.
        prev = dict(key, pos=out[-1][0], finger=out[-1][1], approach=approach)

    return out


# ═══════════════════════════════════════════════════════════════════════════
#  Inverse kinematics
# ═══════════════════════════════════════════════════════════════════════════
def rotation_error(target, current):
    """Axis-angle error taking `current` onto `target`, in world axes."""
    err = target @ current.T
    return np.array([err[2, 1] - err[1, 2],
                     err[0, 2] - err[2, 0],
                     err[1, 0] - err[0, 1]]) * 0.5


def solve_ik(model, data, site, path, cfg):
    """Track a task-space path with damped least squares, warm-started.

    A nullspace term pulls the arm back toward its rest posture. With the hand
    pointing straight down joints 5 and 7 both spin the tool about the vertical,
    so the solver is free to split a wrist roll between them -- and it did,
    driving the shoulder into its stop a third of the way through the lamp's
    unscrew. Biasing everything except joint 7 toward home leaves the roll where
    it belongs.
    """
    import mujoco

    n = len(cfg["arm"])
    lo, hi = model.jnt_range[:n].T
    rest = np.array(cfg["home"], float)
    # Bias every joint toward home except the last, which carries the tool roll.
    weight = np.ones(n)
    weight[-1] = 0.0
    # Fewer than six joints cannot hit an arbitrary orientation, so the roll
    # about the approach axis is dropped from the target and the arm is asked
    # only to point the right way. Even then the remaining five constraints
    # exactly determine a five-joint arm, so a target off its reachable surface
    # has no solution at all -- position and approach then trade against each
    # other and the fingertips land a centimetre out. Weighting the rotation
    # rows down resolves that trade in favour of getting to the point, and lets
    # the wrist angle be whatever the arm can manage.
    free_roll = n < 6
    rot_weight = cfg.get("rot_weight", 0.15 if free_roll else 1.0)
    q = rest.copy()
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    out = []
    worst = 0.0

    worst_precise = 0.0
    for pos, finger, approach, grip, precise in path:
        target = frame(approach, finger)
        for it in range(IK_ITERS):
            data.qpos[:n] = q
            mujoco.mj_kinematics(model, data)
            mujoco.mj_comPos(model, data)

            current = data.site_xmat[site].reshape(3, 3)
            if free_roll:
                # Swing the tool axis onto the target axis and ask for nothing
                # about the roll. Subtracting the roll component out of a full
                # orientation error instead leaves an inconsistent target that
                # the solver stalls on -- it was worse than not relaxing at all.
                rot = np.cross(current[:, 2], target[:, 2])
            else:
                rot = rotation_error(target, current)
            err = np.concatenate([pos - data.site_xpos[site], rot * rot_weight])
            if np.linalg.norm(err[:3]) < 2e-4 and np.linalg.norm(err[3:]) < 2e-3:
                break

            mujoco.mj_jacSite(model, data, jacp, jacr, site)
            jac = np.vstack([jacp[:, :n], jacr[:, :n] * rot_weight])
            # Damped least squares: near a singularity a plain pseudo-inverse
            # asks for joint speeds the arm cannot produce.
            pinv = jac.T @ np.linalg.inv(jac @ jac.T + IK_DAMPING * np.eye(6))
            # The damped inverse makes (I - pinv J) only an approximate
            # nullspace projector, so the posture term leaks a little into the
            # task and the two never quite settle. Shape the arm early, then
            # let the last iterations converge on the fingertips alone.
            gain = 0.2 if it < IK_ITERS // 2 else 0.0
            null = (np.eye(n) - pinv @ jac) @ (weight * (rest - q) * gain)
            q = np.clip(q + 0.6 * (pinv @ err) + null, lo + 0.02, hi - 0.02)

        reached = float(np.linalg.norm(err[:3]))
        worst = max(worst, reached)
        if precise:
            worst_precise = max(worst_precise, reached)
        out.append((q.copy(), grip))

    # Transit poses may be missed -- the arm just takes a slightly different
    # route through free space. Only the poses that touch something count.
    return out, worst_precise


# ═══════════════════════════════════════════════════════════════════════════
#  Verification
# ═══════════════════════════════════════════════════════════════════════════
def replay(model, samples, watch, cfg, spin=0.0):
    """Run a trajectory through the real controller and report what moved.

    This is the acceptance test: `watch` names the module joint the task is
    supposed to drive, and the caller checks the swing it comes back with. The
    widget steps exactly this loop, so what passes here is what ships.
    """
    import mujoco

    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    turn = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "board_spin")
    if turn >= 0:
        data.qpos[model.jnt_qposadr[turn]] = spin
    joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, watch)
    adr = model.jnt_qposadr[joint]
    n = len(cfg["arm"])
    grip_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR,
                                cfg["grip"]["actuator"])

    steps_per_sample = max(int(round((1.0 / RATE) / model.opt.timestep)), 1)
    seen = []
    for q, grip in samples:
        for _ in range(steps_per_sample):
            mujoco.mj_step1(model, data)
            data.ctrl[:n] = q
            data.ctrl[grip_id] = grip
            mujoco.mj_step2(model, data)
        seen.append(float(data.qpos[adr]))

    return np.array(seen)


# ═══════════════════════════════════════════════════════════════════════════
#  Reading the scene
#
#  The tasks are derived from the compiled model rather than written out as
#  world coordinates, so moving a module to a different cell in
#  build-sim-assets.py re-aims its trajectory instead of breaking it.
# ═══════════════════════════════════════════════════════════════════════════
def joint_frame(model, data, name):
    """World anchor and axis of a joint, in the scene's home pose."""
    import mujoco

    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    return np.array(data.xanchor[jid]), unit(data.xaxis[jid]), jid


def visual_points(model, data, body_name):
    """Every drawn vertex of a body, in world coordinates."""
    import mujoco

    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    chunks = []
    for g in range(model.ngeom):
        if model.geom_bodyid[g] != bid or model.geom_group[g] != 2:
            continue
        mesh = model.geom_dataid[g]
        if mesh < 0:
            continue
        adr, num = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
        verts = model.mesh_vert[adr:adr + num] @ data.geom_xmat[g].reshape(3, 3).T
        chunks.append(verts + data.geom_xpos[g])
    return np.vstack(chunks)


def home_key(model, data, cfg, grip):
    """The arm's rest pose, expressed in task space so a path can start there."""
    import mujoco

    site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "tcp")
    data.qpos[:len(cfg["home"])] = cfg["home"]
    mujoco.mj_kinematics(model, data)
    # Copy, do not alias: site_xmat is a live view into MuJoCo's own memory, so
    # a key holding a slice of it silently rewrites itself the next time
    # anything calls mj_kinematics -- which the next home_key() promptly does.
    rot = np.array(data.site_xmat[site]).reshape(3, 3)
    return {"pos": np.array(data.site_xpos[site]), "finger": np.array(rot[:, 1]),
            "approach": np.array(rot[:, 2]), "grip": grip, "secs": 0.0}


# ═══════════════════════════════════════════════════════════════════════════
#  The three tasks
# ═══════════════════════════════════════════════════════════════════════════
def valve_task(model, data, cfg):
    """Grasp the ball valve's lever and swing it through its quarter turn."""
    OPEN, GRASP, FIST = (cfg["grip"][k] for k in ("open", "grasp", "fist"))
    anchor, axis, jid = joint_frame(model, data, "valve_RevoluteJoint")
    points = visual_points(model, data, "valve_alavanca_pivot")
    out = board_out(cfg)

    # The handle proper, out past the hub, measured in the plane it swings in.
    rel = points - anchor
    along = rel @ axis
    flat = rel - np.outer(along, axis)
    radius = np.linalg.norm(flat, axis=1)
    handle = radius > 0.03
    u, v = plane_basis(axis, flat[radius.argmax()])
    # Halfway along the bar by default. `valve_grasp` slides that grip back down
    # towards the hub, which on a board stood upright is simply lower: Spot came
    # at the lever near its tip, where the claw reads as hovering over the end of
    # it rather than holding it.
    grasp_r = 0.5 * (radius[handle].min() + radius.max()) - cfg.get("valve_grasp", 0.0)
    grasp_off = 0.5 * (along[handle].min() + along[handle].max())
    start = float(np.median(np.arctan2(flat[handle] @ v, flat[handle] @ u)))

    # Stop just shy of the stop -- unless the grip needs the lever pressed into
    # it. Nothing holds this lever shut but its own stop: released short of it,
    # on a board stood upright, gravity swings it back open.
    sweep = float(model.jnt_range[jid][0]) + cfg.get("valve_shy", 0.07)
    clear = cfg.get("clearance", 0.10)
    radial = lambda ang: u * math.cos(ang) + v * math.sin(ang)
    at = lambda ang, clear=0.0: (anchor + radial(ang) * grasp_r
                                 + axis * grasp_off + out * clear)
    across = lambda ang: np.cross(axis, radial(ang))

    lean = approach_for(cfg, anchor, robot_base(model, data))
    return {
        "module": "valve",
        "label": "Turn the ball valve",
        "caption": "Grasp the lever, swing it through its quarter turn, leave it shut.",
        "watch": "valve_RevoluteJoint",
        "goal": sweep,
        # And it stays shut: nothing springs it back.
        "holds": True,
        "keys": [
            dict(home_key(model, data, cfg, OPEN), transit=True),
            {"pos": at(start, clear), "finger": across(start), "approach": lean, "grip": OPEN, "secs": 1.8, "transit": True},
            {"pos": at(start), "finger": across(start), "approach": lean, "grip": OPEN, "secs": 1.0},
            {"pos": at(start), "finger": across(start), "approach": lean, "grip": GRASP, "secs": 0.7},
            {"arc": {"anchor": anchor, "axis": axis, "angle": sweep,
                     "hold_approach": cfg.get("tilt", 0) > 1.0}, "approach": lean,
             "grip": GRASP, "secs": 2.8},
            # Hold it shut for a beat before letting go, so the lever sits at
            # the end of its travel instead of starting back the instant it
            # arrives there.
            {"pos": at(start + sweep), "finger": across(start + sweep),
             "grip": GRASP, "secs": 1.0},
            {"pos": at(start + sweep), "finger": across(start + sweep),
             "grip": OPEN, "secs": 0.6},
            {"pos": at(start + sweep, clear), "finger": across(start + sweep),
             "grip": OPEN, "secs": 1.0},
            dict(home_key(model, data, cfg, OPEN), secs=1.8, transit=True),
            # A beat parked at home, so the shut valve is on screen as a
            # finished result before the loop resets the scene.
            dict(home_key(model, data, cfg, OPEN), secs=1.5, transit=True),
        ],
    }


def valve_push_task(model, data, cfg):
    """Sweep the ball valve's lever round with a closed gripper.

    Some grippers cannot pinch a 23 mm lever at all -- Spot's clamshell jaws
    slid off it and only carried the handle two thirds of the way. Driving it
    from the side with the gripper shut is the strategy the hardware platform
    itself adopted for this module, and it suits a big gripper far better.
    """
    OPEN, GRASP, FIST = (cfg["grip"][k] for k in ("open", "grasp", "fist"))
    anchor, _, jid = joint_frame(model, data, "valve_RevoluteJoint")
    points = visual_points(model, data, "valve_alavanca_pivot")

    rel = points - anchor
    radius = np.hypot(rel[:, 0], rel[:, 1])
    bar = points[radius > 0.03]
    push_r = 0.5 * (radius[radius > 0.03].min() + radius.max())
    push_z = 0.5 * (bar[:, 2].min() + bar[:, 2].max())
    start = float(np.median(np.arctan2(rel[radius > 0.03, 1], rel[radius > 0.03, 0])))
    sweep = float(model.jnt_range[jid][0]) + 0.07
    lean = approach_for(cfg, anchor, robot_base(model, data))

    # Stand the gripper just behind the lever, on the side it is to be pushed
    # from, and let the arc carry it round the hinge.
    behind = start - math.copysign(0.16, sweep)
    at = lambda ang, dz=0.0: (
        anchor + np.array([math.cos(ang), math.sin(ang), 0.0]) * push_r
        + np.array([0.0, 0.0, push_z - anchor[2] + dz]) - lean * cfg.get("standoff", 0.0))
    across = lambda ang: np.cross([0.0, 0.0, 1.0], [math.cos(ang), math.sin(ang), 0.0])

    return {
        "module": "valve",
        "label": "Turn the ball valve",
        "caption": "Sweep the lever through its quarter turn with the gripper shut.",
        "watch": "valve_RevoluteJoint",
        "goal": sweep,
        "holds": True,
        "keys": [
            dict(home_key(model, data, cfg, FIST), transit=True),
            {"pos": at(behind, 0.12), "finger": across(behind), "approach": lean,
             "grip": FIST, "secs": 1.8, "transit": True},
            {"pos": at(behind), "finger": across(behind), "approach": lean,
             "grip": FIST, "secs": 1.2},
            {"arc": {"anchor": anchor, "axis": (0, 0, 1),
                     "angle": sweep - (behind - start), "hold_approach": True},
             "approach": lean, "grip": FIST, "secs": 3.0},
            {"pos": at(start + sweep, 0.12), "finger": across(start + sweep),
             "approach": lean, "grip": FIST, "secs": 1.2, "transit": True},
            dict(home_key(model, data, cfg, FIST), secs=1.8, transit=True),
            dict(home_key(model, data, cfg, FIST), secs=1.5, transit=True),
        ],
    }


def lamp_task(model, data, cfg):
    """Back the bulb out of its socket, hold it clear, then screw it home again."""
    OPEN, GRASP, FIST = (cfg["grip"][k] for k in ("open", "grasp", "fist"))
    anchor, axis, jid = joint_frame(model, data, "lamp_RevoluteJoint")
    points = visual_points(model, data, "lamp_lamp_pivot")
    out = board_out(cfg)

    # Grip the widest point of the glass. The cylindrical neck behind it holds
    # a twist better, but the palm rides 103 mm behind the fingertips, so
    # reaching that far in plants the hand on the bulb and presses it into its
    # socket. At the equator the palm clears the glass by 76 mm.
    rel = points - anchor
    along = rel @ axis
    radius = np.linalg.norm(rel - np.outer(along, axis), axis=1)
    seated = anchor + axis * float(along[radius.argmax()])

    # One wrist's worth of rotation, and no more: the sweep has to fit inside
    # the last joint's travel, which differs by arm. Starting the hand wound
    # right back rather than centred is what buys the full range -- centred, the
    # FR3 ran into its stop a third of the way through. LAMP_PITCH x turn also
    # has to stay under the bulb's own travel limit.
    import mujoco

    wrist = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, cfg["arm"][-1])
    span = float(model.jnt_range[wrist][1] - model.jnt_range[wrist][0])
    turn = min(5.2, 0.88 * span)
    # Where in the wrist's travel the grasp starts. Half the sweep back from
    # centre is the geometric answer, but the wrist angle a given hand
    # orientation lands on is offset by the rest of the arm's posture, and that
    # offset differs per robot -- so how far past centre to wind is a knob.
    a0 = -turn / 2 - cfg.get("lamp_wind", 0.9)
    lift = LAMP_PITCH * turn

    u, v = plane_basis(axis, [1.0, 0.0, 0.0])
    spin = lambda a: u * math.cos(a) + v * math.sin(a)

    # Where the tool frame sits inside the gripper decides whether the widest
    # point of the glass is the right thing to aim it at. Between the FR3's
    # parallel jaws it is: they close either side of the bulb. Spot's frame is
    # out at the tips of a claw that keeps closing well past them, and reaching
    # that far in left the claw hooked over the glass on the way out, carrying
    # the bulb 10 mm back off its seat. It stops short along the thread instead.
    # The arc still turns about `seated`, which is on the axis either way.
    hold = seated + out * cfg.get("lamp_standoff", 0.0)
    approach = hold + out * cfg.get("lamp_approach", cfg.get("clearance", 0.14))   # clear of the glass

    # What "out of the socket" actually means: the bulb's base has to clear the
    # rim of the cup it sits in, measured along the thread. Measured, not
    # guessed, so re-exporting the module re-derives it.
    socket = visual_points(model, data, "lamp_World")
    clearance = float((socket @ axis).max() - (points @ axis).min()) + 0.004

    lean = approach_for(cfg, anchor, robot_base(model, data))
    return {
        "module": "lamp",
        "label": "Change the lamp",
        "caption": "Unscrew the bulb out of its socket, hold it clear, screw it back home.",
        # Watch the travel, not the rotation. Once the bulb bottoms out, its
        # joint limit beats the thread equality, so the last of the hand's turn
        # slips in the grasp and the angle stops meaning anything -- while the
        # travel says exactly what the task claims: out, then back in.
        "watch": "lamp_PrismaticJoint",
        "goal": clearance,
        # Clear of the socket with room to spare, not by a hair. This file runs
        # against the MuJoCo built for Python; the widget runs the WASM build,
        # and the two are not the same version. Every trajectory that clears the
        # rim comfortably here reproduces there -- but the SO-101's, which
        # cleared it by 4%, did not move the bulb at all in the browser, and a
        # task list is only worth anything if what it lists actually happens.
        "tolerance": 1.10,
        # The bulb has to end up seated again, or "screw it back home" is only
        # a claim.
        "returns": True,
        "keys": [
            dict(home_key(model, data, cfg, OPEN), transit=True),
            {"pos": approach, "finger": spin(a0), "approach": lean, "grip": OPEN, "secs": 1.8, "transit": True},
            {"pos": hold, "finger": spin(a0), "approach": lean, "grip": OPEN, "secs": 1.2},
            {"pos": hold, "finger": spin(a0), "approach": lean, "grip": GRASP, "secs": 0.7},
            # Out of the socket, climbing the thread as it turns.
            {"arc": {"anchor": seated, "axis": axis, "angle": turn, "rise": lift,
                     "hold_approach": cfg.get("tilt", 0) > 1.0}, "approach": lean,
             "grip": GRASP, "secs": 3.4},
            {"arc": {"anchor": seated, "axis": axis, "angle": 0.0}, "approach": lean, "grip": GRASP, "secs": 1.0},
            # And back down it -- slower than it came out. Screwing in is the
            # half that has to stay on the thread: every millimetre the bulb
            # lags the hand is axial load, and axial load on a thread is torque
            # resisting the very turn that would relieve it. Driven out in 3.4 s
            # the bulb wedged and the hand slipped, leaving it standing 7 mm
            # proud of its socket. Given time to track, it goes home.
            {"arc": {"anchor": seated, "axis": axis, "angle": -turn, "rise": -lift,
                     "hold_approach": cfg.get("tilt", 0) > 1.0}, "approach": lean,
             "grip": GRASP, "secs": 6.0},
            {"pos": hold, "finger": spin(a0), "approach": lean, "grip": OPEN, "secs": 0.6},
            {"pos": approach, "finger": spin(a0), "approach": lean, "grip": OPEN, "secs": 1.2, "transit": True},
            dict(home_key(model, data, cfg, OPEN), secs=1.8, transit=True),
        ],
    }


def breaker_task(model, data, cfg):
    """Flip the circuit breaker's toggle with a closed gripper."""
    OPEN, GRASP, FIST = (cfg["grip"][k] for k in ("open", "grasp", "fist"))
    anchor, axis, jid = joint_frame(model, data, "breaker_RevoluteJoint")
    points = visual_points(model, data, "breaker_lever_pivot")

    rel = points - anchor
    perp = rel - np.outer(rel @ axis, axis)
    lever = perp[np.linalg.norm(perp, axis=1).argmax()]

    # Which way the toggle's tip travels as the joint opens up, and therefore
    # which way to shove it. Only the sense that presses *into* the board is
    # reachable from in front of it: pushing the other way asks the fingers to
    # get behind the paddle and lift, and they just skid off its face instead.
    out = board_out(cfg)
    swing = unit(np.cross(axis, lever))
    if swing @ out > 0:
        swing = -swing
        throw = float(model.jnt_range[jid][0]) + 0.06
    else:
        throw = float(model.jnt_range[jid][1]) - 0.06

    # Push the middle of the paddle, not the corner the farthest vertex
    # happens to sit on: `tip` is 20 mm off to one side along the hinge, and
    # shoving there just skids the fingers off the edge.
    lean = approach_for(cfg, anchor, robot_base(model, data))
    contact = (anchor + unit(lever) * (np.linalg.norm(lever) - 0.005)
               - lean * cfg.get("standoff", 0.0))
    reach = abs(throw) * np.linalg.norm(lever) + 0.022
    turn = 1.0 if throw > 0 else -1.0

    return {
        "module": "breaker",
        "label": "Flip the circuit breaker",
        "caption": "Close the gripper and sweep the toggle across to its other stop.",
        "watch": "breaker_RevoluteJoint",
        "goal": throw,
        # And it stays thrown. Nothing springs this toggle back any more, so a
        # run that ends with the breaker where it started is a failed run --
        # which is precisely what the module's old return spring was hiding.
        "holds": True,
        "keys": [
            dict(home_key(model, data, cfg, FIST), transit=True),
            {"pos": contact - swing * 0.028 + out * cfg.get("clearance", 0.10), "finger": swing,
             "approach": lean, "grip": FIST, "secs": 1.8, "transit": True},
            {"pos": contact - swing * 0.028, "finger": swing, "approach": lean, "grip": FIST, "secs": 1.1},
            {"pos": contact - swing * 0.004, "finger": swing, "approach": lean, "grip": FIST, "secs": 0.5},
            {"arc": {"anchor": anchor, "axis": axis * turn, "angle": abs(throw),
                     "hold_approach": cfg.get("tilt", 0) > 0.5}, "approach": lean,
             "grip": FIST, "secs": 1.8},
            {"pos": contact + swing * reach + out * cfg.get("clearance", 0.10), "finger": swing,
             "approach": lean, "grip": FIST, "secs": 1.0, "transit": True},
            dict(home_key(model, data, cfg, FIST), secs=1.8, transit=True),
        ],
    }


def valve_for(cfg):
    """Grasp the lever, or shove it, depending on what the gripper can hold."""
    return valve_push_task if cfg.get("strategy") == "push" else valve_task


TASKS = [valve_for, lamp_task, breaker_task]


# ═══════════════════════════════════════════════════════════════════════════
#  Hand edits  (temporary -- see tools/traj_edit.py)
#
#  The tasks above are derived from the compiled scene, which is what keeps
#  them honest, but a derived waypoint is still only a guess at where a
#  particular arm wants its hand. tools/traj_edit.py lets one be dragged in
#  the browser and writes the offset here, so the next build picks it up
#  instead of it living in a note somewhere.
#
#  To drop the whole editor: delete this block, its two callers below,
#  tools/traj_edits.json, tools/traj_edit.py and public/sim/edit.js.
# ═══════════════════════════════════════════════════════════════════════════
EDITS_FILE = Path(__file__).with_name("traj_edits.json")


def load_edits():
    """Every hand edit on file, as {robot: {module: {...}}}."""
    if not EDITS_FILE.exists():
        return {}
    return json.loads(EDITS_FILE.read_text() or "{}")


def edits_for(robot, module):
    return load_edits().get(robot, {}).get(module, {})


def apply_edits(task, edits):
    """Fold one task's hand edits into its authored keyframes, in place.

    Offsets rather than absolute positions: a key that was measured off the
    board's own geometry should still follow the board when a module moves
    cell, and only differ from where the geometry put it by however far it
    was dragged.

    A key can also be switched off, which drops it from the path entirely --
    the segment before it runs on to whatever comes next. That is how a
    standoff or a settling beat gets tested for whether it was needed at all.
    """
    keys = (edits or {}).get("keys", {})
    for index, edit in keys.items():
        key = task["keys"][int(index)]
        if "dpos" in edit and "pos" in key:
            key["pos"] = np.asarray(key["pos"], float) + np.asarray(edit["dpos"], float)
        if "secs" in edit:
            key["secs"] = float(edit["secs"])
        if "grip" in edit:
            key["grip"] = float(edit["grip"])
        if "arc" in key:
            for field in ("angle", "rise"):
                if field in edit:
                    key["arc"][field] = float(edit[field])

    # Last, and from the back, so the indices above still mean what they said.
    # Never the first key: that one is not a step, it is where the arm starts.
    for index in sorted(dropped(edits), reverse=True):
        task["keys"].pop(index)
    return task


def dropped(edits):
    """Which keys are switched off, first key excluded."""
    return [int(i) for i, edit in (edits or {}).get("keys", {}).items()
            if edit.get("off") and int(i) > 0]


# ═══════════════════════════════════════════════════════════════════════════
#  Entry point
# ═══════════════════════════════════════════════════════════════════════════
def attempt(model, data, site, cfg, factory, spin, spin_adr, edits=None):
    """Solve and replay one task at one board angle. Returns a verdict dict."""
    import mujoco

    mujoco.mj_resetDataKeyframe(model, data, 0)
    if spin_adr is not None:
        data.qpos[spin_adr] = spin
    mujoco.mj_forward(model, data)

    task = factory(model, data, cfg)
    # A big arm chasing a brisk path lags it; the pace multiplier buys the
    # heavier robots time to actually be where the path says they are.
    for key in task["keys"][1:]:
        key["secs"] = key["secs"] * cfg.get("pace", 1.0)
    apply_edits(task, edits)

    path = sample_path(task["keys"])
    samples, worst = solve_ik(model, data, site, path, cfg)

    # An arm that could not track the path never really did the task, no matter
    # what the module ended up doing -- a flailing gripper can knock a lever
    # further than a grasp would.
    if worst > 0.005:
        return {"task": task, "ok": False, "worst": worst, "samples": samples, "spin": spin, "left": 0.0,
                "why": f"out of reach: fingertips off by {worst * 1000:.0f} mm"}

    swing = replay(model, samples, task["watch"], cfg, spin)
    reached = swing.max() if task["goal"] > 0 else swing.min()
    ok = abs(reached) >= abs(task["goal"]) * task.get("tolerance", 0.85)

    # A task that puts something back has to actually put it back -- and a task
    # that leaves something set has to leave it set. "Stays put" is a property
    # of the gripper as much as the task, so it is gated only where the robot
    # can honour it, and reported either way.
    if ok and task.get("returns"):
        ok = abs(float(swing[-1])) <= 0.12 * abs(task["goal"])
    # Against the goal rather than the peak: a toggle shoved into its stop
    # bounces a little past it, and ending up exactly where it was asked to go
    # should not read as slipping just because the overshoot was larger.
    if ok and task.get("holds") and cfg.get("clean_release", True):
        ok = abs(float(swing[-1])) >= 0.9 * abs(task["goal"])

    unit_name = "mm" if task["watch"].endswith("PrismaticJoint") else "rad"
    scale = 1000.0 if unit_name == "mm" else 1.0
    ending = (f" back to {swing[-1] * scale:+.2f}" if task.get("returns")
              else f", left at {swing[-1] * scale:+.2f}" if task.get("holds") else "")
    return {
        "task": task, "ok": ok, "worst": worst, "samples": samples, "spin": spin,
        "left": round(float(swing[-1]), 4),
        "why": (f"{len(samples):4d} samples ({len(samples) / RATE:4.1f}s)  "
                f"ik error {worst * 1000:4.1f} mm  "
                f"{task['watch'].split('_', 1)[1]} reached {reached * scale:+.2f} "
                f"of {task['goal'] * scale:+.2f} {unit_name}{ending}"),
    }


def build(scene_path, cfg):
    """Solve, verify and package every task this robot can actually do.

    A task that misses is dropped from the robot's list rather than failing the
    build: the arms differ enough in reach, degrees of freedom and gripper that
    not every one of them can work every module, and pretending otherwise would
    be the lie the widget exists to avoid.

    Where the stand turns, each task is tried both with the board swung round
    to face its module and with it left alone, and whichever verifies is kept.
    Turning it is what lets a short arm reach the far cells at all, but it is
    not free -- it can swing a module that was comfortable out to an awkward
    angle -- so it is offered rather than imposed.
    """
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "tcp")

    turn = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "board_spin")
    spin_adr = model.jnt_qposadr[turn] if turn >= 0 else None

    out = {}
    for factory in TASKS:
        if factory.__name__.endswith("_for"):
            factory = factory(cfg)
        module = factory.__name__.split("_")[0]
        if module in cfg.get("skip", ()):
            continue

        spins = [0.0]
        if spin_adr is not None:
            facing = board_spin_for(model, data, cfg, module)
            spins = [facing, 0.0] if abs(facing) > 1e-3 else [0.0]

        best = None
        for spin in spins:
            try:
                result = attempt(model, data, site, cfg, factory, spin, spin_adr,
                                 edits_for(cfg["name"], module))
            except Exception as err:              # unreachable geometry, bad seed
                result = {"ok": False, "why": f"skipped: {err}", "spin": spin}
            turned = f" [board {math.degrees(spin):+.0f}°]" if spin else ""
            print(f"    {module:8s} {result['why']}{turned}"
                  f"  [{'ok' if result['ok'] else 'MISSED'}]")
            best = result
            if result["ok"]:
                break

        if not best or "samples" not in best:
            continue

        out[module] = package(model, site, cfg, best)

    return out


def package(model, site, cfg, result):
    """One verified task in the form the widget plays back."""
    task, samples = result["task"], result["samples"]
    return {
        "label": task["label"],
        "caption": task["caption"],
        "rate": RATE,
        "watch": task["watch"],
        "goal": task["goal"],
        "left": result["left"],
        "spin": round(result["spin"], 5),
        "qpos": [[round(v, 5) for v in q] for q, _ in samples],
        "grip": [round(g, 5) for _, g in samples],
        # Where the fingertips go, so the widget can draw the path the arm
        # is about to take rather than making the viewer infer it.
        "tcp": [[round(v, 4) for v in p] for p in tcp_path(model, samples, site, cfg)],
    }


def tcp_path(model, samples, site, cfg):
    """Forward-kinematics the fingertip position for every sample."""
    import mujoco

    data = mujoco.MjData(model)
    out = []
    n = len(cfg["arm"])
    for q, _ in samples:
        data.qpos[:n] = q
        mujoco.mj_kinematics(model, data)
        out.append(np.array(data.site_xpos[site]))
    return out


def dump(trajectories, path):
    path.write_text(json.dumps(trajectories, separators=(",", ":")) + "\n")
