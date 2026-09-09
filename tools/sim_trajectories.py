#!/usr/bin/env python3
import json
import math
from pathlib import Path
import mujoco
import numpy as np

RATE = 50
IK_ITERS = 200
IK_DAMPING = 1e-3
LAMP_PITCH = 0.0082
DOWN = np.array([0.0, 0.0, -1.0])


def approach_for(cfg, point, base):

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

    axis = unit(axis)
    u = np.asarray(hint, float)
    u = u - axis * (u @ axis)
    if np.linalg.norm(u) < 1e-6:
        fallback = [0.0, 0.0, 1.0] if abs(axis[2]) < 0.9 else [1.0, 0.0, 0.0]
        u = np.cross(axis, fallback)
    u = unit(u)
    return u, np.cross(axis, u)


def board_out(cfg):

    return unit(cfg.get("board_normal", (0.0, 0.0, 1.0)))


def board_spin_for(model, data, cfg, module):

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

    return np.array(data.xpos[1])


def unit(v):

    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def smoothstep(s):

    return s * s * (3.0 - 2.0 * s)


def frame(approach, finger):

    z = unit(approach)
    y = np.asarray(finger, float)
    y = y - z * (y @ z)
    if np.linalg.norm(y) < 1e-6:
        fallback = np.array([1.0, 0.0, 0.0]) if abs(z[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        y = fallback - z * (fallback @ z)
    y = unit(y)
    return np.column_stack([np.cross(y, z), y, z])


def lerp_dir(a, b, s):

    a, b = unit(a), unit(b)
    dot = float(np.clip(a @ b, -1.0, 1.0))
    if dot > 0.9995:
        return unit(a + s * (b - a))
    if dot < -0.9995:
        helper = np.array([1.0, 0.0, 0.0]) if abs(a[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        perp = unit(helper - a * (helper @ a))
        theta = math.pi * s
        return a * math.cos(theta) + perp * math.sin(theta)
    theta = math.acos(dot) * s
    perp = unit(b - a * dot)
    return a * math.cos(theta) + perp * math.sin(theta)


def rotate(axis, angle):

    k = unit(axis)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * (K @ K)


def sample_path(keys):

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
                arc = key["arc"]
                anchor = np.asarray(arc["anchor"], float)
                axis = unit(arc["axis"])
                rot = rotate(axis, s * arc["angle"])
                pos = anchor + rot @ (np.asarray(prev["pos"], float) - anchor)
                pos = pos + axis * (arc.get("rise", 0.0) * s)
                finger = rot @ unit(prev["finger"])
                approach = (unit(prev.get("approach", DOWN)) if arc.get("hold_approach")
                            else unit(rot @ unit(prev.get("approach", DOWN))))
            else:
                pos = np.asarray(prev["pos"], float) * (1 - s) + np.asarray(key["pos"], float) * s
                finger = lerp_dir(prev["finger"], key["finger"], s)
                approach = lerp_dir(was, wants, s)
            out.append((pos, unit(finger), approach, grip, not key.get("transit")))

        prev = dict(key, pos=out[-1][0], finger=out[-1][1], approach=approach)

    return out


def rotation_error(target, current):

    err = target @ current.T
    return np.array([err[2, 1] - err[1, 2],
                     err[0, 2] - err[2, 0],
                     err[1, 0] - err[0, 1]]) * 0.5


def solve_ik(model, data, site, path, cfg):

    n = len(cfg["arm"])
    lo, hi = model.jnt_range[:n].T
    rest = np.array(cfg.get("ik_home", cfg["home"]), float)
    weight = np.ones(n)
    weight[-1] = 0.0
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
                rot = np.cross(current[:, 2], target[:, 2])
            else:
                rot = rotation_error(target, current)
            err = np.concatenate([pos - data.site_xpos[site], rot * rot_weight])
            if np.linalg.norm(err[:3]) < 2e-4 and np.linalg.norm(err[3:]) < 2e-3:
                break

            mujoco.mj_jacSite(model, data, jacp, jacr, site)
            jac = np.vstack([jacp[:, :n], jacr[:, :n] * rot_weight])
            pinv = jac.T @ np.linalg.inv(jac @ jac.T + IK_DAMPING * np.eye(6))
            gain = 0.2 if it < IK_ITERS // 2 else 0.0
            null = (np.eye(n) - pinv @ jac) @ (weight * (rest - q) * gain)
            q = np.clip(q + 0.6 * (pinv @ err) + null, lo + 0.02, hi - 0.02)

        reached = float(np.linalg.norm(err[:3]))
        worst = max(worst, reached)
        if precise:
            worst_precise = max(worst_precise, reached)
        out.append((q.copy(), grip))

    return out, worst_precise


def replay(model, samples, watch, cfg, spin=0.0):

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
    states = []
    for q, grip in samples:
        for _ in range(steps_per_sample):
            mujoco.mj_step1(model, data)
            data.ctrl[:n] = q
            data.ctrl[grip_id] = grip
            mujoco.mj_step2(model, data)
        seen.append(float(data.qpos[adr]))
        states.append(data.qpos.copy())

    return np.array(seen), states


def module_geoms(model):
    board = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "hiveboard")
    out = {}
    for g in range(model.ngeom):
        body = model.geom_bodyid[g]
        while body > 0 and body != board:
            body = model.body_parentid[body]
        if body != board:
            continue
        name = model.body(model.geom_bodyid[g]).name or ""
        prefix = name.split("_", 1)[0]
        if prefix and prefix != "panel":
            out.setdefault(prefix, []).append(g)
    return out


def only_module(model, geoms, contact, live):

    model.geom_contype[:] = contact[0]
    model.geom_conaffinity[:] = contact[1]
    for prefix, ids in geoms.items():
        if prefix == live:
            continue
        model.geom_contype[ids] = 0
        model.geom_conaffinity[ids] = 0


def joint_frame(model, data, name):

    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    return np.array(data.xanchor[jid]), unit(data.xaxis[jid]), jid


def visual_points(model, data, body_name):

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

    site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "tcp")
    data.qpos[:len(cfg["home"])] = cfg["home"]
    mujoco.mj_kinematics(model, data)
    rot = np.array(data.site_xmat[site]).reshape(3, 3)
    return {"pos": np.array(data.site_xpos[site]), "finger": np.array(rot[:, 1]),
            "approach": np.array(rot[:, 2]), "grip": grip, "secs": 0.0}


def valve_task(model, data, cfg):

    OPEN, GRASP, FIST = (cfg["grip"][k] for k in ("open", "grasp", "fist"))
    anchor, axis, jid = joint_frame(model, data, "valve_RevoluteJoint")
    points = visual_points(model, data, "valve_alavanca_pivot")
    out = board_out(cfg)

    rel = points - anchor
    along = rel @ axis
    flat = rel - np.outer(along, axis)
    radius = np.linalg.norm(flat, axis=1)
    handle = radius > 0.03
    u, v = plane_basis(axis, flat[radius.argmax()])
    grasp_r = 0.5 * (radius[handle].min() + radius.max()) - cfg.get("valve_grasp", 0.0)
    grasp_off = 0.5 * (along[handle].min() + along[handle].max())
    start = float(np.median(np.arctan2(flat[handle] @ v, flat[handle] @ u)))

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
        "holds": True,
        "drive": {"valve_RevoluteJoint": [0.0] * 4 + [sweep] * 6},
        "keys": [
            dict(home_key(model, data, cfg, OPEN), transit=True),
            {"pos": at(start, clear), "finger": across(start), "approach": lean, "grip": OPEN, "secs": 1.8, "transit": True},
            {"pos": at(start), "finger": across(start), "approach": lean, "grip": OPEN, "secs": 1.0},
            {"pos": at(start), "finger": across(start), "approach": lean, "grip": GRASP, "secs": 0.7},
            {"arc": {"anchor": anchor, "axis": axis, "angle": sweep,
                     "hold_approach": cfg.get("tilt", 0) > 1.0}, "approach": lean,
             "grip": GRASP, "secs": 2.8},
            {"pos": at(start + sweep), "finger": across(start + sweep),
             "grip": GRASP, "secs": 1.0},
            {"pos": at(start + sweep), "finger": across(start + sweep),
             "grip": OPEN, "secs": 0.6},
            {"pos": at(start + sweep, clear), "finger": across(start + sweep),
             "grip": OPEN, "secs": 1.0},
            dict(home_key(model, data, cfg, OPEN), secs=1.8, transit=True),
            dict(home_key(model, data, cfg, OPEN), secs=1.5, transit=True),
        ],
    }


def valve_push_task(model, data, cfg):

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
        "drive": {"valve_RevoluteJoint": [0.0] * 3 + [sweep] * 4},
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

    OPEN, GRASP, FIST = (cfg["grip"][k] for k in ("open", "grasp", "fist"))
    anchor, axis, jid = joint_frame(model, data, "lamp_RevoluteJoint")
    points = visual_points(model, data, "lamp_lamp_pivot")
    out = board_out(cfg)

    rel = points - anchor
    along = rel @ axis
    radius = np.linalg.norm(rel - np.outer(along, axis), axis=1)
    seated = anchor + axis * float(along[radius.argmax()])

    wrist = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, cfg["arm"][-1])
    span = float(model.jnt_range[wrist][1] - model.jnt_range[wrist][0])
    turn = min(5.2, 0.88 * span)
    a0 = -turn / 2 - cfg.get("lamp_wind", 0.9)
    lift = LAMP_PITCH * turn

    u, v = plane_basis(axis, [1.0, 0.0, 0.0])
    spin = lambda a: u * math.cos(a) + v * math.sin(a)

    hold = seated + out * cfg.get("lamp_standoff", 0.0)
    grasp_radius = cfg.get("lamp_grasp_radius", 0.0)
    if grasp_radius:
        toward = robot_base(model, data) - anchor
        toward = toward - out * (toward @ out)
        hold = hold + unit(toward) * grasp_radius
    approach = hold + out * cfg.get("lamp_approach", cfg.get("clearance", 0.14))

    lean = approach_for(cfg, anchor, robot_base(model, data))
    if cfg.get("lamp_demo"):
        return {
            "module": "lamp",
            "label": "Change the lamp",
            "caption": "Leave the lamp, turn out and back without interaction.",
            "watch": "lamp_PrismaticJoint",
            "goal": 0.0,
            "demo": True,
            "tolerance": 0.0,
            "ik_tolerance": 1.0,
            "drive": {
                "lamp_RevoluteJoint": [0.0, 0.0, 0.0, turn, 0.0, 0.0, 0.0],
                "lamp_PrismaticJoint": [0.0, 0.0, 0.0, lift, 0.0, 0.0, 0.0],
            },
            "keys": [
                dict(home_key(model, data, cfg, OPEN), transit=True),
                {"pos": approach, "finger": spin(a0), "approach": lean,
                 "grip": OPEN, "secs": 1.5, "transit": True},
                {"pos": hold, "finger": spin(a0), "approach": lean,
                 "grip": OPEN, "secs": 1.0},
                {"arc": {"anchor": seated, "axis": axis, "angle": turn,
                          "rise": lift}, "approach": lean,
                 "grip": OPEN, "secs": 3.4},
                {"arc": {"anchor": seated, "axis": axis, "angle": -turn,
                          "rise": -lift}, "approach": lean,
                 "grip": OPEN, "secs": 3.4},
                {"pos": approach, "finger": spin(a0), "approach": lean,
                 "grip": OPEN, "secs": 1.2},
                dict(home_key(model, data, cfg, OPEN), secs=1.5, transit=True),
            ],
        }

    socket = visual_points(model, data, "lamp_World")
    clearance = float((socket @ axis).max() - (points @ axis).min()) + 0.004

    return {
        "module": "lamp",
        "label": "Change the lamp",
        "caption": "Unscrew the bulb out of its socket, hold it clear, screw it back home.",
        "watch": "lamp_PrismaticJoint",
        "goal": clearance,
        "tolerance": cfg.get("lamp_tolerance", 1.10),
        "ik_tolerance": cfg.get("lamp_ik_tolerance", 0.005),
        "measure_from": 1,
        "returns": True,
        "drive": {
            "lamp_RevoluteJoint": [0.0] * 4 + [turn, turn] + [0.0] * 4,
            "lamp_PrismaticJoint": [0.0] * 4 + [lift, lift] + [0.0] * 4,
        },
        "keys": [
            dict(home_key(model, data, cfg, OPEN), transit=True),
            {"pos": approach, "finger": spin(a0), "approach": lean, "grip": OPEN, "secs": 1.8, "transit": True},
            {"pos": hold, "finger": spin(a0), "approach": lean, "grip": OPEN, "secs": 1.2},
            {"pos": hold, "finger": spin(a0), "approach": lean, "grip": GRASP, "secs": 0.7},
            {"arc": {"anchor": seated, "axis": axis, "angle": turn, "rise": lift,
                     "hold_approach": cfg.get("tilt", 0) > 1.0}, "approach": lean,
             "grip": GRASP, "secs": 3.4},
            {"arc": {"anchor": seated, "axis": axis, "angle": 0.0}, "approach": lean, "grip": GRASP, "secs": 1.0},
            {"arc": {"anchor": seated, "axis": axis, "angle": -turn, "rise": -lift,
                     "hold_approach": cfg.get("tilt", 0) > 1.0}, "approach": lean,
             "grip": GRASP, "secs": 6.0},
            {"pos": hold, "finger": spin(a0), "approach": lean, "grip": OPEN, "secs": 0.6},
            {"pos": approach, "finger": spin(a0), "approach": lean, "grip": OPEN, "secs": 1.2, "transit": True},
            dict(home_key(model, data, cfg, OPEN), secs=1.8, transit=True),
        ],
    }


def breaker_task(model, data, cfg):

    OPEN, GRASP, FIST = (cfg["grip"][k] for k in ("open", "grasp", "fist"))
    anchor, axis, jid = joint_frame(model, data, "breaker_RevoluteJoint")
    points = visual_points(model, data, "breaker_lever_pivot")

    rel = points - anchor
    perp = rel - np.outer(rel @ axis, axis)
    lever = perp[np.linalg.norm(perp, axis=1).argmax()]

    out = board_out(cfg)
    swing = unit(np.cross(axis, lever))
    if swing @ out > 0:
        swing = -swing
        throw = float(model.jnt_range[jid][0]) + 0.06
    else:
        throw = float(model.jnt_range[jid][1]) - 0.06

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
        "holds": True,
        "drive": {"breaker_RevoluteJoint": [0.0] * 4 + [throw] * 3},
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


def wrist_turn(model, cfg, cap):
    wrist = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, cfg["arm"][-1])
    span = float(model.jnt_range[wrist][1] - model.jnt_range[wrist][0])
    return min(cap, 0.88 * span)


def turn_axis(model, data, cfg, joint):

    anchor, axis, jid = joint_frame(model, data, joint)
    return (anchor, axis if axis @ board_out(cfg) >= 0 else -axis, jid)


def grip_band(points, axis, reach, depth=0.025):

    band = points[(points @ axis) >= reach - depth]
    return band if len(band) >= 8 else points


def pinch_phase(points, anchor, axis, u, v, across):

    flat = points - anchor - np.outer((points - anchor) @ axis, axis)
    plane = np.column_stack([flat @ u, flat @ v])
    plane = plane - 0.5 * (plane.min(axis=0) + plane.max(axis=0))
    angles = np.linspace(0.0, math.pi, 181, endpoint=False)
    reach = np.array([np.abs(plane @ [math.cos(a), math.sin(a)]).max()
                      for a in angles])
    return float(angles[reach.argmax() if across == "wide" else reach.argmin()])


def nearest_phase(phase, want, symmetry):

    step = 2.0 * math.pi / max(symmetry, 1)
    return phase + step * round((want - phase) / step)


def twist_task(model, data, cfg, spec):

    OPEN, GRASP = cfg["grip"]["open"], cfg["grip"][spec.get("pinch", "grasp")]
    module = spec["module"]
    anchor, axis, _ = turn_axis(model, data, cfg, f"{module}_{spec['turn']}")
    out = board_out(cfg)
    points = visual_points(model, data, spec["body"])

    turn = spec["sign"] * wrist_turn(model, cfg, spec["cap"])
    lift = spec.get("pitch", 0.0) * abs(turn)
    reach = float((points @ axis).max()) - spec["inset"]
    seat = anchor + axis * (reach - anchor @ axis)

    u, v = plane_basis(axis, [1.0, 0.0, 0.0])
    face = lambda a: u * math.cos(a) + v * math.sin(a)
    a0 = nearest_phase(pinch_phase(grip_band(points, axis, reach),
                                   anchor, axis, u, v,
                                   spec.get("across", "narrow")),
                       -turn / 2, spec.get("symmetry", 6))

    lean = approach_for(cfg, anchor, robot_base(model, data))
    clear = out * cfg.get("clearance", 0.10)
    riding = spec["watch"] in ("RiseJoint", "PrismaticJoint")

    drive = {f"{module}_{spec['turn']}": [0.0] * 4 + [turn] * 5}
    if riding:
        drive[f"{module}_{spec['watch']}"] = [0.0] * 4 + [lift] * 5

    return {
        "module": module,
        "label": spec["label"],
        "caption": spec["caption"],
        "watch": f"{module}_{spec['watch']}",
        "goal": spec.get("goal", lift if riding else turn),
        "holds": True,
        "drive": drive,
        "tolerance": spec.get("tolerance", 0.85),
        "keys": [
            dict(home_key(model, data, cfg, OPEN), transit=True),
            {"pos": seat + clear, "finger": face(a0), "approach": lean,
             "grip": OPEN, "secs": 1.8, "transit": True},
            {"pos": seat, "finger": face(a0), "approach": lean,
             "grip": OPEN, "secs": 1.2},
            {"pos": seat, "finger": face(a0), "approach": lean,
             "grip": GRASP, "secs": 0.8},
            {"arc": {"anchor": seat, "axis": axis, "angle": turn, "rise": lift,
                     "hold_approach": cfg.get("tilt", 0) > 1.0},
             "approach": lean, "grip": GRASP, "secs": spec.get("turn_secs", 3.6)},
            {"arc": {"anchor": seat, "axis": axis, "angle": 0.0},
             "approach": lean, "grip": GRASP, "secs": 0.8},
            {"pos": seat + axis * lift, "finger": face(a0 + turn),
             "approach": lean, "grip": OPEN, "secs": 0.7},
            {"pos": seat + axis * lift + clear, "finger": face(a0 + turn),
             "approach": lean, "grip": OPEN, "secs": 1.2, "transit": True},
            dict(home_key(model, data, cfg, OPEN), secs=1.8, transit=True),
        ],
    }


def key_task(model, data, cfg, spec):

    OPEN, GRASP = cfg["grip"]["open"], cfg["grip"][spec.get("pinch", "grasp")]
    module = spec["module"]
    anchor, axis, _ = turn_axis(model, data, cfg, f"{module}_RevoluteJoint")
    out = board_out(cfg)
    points = visual_points(model, data, spec["body"])

    turn = spec["sign"] * wrist_turn(model, cfg, spec["cap"])
    draw = spec["draw"]
    reach = float((points @ axis).max()) - spec["inset"]
    seat = anchor + axis * (reach - anchor @ axis)

    u, v = plane_basis(axis, [1.0, 0.0, 0.0])
    face = lambda a: u * math.cos(a) + v * math.sin(a)
    a0 = nearest_phase(pinch_phase(grip_band(points, axis, reach),
                                   anchor, axis, u, v,
                                   spec.get("across", "narrow")),
                       -turn / 2, spec.get("symmetry", 2))
    turned = face(a0 + turn)

    lean = approach_for(cfg, anchor, robot_base(model, data))
    clear = out * cfg.get("clearance", 0.10)
    hold = lambda pos, finger, grip, secs, **extra: dict(
        {"pos": pos, "finger": finger, "approach": lean, "grip": grip,
         "secs": secs}, **extra)
    swing = lambda angle: {
        "arc": {"anchor": seat, "axis": axis, "angle": angle,
                "hold_approach": cfg.get("tilt", 0) > 1.0},
        "approach": lean, "grip": GRASP, "secs": spec.get("turn_secs", 3.0)}

    return {
        "module": module,
        "label": spec["label"],
        "caption": spec["caption"],
        "watch": f"{module}_RevoluteJoint",
        "goal": spec.get("goal", turn),
        "returns": True,
        "drive": {
            f"{module}_RevoluteJoint":
                [0.0] * 4 + [turn] * 4 + [0.0] * 4,
            f"{module}_PrismaticJoint":
                [0.0] * 5 + [draw, draw] + [0.0] * 5,
        },
        "tolerance": spec.get("tolerance", 0.85),
        "keys": [
            dict(home_key(model, data, cfg, OPEN), transit=True),
            hold(seat + clear, face(a0), OPEN, 1.8, transit=True),
            hold(seat, face(a0), OPEN, 1.2),
            hold(seat, face(a0), GRASP, 0.8),
            swing(turn),
            hold(seat + axis * draw, turned, GRASP, 1.8),
            hold(seat + axis * draw, turned, GRASP, 0.6),
            hold(seat, turned, GRASP, 1.8),
            swing(-turn),
            hold(seat, face(a0), OPEN, 0.7),
            hold(seat + clear, face(a0), OPEN, 1.2, transit=True),
            dict(home_key(model, data, cfg, OPEN), secs=1.8, transit=True),
        ],
    }


def rim_task(model, data, cfg, spec):

    OPEN, GRASP = cfg["grip"]["open"], cfg["grip"][spec.get("pinch", "grasp")]
    module = spec["module"]
    anchor, axis, _ = turn_axis(model, data, cfg, f"{module}_{spec['turn']}")
    out = board_out(cfg)
    points = visual_points(model, data, spec["body"])

    rel = points - anchor
    along = rel @ axis
    flat = rel - np.outer(along, axis)
    radius = np.linalg.norm(flat, axis=1)
    rim = radius > spec.get("rim", 0.75) * radius.max()
    grasp_r = float(radius[rim].mean()) - spec.get("bite", 0.004)
    grasp_off = float(0.5 * (along[rim].min() + along[rim].max()))

    u, v = plane_basis(axis, flat[radius.argmax()])
    start = float(np.median(np.arctan2(flat[rim] @ v, flat[rim] @ u)))
    turn = spec["sign"] * wrist_turn(model, cfg, spec["cap"])

    radial = lambda a: u * math.cos(a) + v * math.sin(a)
    at = lambda a, gap=0.0: (anchor + radial(a) * grasp_r
                             + axis * grasp_off + out * gap)
    across = lambda a: np.cross(axis, radial(a))

    lean = approach_for(cfg, anchor, robot_base(model, data))
    clear = cfg.get("clearance", 0.10)

    return {
        "module": module,
        "label": spec["label"],
        "caption": spec["caption"],
        "watch": f"{module}_{spec['watch']}",
        "goal": turn,
        "holds": True,
        "drive": {f"{module}_{spec['turn']}": [0.0] * 4 + [turn] * 5},
        "tolerance": spec.get("tolerance", 0.85),
        "keys": [
            dict(home_key(model, data, cfg, OPEN), transit=True),
            {"pos": at(start, clear), "finger": across(start), "approach": lean,
             "grip": OPEN, "secs": 1.8, "transit": True},
            {"pos": at(start), "finger": across(start), "approach": lean,
             "grip": OPEN, "secs": 1.1},
            {"pos": at(start), "finger": across(start), "approach": lean,
             "grip": GRASP, "secs": 0.8},
            {"arc": {"anchor": anchor, "axis": axis, "angle": turn,
                     "hold_approach": cfg.get("tilt", 0) > 1.0},
             "approach": lean, "grip": GRASP, "secs": 3.4},
            {"pos": at(start + turn), "finger": across(start + turn),
             "grip": GRASP, "secs": 0.8},
            {"pos": at(start + turn), "finger": across(start + turn),
             "grip": OPEN, "secs": 0.6},
            {"pos": at(start + turn, clear), "finger": across(start + turn),
             "grip": OPEN, "secs": 1.2, "transit": True},
            dict(home_key(model, data, cfg, OPEN), secs=1.8, transit=True),
        ],
    }


def module_top(model, data, spec):

    module = spec["module"]
    top = -1e9
    for bid in range(model.nbody):
        name = model.body(bid).name or ""
        if not name.startswith(f"{module}_") or name == spec["body"]:
            continue
        try:
            here = visual_points(model, data, name)
        except ValueError:
            continue
        _, axis, _ = joint_frame(model, data, f"{module}_{spec['watch']}")
        top = max(top, float((here @ unit(axis)).max()))
    return top


def narrow_side(points, axis, fallback):

    flat = points - np.outer(points @ axis, axis)
    span = flat.max(axis=0) - flat.min(axis=0)
    span = span - axis * (span @ axis)
    if float(np.linalg.norm(span)) < 1e-6:
        return unit(fallback)
    keep = np.argsort(np.abs(span))
    if abs(span[keep[-1]]) - abs(span[keep[-2]]) < 1e-3:
        return unit(fallback)
    axes = np.eye(3)
    return unit(axes[keep[-2]] - axis * (axes[keep[-2]] @ axis))


def draw_task(model, data, cfg, spec):

    OPEN, GRASP = cfg["grip"]["open"], cfg["grip"][spec.get("pinch", "grasp")]
    module = spec["module"]
    anchor, axis, _ = turn_axis(model, data, cfg, f"{module}_{spec['watch']}")
    out = board_out(cfg)
    points = visual_points(model, data, spec["body"])

    travel = spec["travel"]
    proud = points[(points @ axis) > module_top(model, data, spec) + 1e-4]
    if len(proud) < 4:
        proud = points
    flat = proud - np.outer(proud @ axis, axis)
    seat = (0.5 * (flat.min(axis=0) + flat.max(axis=0))
            + axis * (float((proud @ axis).max()) - spec["inset"]))
    finger = (unit(spec["finger"]) if "finger" in spec
              else narrow_side(proud, axis, np.cross(axis, out)))

    lean = approach_for(cfg, anchor, robot_base(model, data))
    clear = out * cfg.get("clearance", 0.10)
    hold = lambda pos, grip, secs, **extra: dict(
        {"pos": pos, "finger": finger, "approach": lean, "grip": grip,
         "secs": secs}, **extra)

    keys = [
        dict(home_key(model, data, cfg, OPEN), transit=True),
        hold(seat + clear, OPEN, 1.8, transit=True),
        hold(seat, OPEN, 1.2),
        hold(seat, GRASP, 0.8),
        hold(seat + axis * travel, GRASP, spec.get("draw_secs", 2.4)),
    ]
    if spec.get("returns"):
        keys += [hold(seat + axis * travel, GRASP, 0.8),
                 hold(seat, GRASP, 2.6),
                 hold(seat, OPEN, 0.6),
                 hold(seat + clear, OPEN, 1.2, transit=True)]
    else:
        keys += [hold(seat + axis * travel, OPEN, 0.6),
                 hold(seat + axis * travel + clear, OPEN, 1.2, transit=True)]
    keys.append(dict(home_key(model, data, cfg, OPEN), secs=1.8, transit=True))
    pulled = ([0.0] * 4 + [travel, travel] + [0.0] * 4 if spec.get("returns")
              else [0.0] * 4 + [travel] * 4)

    return {
        "module": module,
        "label": spec["label"],
        "caption": spec["caption"],
        "watch": f"{module}_{spec['watch']}",
        "goal": spec.get("goal", travel),
        "drive": {f"{module}_{spec['watch']}": pulled},
        "returns": bool(spec.get("returns")),
        "holds": not spec.get("returns"),
        "measure_from": 1 if spec.get("returns") else 0,
        "tolerance": spec.get("tolerance", 0.85),
        "keys": keys,
    }


def button_task(model, data, cfg, spec):

    OPEN, GRASP, FIST = (cfg["grip"][k] for k in ("open", "grasp", "fist"))
    module = spec["module"]
    out = board_out(cfg)
    hinge, hinge_axis, hid = joint_frame(model, data, f"{module}_RevoluteJoint")
    lid = visual_points(model, data, f"{module}_lid_pivot")
    button = visual_points(model, data, f"{module}_button_pivot")

    swing = float(model.jnt_range[hid][1] - data.qpos[model.jnt_qposadr[hid]])
    swing = math.copysign(min(abs(swing), 1.5) , swing) - spec.get("shy", 0.1)

    edge = lid[((lid - hinge) @ unit(np.cross(hinge_axis, out))).argmax()]
    edge = edge - out * spec.get("bite", 0.006)
    lean = approach_for(cfg, hinge, robot_base(model, data))
    across = unit(hinge_axis)
    clear = out * cfg.get("clearance", 0.10)

    press = button[(button @ out).argmax()] - out * spec.get("standoff", 0.0)
    push = spec["push"]

    open_keys = [
        dict(home_key(model, data, cfg, OPEN), transit=True),
        {"pos": edge + clear, "finger": across, "approach": lean,
         "grip": OPEN, "secs": 1.8, "transit": True},
        {"pos": edge, "finger": across, "approach": lean, "grip": OPEN, "secs": 1.1},
        {"pos": edge, "finger": across, "approach": lean, "grip": GRASP, "secs": 0.7},
        {"arc": {"anchor": hinge, "axis": hinge_axis, "angle": swing,
                 "hold_approach": True}, "approach": lean,
         "grip": GRASP, "secs": 2.4},
        {"arc": {"anchor": hinge, "axis": hinge_axis, "angle": 0.0},
         "approach": lean, "grip": OPEN, "secs": 0.6},
    ]
    press_keys = [
        {"pos": press + clear, "finger": across, "approach": -out,
         "grip": FIST, "secs": 1.4, "transit": True},
        {"pos": press, "finger": across, "approach": -out, "grip": FIST, "secs": 1.1},
        {"pos": press + out * push, "finger": across, "approach": -out,
         "grip": FIST, "secs": 1.0},
        {"pos": press + out * push, "finger": across, "approach": -out,
         "grip": FIST, "secs": 0.5},
        {"pos": press + clear, "finger": across, "approach": -out,
         "grip": FIST, "secs": 1.0, "transit": True},
        dict(home_key(model, data, cfg, FIST), secs=1.8, transit=True),
    ]

    shut = float(data.qpos[model.jnt_qposadr[hid]])
    drive = {
        f"{module}_RevoluteJoint":
            [shut] * 4 + [shut + swing] * (len(open_keys) + len(press_keys) - 4),
        f"{module}_PrismaticJoint":
            [0.0] * (len(open_keys) + 2) + [push, push] + [0.0] * 2,
    }

    return {
        "module": module,
        "label": spec["label"],
        "caption": spec["caption"],
        "watch": f"{module}_PrismaticJoint",
        "goal": push,
        "returns": True,
        "drive": drive,
        "measure_from": len(open_keys) - 1,
        "tolerance": spec.get("tolerance", 0.7),
        "keys": open_keys + press_keys,
    }


def valve_for(cfg):

    return valve_push_task if cfg.get("strategy") == "push" else valve_task


def bind(factory, **spec):

    def made(model, data, cfg):
        return factory(model, data, cfg, spec)
    made.__name__ = spec["module"]
    return made


TASKS = {
    "valve": valve_for,
    "lamp": lamp_task,
    "breaker": breaker_task,

    "high-valve": bind(
        rim_task, module="high-valve", body="high-valve_nut",
        turn="RevoluteJoint", watch="RevoluteJoint", sign=1.0, cap=3.0,
        rim=0.8, bite=0.006,
        label="Open the gate valve",
        caption="Pinch the handwheel rim and wind it round a full turn."),

    "small-valve": bind(
        rim_task, module="small-valve", body="small-valve_eixo_trans",
        turn="RevoluteJoint", watch="RevoluteJoint", sign=1.0, cap=5.2,
        rim=0.8, bite=0.005,
        label="Open the small gate valve",
        caption="Take the little handwheel by its rim and wind it open."),

    "thread-m30": bind(
        twist_task, module="thread-m30", body="thread-m30_Nut",
        turn="RevoluteJoint", watch="RiseJoint", sign=1.0, cap=5.2,
        pitch=0.0035, inset=0.002, symmetry=6,
        label="Run the M30 nut up its thread",
        caption="Grip the hex across its flats and wind it up the stud."),

    "thread-m8": bind(
        twist_task, module="thread-m8", body="thread-m8_nut_pivot",
        turn="RevoluteJoint", watch="PrismaticJoint", sign=1.0, cap=1.05,
        pitch=0.00125, inset=-0.007, symmetry=6, pinch="fist",
        label="Run the M8 nut up its thread",
        caption="Pinch the small hex nut and wind it up the screw."),

    "peg-insertion": bind(
        draw_task, module="peg-insertion", body="peg-insertion_peg",
        watch="PrismaticJoint", travel=0.028, inset=0.0025,
        finger=(0.0, 1.0, 0.0), returns=True, pinch="fist",
        label="Draw and reseat the peg",
        caption="Pull the peg clear of its plate, then feed it back down the hole."),

    "button-cover": bind(
        button_task, module="button-cover", push=-0.009, bite=0.005,
        standoff=0.0, shy=0.12,
        label="Press the hidden button",
        caption="Swing the cover off the button, then press it home."),

    "key-lock": bind(
        key_task, module="key-lock", body="key-lock_key",
        sign=1.0, cap=1.55, draw=0.018, inset=0.011,
        symmetry=2, across="wide",
        label="Work the key",
        caption="Turn the key through the lock, draw it part way out and feed "
                "it back, then turn it home again."),

    "drawer": bind(
        draw_task, module="drawer", body="drawer_drawer",
        watch="PrismaticJoint", travel=0.03, inset=0.006,
        finger=(0.0, 1.0, 0.0),
        label="Pull the drawer open",
        caption="Pinch the drawer front and draw it out of its case."),

}
EDITS_FILE = Path(__file__).with_name("traj_edits.json")
VIEWS_FILE = Path(__file__).with_name("task_views.json")


def views_for(robot):

    if not VIEWS_FILE.exists():
        return {}
    return json.loads(VIEWS_FILE.read_text() or "{}").get(robot, {})

DRAFT_MODULES = []


def load_edits():

    if not EDITS_FILE.exists():
        return {}
    return json.loads(EDITS_FILE.read_text() or "{}")


def edits_for(robot, module):

    return load_edits().get(robot, {}).get(module, {})


def apply_edits(task, edits):

    for i, key in enumerate(task["keys"]):
        key["_id"] = str(i)
        key["_authored"] = i

    if not edits:
        return task

    keys = (edits or {}).get("keys", {})
    for index, edit in keys.items():
        if str(index).isdigit():
            idx = int(index)
            if idx < len(task["keys"]):
                key = task["keys"][idx]
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

    added = (edits or {}).get("added", [])
    for idx, add in enumerate(added):
        after_id = str(add.get("after", ""))
        pos = np.asarray(add["pos"], float)
        if "dpos" in add:
            pos = pos + np.asarray(add["dpos"], float)
        new_key = {
            "_id": str(add.get("id", f"a{idx+1}")),
            "_added": True,
            "pos": pos,
            "finger": np.asarray(add.get("finger", [0.0, 1.0, 0.0]), float),
            "approach": np.asarray(add.get("approach", DOWN), float),
            "grip": float(add.get("grip", 0.0)),
            "secs": float(add.get("secs", 1.0)),
            "transit": bool(add.get("transit", False)),
        }
        insert_idx = None
        for i, k in enumerate(task["keys"]):
            if str(k.get("_id")) == after_id or str(k.get("_authored")) == after_id:
                insert_idx = i + 1
                break
        if insert_idx is not None:
            task["keys"].insert(insert_idx, new_key)
        else:
            task["keys"].insert(max(1, len(task["keys"]) - 1), new_key)

    drop_indices = []
    for i, k in enumerate(task["keys"]):
        if k.get("_authored") is not None:
            auth_idx = str(k["_authored"])
            if auth_idx != "0" and keys.get(auth_idx, {}).get("off"):
                drop_indices.append(i)
        elif k.get("_added") and k.get("_id") in keys and keys[k["_id"]].get("off"):
            drop_indices.append(i)

    for i in sorted(drop_indices, reverse=True):
        task["keys"].pop(i)

    return task


def apply_joint_edits(samples, task, edits):

    if not edits or not samples:
        return samples
    overrides = (edits or {}).get("keys", {})
    if not overrides:
        return samples

    anchors = []
    frame = 0
    for i, key in enumerate(task["keys"]):
        if i:
            frame += max(int(round(key["secs"] * RATE)), 1)
        anchors.append(min(frame, len(samples) - 1))

    out = [(np.asarray(q, float).copy(), grip) for q, grip in samples]
    override_frames = {}
    for key_index, edit in overrides.items():
        if "qpos" not in edit:
            continue
        wanted = str(key_index)
        for task_index, key in enumerate(task["keys"]):
            if (str(key.get("_id")) == wanted or
                    str(key.get("_authored")) == wanted):
                override_frames[anchors[task_index]] = edit
                break
    for frame, edit in override_frames.items():
        target = np.asarray(edit["qpos"], float)
        if target.shape == out[0][0].shape:
            out[frame] = (target.copy(), out[frame][1])

    edited_frames = set(override_frames)
    for start, end in zip(anchors[:-1], anchors[1:]):
        if start not in edited_frames and end not in edited_frames:
            continue
        a, b = out[start][0], out[end][0]
        span = max(end - start, 1)
        for j in range(start, end + 1):
            t = (j - start) / span
            q = (1.0 - t) * a + t * b
            out[j] = (q, out[j][1])
    return out


def apply_object_edits(states, task, edits, addresses):

    if not edits or not states or not addresses:
        return states
    overrides = dict((edits or {}).get("keys", {}))
    for add in (edits or {}).get("added", []):
        if "objectQpos" in add:
            overrides.setdefault(str(add.get("id", "")), add)
    if not overrides:
        return states
    anchors, frame = [], 0
    for i, key in enumerate(task["keys"]):
        if i:
            frame += max(int(round(key["secs"] * RATE)), 1)
        anchors.append(min(frame, len(states) - 1))
    out = [np.asarray(s, float).copy() for s in states]
    frames = {}
    for key_index, edit in overrides.items():
        if "objectQpos" not in edit:
            continue
        wanted = str(key_index)
        for i, key in enumerate(task["keys"]):
            if str(key.get("_id")) == wanted or str(key.get("_authored")) == wanted:
                values = np.asarray(edit["objectQpos"], float)
                if values.shape == (len(addresses),):
                    frames[anchors[i]] = values
                break
    for f, values in frames.items():
        for a, value in zip(addresses, values):
            out[f][a] = value
    for start, end in zip(anchors[:-1], anchors[1:]):
        if start not in frames and end not in frames:
            continue
        span = max(end - start, 1)
        for j in range(start, end + 1):
            t = (j - start) / span
            for a in addresses:
                out[j][a] = (1 - t) * out[start][a] + t * out[end][a]
    return out


def dropped(edits):

    return [int(i) for i, edit in (edits or {}).get("keys", {}).items()
            if edit.get("off") and str(i).isdigit() and int(i) > 0]


def key_anchors(task, count):

    anchors, frame = [], 0
    for i, key in enumerate(task["keys"]):
        if i:
            frame += max(int(round(key["secs"] * RATE)), 1)
        anchors.append(min(frame, count - 1))
    return anchors


def drive_module(model, task, states):

    drive = task.get("drive")
    if not drive:
        return states, None

    anchors = key_anchors(task, len(states))
    out = [np.asarray(s, float).copy() for s in states]

    rest = model.key_qpos[0] if model.nkey else model.qpos0
    for jid in range(model.njnt):
        name = model.joint(jid).name or ""
        if not name.startswith(f"{task['module']}_"):
            continue
        adr = model.jnt_qposadr[jid]
        for state in out:
            state[adr] = rest[adr]

    span = {}
    for name, values in drive.items():
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            continue
        adr = model.jnt_qposadr[jid]
        series = driven_series(task, values)
        lo, hi = model.jnt_range[jid]
        if model.jnt_limited[jid]:
            clamped = [min(max(v, float(lo)), float(hi)) for v in series]
            if any(abs(a - b) > 1e-4 for a, b in zip(series, clamped)):
                span[name] = "clamped"
            series = clamped
        for (a, va), (b, vb) in zip(zip(anchors, series),
                                    zip(anchors[1:], series[1:])):
            steps = max(b - a, 1)
            for j in range(a, b + 1):
                out[j][adr] = va + (vb - va) * smoothstep((j - a) / steps)
        for j in range(anchors[-1], len(out)):
            out[j][adr] = series[-1]
    return out, span


def driven_series(task, values):

    out, last = [], values[0]
    for key in task["keys"]:
        index = key.get("_authored")
        if index is not None and index < len(values):
            last = values[index]
        out.append(last)
    return out


def clamped_joints(span):

    return sorted(span or {})


def attempt(model, data, site, cfg, factory, spin, spin_adr, edits=None):

    mujoco.mj_resetDataKeyframe(model, data, 0)
    if spin_adr is not None:
        data.qpos[spin_adr] = spin
    mujoco.mj_forward(model, data)

    task = factory(model, data, cfg)
    hands_on = (not cfg.get("scripted", True)
                or task["module"] in cfg.get("physical", ()))
    if hands_on and not task.get("demo"):
        task.pop("drive", None)
    for key in task["keys"][1:]:
        key["secs"] = key["secs"] * cfg.get("pace", 1.0)
    original_grips = [float(key.get("grip", 0.0)) for key in task["keys"]]
    apply_edits(task, edits)

    path = sample_path(task["keys"])
    samples, worst = solve_ik(model, data, site, path, cfg)
    original_samples = [(np.asarray(q, float).copy(), grip) for q, grip in samples]
    samples = apply_joint_edits(samples, task, edits)

    if worst > task.get("ik_tolerance", 0.005):
        return {"task": task, "ok": False, "worst": worst, "samples": samples,
                "original_samples": original_samples, "original_grips": original_grips,
                "spin": spin, "left": 0.0,
                "why": f"out of reach: fingertips off by {worst * 1000:.0f} mm"}

    swing, states = replay(model, samples, task["watch"], cfg, spin)
    original_states = [np.asarray(s, float).copy() for s in states]
    module = task["module"]
    object_addresses = [model.jnt_qposadr[jid] for jid in range(model.njnt)
                        if (model.joint(jid).name or "").startswith(f"{module}_")]
    states, span = drive_module(model, task, states)
    states = apply_object_edits(states, task, edits, object_addresses)

    watch = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, task["watch"])
    if span is not None:
        swing = np.array([s[model.jnt_qposadr[watch]] for s in states])

    measure_from = int(task.get("measure_from", 0))
    start = sum(max(int(round(k["secs"] * RATE)), 1)
                for k in task["keys"][1:measure_from + 1])
    measured = swing[start:]
    reached = measured.max() if task["goal"] > 0 else measured.min()
    ok = abs(reached) >= abs(task["goal"]) * task.get("tolerance", 0.85)

    if ok and task.get("returns"):
        ok = abs(float(swing[-1])) <= 0.12 * abs(task["goal"])
    if ok and task.get("holds") and cfg.get("clean_release", True):
        ok = abs(float(swing[-1])) >= 0.9 * abs(task["goal"])

    past = clamped_joints(span)

    unit_name = "mm" if task["watch"].endswith("PrismaticJoint") else "rad"
    scale = 1000.0 if unit_name == "mm" else 1.0
    ending = (f" back to {swing[-1] * scale:+.2f}" if task.get("returns")
              else f", left at {swing[-1] * scale:+.2f}" if task.get("holds") else "")
    return {
        "task": task, "ok": ok, "worst": worst, "samples": samples,
        "states": states, "original_samples": original_samples,
        "original_states": original_states, "object_addresses": object_addresses,
        "original_grips": original_grips, "spin": spin,
        "left": round(float(swing[-1]), 4),
        "why": (f"{len(samples):4d} samples ({len(samples) / RATE:4.1f}s)  "
                f"ik error {worst * 1000:4.1f} mm  "
                f"{task['watch'].split('_', 1)[1]} reached {reached * scale:+.2f} "
                f"of {task['goal'] * scale:+.2f} {unit_name}{ending}"
                + (f"  [clamped to travel: {', '.join(past)}]" if past else "")),
    }


def build(scene_path, cfg):



    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "tcp")

    turn = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "board_spin")
    spin_adr = model.jnt_qposadr[turn] if turn >= 0 else None

    geoms = module_geoms(model)
    contact = (model.geom_contype.copy(), model.geom_conaffinity.copy())

    out = {}
    for module, factory in TASKS.items():
        if factory.__name__.endswith("_for"):
            factory = factory(cfg)
        if module in cfg.get("skip", ()):
            continue
        only_module(model, geoms, contact, module)

        spins = [0.0]
        if spin_adr is not None:
            facing = cfg.get("task_spins", {}).get(
                module, board_spin_for(model, data, cfg, module))
            spins = [facing, 0.0] if abs(facing) > 1e-3 else [0.0]

        best = None
        for spin in spins:
            try:
                result = attempt(model, data, site, cfg, factory, spin, spin_adr,
                                 edits_for(cfg["name"], module))
            except Exception as err:
                result = {"ok": False, "why": f"skipped: {err}", "spin": spin}
            turned = f" [board {math.degrees(spin):+.0f}°]" if spin else ""
            print(f"    {module:8s} {result['why']}{turned}"
                  f"  [{'ok' if result['ok'] else 'MISSED'}]")
            if best is None or result["ok"]:
                best = result
            if result["ok"]:
                break

        if not best or "samples" not in best:
            continue

        out[module] = package(model, site, cfg, best)
        view = views_for(cfg["name"]).get(module)
        if view:
            out[module]["view"] = view


    for module in DRAFT_MODULES:
        keys = edits_for(cfg["name"], module).get("draftKeys")
        if keys:
            out[module] = draft_build(model, site, cfg, module, keys)["traj"]

    return out


def package(model, site, cfg, result):

    task, samples = result["task"], result["samples"]
    return {
        "label": task["label"],
        "caption": task["caption"],
        "rate": RATE,
        "watch": task["watch"],
        "goal": task["goal"],
        "left": result["left"],
        "ok": bool(result.get("ok")),
        "spin": round(result["spin"], 5),
        "demo": bool(task.get("demo") or task.get("drive")),
        "qpos": [[round(v, 5) for v in q] for q, _ in samples],
        "stateQpos": [[round(float(v), 6) for v in state]
                      for state in result.get("states", [])],
        "grip": [round(g, 5) for _, g in samples],
        "tcp": [[round(v, 4) for v in p] for p in tcp_path(model, samples, site, cfg)],
    }


def tcp_path(model, samples, site, cfg):

    data = mujoco.MjData(model)
    out = []
    n = len(cfg["arm"])
    for q, _ in samples:
        data.qpos[:n] = q
        mujoco.mj_kinematics(model, data)
        out.append(np.array(data.site_xpos[site]))
    return out


def draft_home_key(cfg):

    return {
        "qpos": [float(v) for v in cfg["home"]],
        "grip": float(cfg["grip"]["open"]),
        "secs": 0.0,
        "transit": True,
    }


def draft_samples(model, cfg, keys):

    base_data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, base_data, 0)
    base = base_data.qpos.copy()
    n = len(cfg["arm"])

    usable = [k for k in keys if not k.get("off") and k.get("qpos")]
    if not usable:
        usable = [draft_home_key(cfg)]

    samples, anchors = [], []
    for i, key in enumerate(usable):
        q1 = np.asarray(key["qpos"], float)[:n]
        g1 = float(key.get("grip", cfg["grip"]["open"]))
        if i == 0:
            samples.append((q1.copy(), g1))
            anchors.append(0)
            continue
        q0, g0 = samples[-1]
        count = max(int(round(float(key.get("secs", 1.0)) * RATE)), 1)
        for j in range(1, count + 1):
            t = j / count
            samples.append(((1.0 - t) * q0 + t * q1, (1.0 - t) * g0 + t * g1))
        anchors.append(len(samples) - 1)

    if len(samples) < 2:                      
        samples.append((samples[0][0].copy(), samples[0][1]))

    states = []
    for q, _ in samples:
        state = base.copy()
        state[:n] = q
        states.append(state)
    return samples, states, anchors, usable


def draft_build(model, site, cfg, module, keys):
\
\
\
\

    samples, states, anchors, usable = draft_samples(model, cfg, keys)
    for key, at in zip(usable, anchors):
        key["sample"] = int(at)
    traj = {
        "label": f"{module} (draft)",
        "caption": "Arm-only draft — no module or physics check yet.",
        "module": module,
        "rate": RATE,
        "watch": "",
        "goal": 0.0,
        "left": 0.0,
        "spin": 0.0,
        "demo": True,
        "armOnly": True,
        "qpos": [[round(float(v), 5) for v in q] for q, _ in samples],
        "stateQpos": [[round(float(v), 6) for v in s] for s in states],
        "grip": [round(float(g), 5) for _, g in samples],
        "tcp": [[round(float(v), 4) for v in p]
                for p in tcp_path(model, samples, site, cfg)],
    }
    return {"traj": traj, "keys": usable}


def dump(trajectories, path):

    # allow_nan=False on purpose: json.dumps would otherwise write NaN and
    # Infinity as bare tokens, which the browser's JSON.parse rejects -- one of
    # them makes the whole robot unloadable, and the file looks fine to Python.
    try:
        body = json.dumps(trajectories, separators=(",", ":"), allow_nan=False)
    except ValueError as err:
        raise ValueError(f"{path.name}: refusing to publish a non-finite value ({err})") from None
    path.write_text(body + "\n")
