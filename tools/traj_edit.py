#!/usr/bin/env python3
import argparse
import gzip
import importlib.util
import json
import math
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import mujoco
import numpy as np

REPO = Path(__file__).resolve().parent.parent
MODELS = REPO / "public/sim/models"

sys.path.insert(0, str(REPO / "tools"))
import sim_trajectories as st

_spec = importlib.util.spec_from_file_location(
    "build_sim_assets", REPO / "tools/build-sim-assets.py")
build_sim_assets = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(build_sim_assets)

_scenes = {}
_lock = threading.Lock()


def scene(name):

    if name not in _scenes:
        cfg = next((r for r in build_sim_assets.ROBOTS if r["name"] == name), None)
        if cfg is None or cfg.get("soon"):
            raise KeyError(f"no such robot: {name}")
        path = MODELS / f"{name}.xml"
        if not path.exists():
            raise FileNotFoundError(f"{path} -- run tools/build-sim-assets.py first")
        cfg = dict(cfg, board_normal=build_sim_assets.board_normal(cfg))
        model = mujoco.MjModel.from_xml_path(str(path))
        data = mujoco.MjData(model)
        turn = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "board_spin")
        _scenes[name] = {
            "cfg": cfg,
            "model": model,
            "data": data,
            "site": mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "tcp"),
            "spin_adr": int(model.jnt_qposadr[turn]) if turn >= 0 else None,
        }
    return _scenes[name]


def factories(cfg):

    out = {}
    for module, factory in st.TASKS.items():
        if factory.__name__.endswith("_for"):
            factory = factory(cfg)
        if module not in cfg.get("skip", ()):
            out[module] = factory
    return out


def in_scene(ctx, module):
    """Is this module actually in the compiled scene?

    A robot's scene is generated with whichever modules existed at the time, so
    a model built before a module was added simply has no bodies for it.  The
    editor used to offer them all and fail inside the solver on the missing
    ones; ask the model instead.
    """
    model = ctx["model"]
    prefix = f"{module}_"
    return any((mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b) or "").startswith(prefix)
               for b in range(model.nbody))


def scene_modules(ctx):
    return [m for m in factories(ctx["cfg"]) if in_scene(ctx, m)]


def joint_info(ctx):

    model = ctx["model"]
    names, ranges = [], []
    for name in ctx["cfg"].get("arm", ()):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        names.append(name)
        ranges.append([float(model.jnt_range[jid, 0]), float(model.jnt_range[jid, 1])])
    return names, ranges


def finite(value):
    """The payload with any NaN or infinity replaced by null."""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {k: finite(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [finite(v) for v in value]
    return value


def grip_range(ctx):

    model = ctx["model"]
    aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR,
                             ctx["cfg"]["grip"]["actuator"])
    return [float(model.actuator_ctrlrange[aid, 0]),
            float(model.actuator_ctrlrange[aid, 1])]


def object_info(ctx, module):

    model = ctx["model"]
    names, addresses, ranges = [], [], []
    prefix = f"{module}_"
    for jid in range(model.njnt):
        name = model.joint(jid).name
        if not name or not name.startswith(prefix):
            continue
        names.append(name)
        addresses.append(int(model.jnt_qposadr[jid]))
        ranges.append([float(model.jnt_range[jid, 0]), float(model.jnt_range[jid, 1])])
    return names, addresses, ranges


def published(name):

    path = MODELS / f"{name}.traj.json"
    return json.loads(path.read_text()) if path.exists() else {}


def describe(task, home, edits=None, samples=None, original_samples=None,
             original_grips=None, states=None, original_states=None,
             object_addresses=None):

    keys = task["keys"]
    path = st.sample_path(keys)
    at = 0
    live_rows = []
    for i, key in enumerate(keys):
        if i:
            at += max(int(round(key["secs"] * st.RATE)), 1)
        arc = key.get("arc")
        pos = np.asarray(path[at][0], float)
        home_key = bool(np.linalg.norm(pos - home) < 1e-6)
        is_added = bool(key.get("_added"))
        key_id = key.get("_id", str(i))
        auth_idx = key.get("_authored")

        finger = key.get("finger")
        if finger is None and at < len(path):
            finger = path[at][1]
        approach = key.get("approach")
        if approach is None and at < len(path):
            approach = path[at][2]

        live_rows.append({
            "index": key_id,
            "authored": auth_idx,
            "added": is_added,
            "off": False,
            "kind": "arc" if arc else "pose",
            "pos": [round(float(v), 5) for v in pos],
            "finger": [round(float(v), 5) for v in finger] if finger is not None else [0.0, 1.0, 0.0],
            "approach": [round(float(v), 5) for v in approach] if approach is not None else [0.0, 0.0, -1.0],
            "secs": round(float(key.get("secs", 0.0)), 3),
            "grip": round(float(key["grip"]), 4),
            "originalGrip": (round(float(original_grips[key["_authored"]]), 4)
                             if original_grips is not None and key.get("_authored") is not None
                             and key["_authored"] < len(original_grips)
                             else round(float(key["grip"]), 4)),
            "objectQpos": ([round(float(states[min(at, len(states) - 1)][a]), 6)
                            for a in object_addresses]
                           if states and object_addresses else None),
            "originalObjectQpos": ([round(float(original_states[min(at, len(original_states) - 1)][a]), 6)
                                    for a in object_addresses]
                                   if original_states and object_addresses else None),
            "transit": bool(key.get("transit")),
            "home": home_key,
            "angle": round(float(arc["angle"]), 4) if arc else None,
            "rise": round(float(arc.get("rise", 0.0)), 5) if arc else None,
            "draggable": arc is None,
            "qpos": ([round(float(v), 6) for v in samples[min(at, len(samples) - 1)][0]]
                     if samples else None),
            "originalQpos": ([round(float(v), 6) for v in original_samples[min(at, len(original_samples) - 1)][0]]
                              if original_samples else None),
            "sample": min(at, len(samples) - 1) if samples else None,
        })

    off_indices = st.dropped(edits)
    if not off_indices:
        return live_rows

    out = list(live_rows)
    for off_idx in sorted(off_indices):
        placeholder = {
            "index": str(off_idx),
            "authored": off_idx,
            "added": False,
            "kind": "off",
            "off": True,
            "pos": None,
            "finger": None,
            "approach": None,
            "secs": None,
            "grip": None,
            "originalGrip": None,
            "objectQpos": None,
            "originalObjectQpos": None,
            "transit": False,
            "home": False,
            "angle": None,
            "rise": None,
            "draggable": False,
            "qpos": None,
            "originalQpos": None,
            "sample": None,
        }
        inserted = False
        for pos_idx, row in enumerate(out):
            if row.get("authored") is not None and row["authored"] > off_idx:
                out.insert(pos_idx, placeholder)
                inserted = True
                break
        if not inserted:
            out.append(placeholder)

    return out


def solve(name, module, edits, spin=0.0):

    ctx = scene(name)
    cfg, model, data = ctx["cfg"], ctx["model"], ctx["data"]
    factory = factories(cfg).get(module)
    if factory is None:
        return {
            "module": module,
            "ok": False,
            "why": f"{name} has no {module} task",
            "keys": [],
        }
    if not in_scene(ctx, module):
        return {
            "module": module,
            "ok": False,
            "why": f"{name}.xml has no {module} bodies -- rebuild it with "
                   f"tools/build-sim-assets.py --robot {name}",
            "keys": [],
        }

    result = st.attempt(model, data, ctx["site"], cfg, factory,
                        spin, ctx["spin_adr"], edits)
    task = result["task"]
    home = st.home_key(model, data, cfg, 0.0)["pos"]
    out = {
        "module": module,
        "ok": bool(result["ok"]),
        "why": result["why"],
        "keys": describe(task, home, edits, result.get("samples"),
                          result.get("original_samples"),
                          result.get("original_grips"), result.get("states"),
                          result.get("original_states"),
                          result.get("object_addresses")),
        "joints": list(cfg["arm"]),
        "jointRanges": joint_info(ctx)[1],
        "gripRange": grip_range(ctx),
        "objectJoints": object_info(ctx, module)[0],
        "objectRanges": object_info(ctx, module)[2],
    }
    if "samples" in result:
        out["traj"] = st.package(model, ctx["site"], cfg, result)
    return out


def manual_solve(name, module, keys, spin=0.0):

    ctx = scene(name)
    cfg, model = ctx["cfg"], ctx["model"]

    usable = [k for k in keys if not k.get("off") and k.get("qpos")]
    if len(usable) < 2:
        return {"module": module, "ok": False, "why": "need at least two manual poses", "keys": keys}
    samples = []
    object_names, object_addresses, object_ranges = object_info(ctx, module)
    base = np.zeros(model.nq, float)
    mujoco.mj_resetDataKeyframe(model, base_data := mujoco.MjData(model), 0)
    turn = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "board_spin")
    if turn >= 0:
        base_data.qpos[model.jnt_qposadr[turn]] = spin
    # Unconditionally, not just when the board spins: the task factory below
    # reads joint frames that only a forward pass fills in.
    mujoco.mj_forward(model, base_data)
    base[:] = base_data.qpos

    def object_pose(key):

        values = key.get("objectQpos")
        return (np.asarray(values, float) if values and len(values) == len(object_addresses)
                else np.asarray([base[a] for a in object_addresses], float))

    sample_indices = []
    for index, key in enumerate(usable):
        q1 = np.asarray(key["qpos"], float)
        g1 = float(key.get("grip", 0.0))
        if index == 0:
            samples.append((q1.copy(), g1))
            object_samples = [object_pose(key)]
            sample_indices.append(0)
            continue
        q0, g0 = samples[-1]
        o0, o1 = object_samples[-1], object_pose(key)
        count = max(int(round(float(key.get("secs", 1.0)) * st.RATE)), 1)
        sample_indices.append(len(samples) + count - 1)
        for j in range(1, count + 1):
            t = j / count
            samples.append(((1 - t) * q0 + t * q1, (1 - t) * g0 + t * g1))
            object_samples.append((1 - t) * o0 + t * o1)

    for key, sample_index in zip(usable, sample_indices):
        key["sample"] = sample_index

    states = []
    for (q, _), obj in zip(samples, object_samples):
        state = base.copy()
        state[:len(cfg["arm"])] = q
        for address, value in zip(object_addresses, obj):
            state[address] = value
        states.append(state)
    # The factory reads joint frames off the data (xanchor/xaxis), which are
    # only populated by a forward pass -- base_data has had one, a fresh MjData
    # has not, and unit() on its zero axis returns NaN. That NaN reached the
    # browser as the bare token `NaN`, which JSON.parse rejects.
    task = factories(cfg)[module](model, base_data, cfg)
    fake = {"task": task, "samples": samples, "states": states,
            "spin": spin, "left": 0.0}
    out = st.package(model, ctx["site"], cfg, fake)
    out["label"] = "Change the lamp"
    out["caption"] = "Manual pose trajectory"
    out["manual"] = True
    return {"module": module, "ok": True, "why": f"manual trajectory: {len(samples)} samples",
            "traj": out, "keys": keys, "joints": list(cfg["arm"]),
            "jointRanges": joint_info(ctx)[1], "gripRange": grip_range(ctx),
            "objectJoints": object_names, "objectRanges": object_ranges}


def draft_rows(keys, cfg):

    open_grip = float(cfg["grip"]["open"])
    rows = []
    for i, key in enumerate(keys):
        q = [round(float(v), 6) for v in key["qpos"]]
        rows.append({
            "index": str(i),
            "authored": i,
            "added": False,
            "off": False,
            "kind": "pose",
            "pos": None,
            "finger": None,
            "approach": None,
            "secs": round(float(key.get("secs", 0.0 if i == 0 else 1.0)), 3),
            "grip": round(float(key.get("grip", open_grip)), 4),
            "originalGrip": round(float(key.get("grip", open_grip)), 4),
            "objectQpos": None,
            "originalObjectQpos": None,
            "transit": bool(key.get("transit")),
            "home": i == 0,
            "angle": None,
            "rise": None,
            "draggable": False,
            "qpos": q,
            "originalQpos": q,
            "sample": int(key.get("sample", 0)),
        })
    return rows


def saved_draft_keys(name, module):

    return read_edits().get(name, {}).get(module, {}).get("draftKeys")


def handle_draft(name, module, keys):

    if module not in st.DRAFT_MODULES:
        return {"module": module, "ok": False,
                "why": f"{module} is not a draft module", "keys": []}
    ctx = scene(name)
    cfg, model = ctx["cfg"], ctx["model"]
    keys = keys or saved_draft_keys(name, module) or [st.draft_home_key(cfg)]
    built = st.draft_build(model, ctx["site"], cfg, module, keys)
    return {
        "module": module,
        "ok": True,
        "armOnly": True,
        "why": f"draft: {len(built['keys'])} pose(s), "
               f"{len(built['traj']['qpos'])} samples",
        "traj": built["traj"],
        "keys": draft_rows(built["keys"], cfg),
        "joints": list(cfg["arm"]),
        "jointRanges": joint_info(ctx)[1],
        "gripRange": grip_range(ctx),
        "objectJoints": [],
        "objectRanges": [],
    }


def handle_draft_save(name, module, keys):

    if module not in st.DRAFT_MODULES:
        return {"module": module, "ok": False,
                "why": f"{module} is not a draft module"}
    ctx = scene(name)
    cfg = ctx["cfg"]
    clean = [{k: key[k] for k in ("qpos", "grip", "secs", "transit") if k in key}
             for key in keys if key.get("qpos") and not key.get("off")]

    all_edits = read_edits()
    node = all_edits.setdefault(name, {}).setdefault(module, {})
    if clean:
        node["draftKeys"] = clean
    else:
        node.pop("draftKeys", None)
        if not node:
            all_edits[name].pop(module, None)
            if not all_edits[name]:
                all_edits.pop(name, None)
    write_edits(all_edits)

    tasks = published(name)
    if clean:
        built = st.draft_build(ctx["model"], ctx["site"], cfg, module, clean)
        tasks[module] = built["traj"]
        samples = len(built["traj"]["qpos"])
    else:
        tasks.pop(module, None)
        samples = 0
    path = MODELS / f"{name}.traj.json"
    st.dump(tasks, path)
    path.with_suffix(path.suffix + ".gz").write_bytes(
        gzip.compress(path.read_bytes(), 9, mtime=0))
    print(f"  saved {module:12s} draft: {len(clean)} pose(s), {samples} samples")
    return {"saved": str(st.EDITS_FILE.relative_to(REPO)), "module": module,
            "ok": True, "poses": len(clean), "samples": samples}


def republish(name, edits, manual_keys=None):

    tasks = published(name)
    report = []
    for module in factories(scene(name)["cfg"]):
        if module == "lamp" and manual_keys:
            result = manual_solve(name, module, manual_keys,
                                  float(tasks.get(module, {}).get("spin", 0.0)))
            if "traj" in result:
                tasks[module] = result["traj"]
            report.append({"module": module, "ok": result["ok"], "why": result["why"]})
            continue
        mod_edits = edits.get(module, {})
        if not mod_edits.get("keys") and not mod_edits.get("added"):
            report.append({"module": module, "ok": True, "why": "left as published"})
            continue
        spin = float(tasks.get(module, {}).get("spin", 0.0))
        result = solve(name, module, mod_edits, spin)
        report.append({"module": module, "ok": result["ok"], "why": result["why"]})
        if "traj" in result:
            tasks[module] = result["traj"]

    path = MODELS / f"{name}.traj.json"
    st.dump(tasks, path)
    gz = path.with_suffix(path.suffix + ".gz")
    gz.write_bytes(gzip.compress(path.read_bytes(), 9, mtime=0))
    return report


def read_edits():

    return st.load_edits()


def write_edits(all_edits):

    st.EDITS_FILE.write_text(json.dumps(all_edits, indent=1, sort_keys=True) + "\n")


# Camera framing per robot and task.  It lives beside traj_edits.json rather
# than under models/, because the asset build wipes that directory.
VIEWS_FILE = Path(__file__).with_name("task_views.json")


def read_views():

    if VIEWS_FILE.exists():
        return json.loads(VIEWS_FILE.read_text() or "{}")
    return {}


def handle_view_save(name, task, target, position):

    data = read_views()
    node = data.setdefault(name, {})
    if target is None or position is None:
        node.pop(task, None)
        if not node:
            data.pop(name, None)
        entry = None
    else:
        entry = {"target": [round(float(v), 4) for v in target][:3],
                 "position": [round(float(v), 4) for v in position][:3]}
        node[task] = entry
    VIEWS_FILE.write_text(json.dumps(data, indent=1, sort_keys=True) + "\n")
    return {"saved": str(VIEWS_FILE), "view": entry,
            "views": data.get(name, {})}


PLACEMENT_FILE = MODELS / "module_placement.json"


def read_placement():

    if PLACEMENT_FILE.exists():
        return json.loads(PLACEMENT_FILE.read_text() or "{}")
    return {}


def handle_place_save(name, module, offset, spin, scale):

    data = read_placement()
    entry = {
        "offset": [round(float(v), 5) for v in (offset or [0, 0, 0])][:3],
        "spin": round(float(spin or 0.0), 5),
        "scale": round(float(scale if scale else 1.0), 4),
    }
    zero = entry["offset"] == [0.0, 0.0, 0.0] and entry["spin"] == 0.0 \
        and entry["scale"] == 1.0
    node = data.setdefault(name, {})
    if zero:
        node.pop(module, None)
        if not node:
            data.pop(name, None)
    else:
        node[module] = entry
    PLACEMENT_FILE.write_text(json.dumps(data, indent=1, sort_keys=True) + "\n")
    print(f"  placed {name}/{module}: {entry if not zero else 'reset'}")
    return {"saved": str(PLACEMENT_FILE.relative_to(REPO)), "robot": name,
            "module": module, "placement": data.get(name, {}).get(module),
            "ok": True}


class Handler(BaseHTTPRequestHandler):
    def reply(self, payload, code=200):

        try:
            # json.dumps writes NaN and Infinity as bare tokens, which are legal
            # Python but not JSON -- the browser rejects the whole body and the
            # editor cannot say why. Send null instead, and name it in the log.
            try:
                body = json.dumps(payload, allow_nan=False).encode()
            except ValueError:
                print(f"  warning: {self.path} produced a non-finite value; sent as null")
                body = json.dumps(finite(payload)).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_OPTIONS(self):

        try:
            self.send_response(204)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type")
            self.end_headers()
        except (BrokenPipeError, ConnectionResetError):
            pass

    def body(self):

        return json.loads(self.rfile.read(int(self.headers["Content-Length"] or 0)) or "{}")

    def route(self, path, payload):

        name = payload.get("robot", "spot")

        if path == "/state":
            ctx = scene(name)
            joint_names, joint_ranges = joint_info(ctx)
            robot_edits = read_edits().get(name, {})
            return {"robot": name,
                    "modules": scene_modules(ctx) + list(st.DRAFT_MODULES),
                    "drafts": list(st.DRAFT_MODULES),
                    "joints": joint_names,
                    "jointRanges": joint_ranges,
                    "gripRange": grip_range(ctx),
                    "edits": robot_edits,
                    "placement": read_placement().get(name, {}),
                    "views": read_views().get(name, {}),
                    "manualKeys": robot_edits.get("lamp", {}).get("manualKeys")}

        if path == "/solve":
            return solve(name, payload["module"], payload.get("edits", {}),
                         float(payload.get("spin", 0.0)))

        if path == "/manual":
            return manual_solve(name, payload["module"], payload.get("keys", []),
                                float(payload.get("spin", 0.0)))

        if path == "/draft":
            return handle_draft(name, payload["module"], payload.get("keys"))

        if path == "/draft-save":
            return handle_draft_save(name, payload["module"],
                                     payload.get("keys", []))

        if path == "/views":
            return {"views": read_views().get(name, {})}

        if path == "/view-save":
            return handle_view_save(name, payload["task"],
                                    payload.get("target"),
                                    payload.get("position"))

        if path == "/place-save":
            return handle_place_save(name, payload["module"],
                                     payload.get("offset"), payload.get("spin"),
                                     payload.get("scale"))

        if path == "/save":
            all_edits = read_edits()
            clean_robot_edits = {}
            for k, v in payload.get("edits", {}).items():
                mod_dict = {}
                if v.get("keys"):
                    mod_dict["keys"] = v["keys"]
                if v.get("added"):
                    mod_dict["added"] = v["added"]
                if v.get("manualKeys"):
                    mod_dict["manualKeys"] = v["manualKeys"]
                if mod_dict:
                    clean_robot_edits[k] = mod_dict

            if clean_robot_edits:
                all_edits[name] = clean_robot_edits
            else:
                all_edits.pop(name, None)

            if payload.get("manualKeys"):
                all_edits.setdefault(name, {}).setdefault("lamp", {})["manualKeys"] = payload["manualKeys"]

            write_edits(all_edits)
            report = republish(name, payload.get("edits", {}),
                               payload.get("manualKeys"))
            for line in report:
                print(f"  saved {line['module']:8s} {line['why']} "
                      f"[{'ok' if line['ok'] else 'MISSED'}]")
            response = {"saved": str(st.EDITS_FILE.relative_to(REPO)), "report": report}
            if payload.get("manualKeys"):
                response["trajectory"] = published(name).get("lamp")
            return response

        return None

    def handle_request(self, path, payload):

        try:
            with _lock:
                result = self.route(path, payload)
        except Exception as err:
            import traceback

            traceback.print_exc()
            return self.reply({"error": f"{type(err).__name__}: {err}"}, 500)
        if result is None:
            return self.reply({"error": f"no route {path}"}, 404)
        self.reply(result)

    def do_GET(self):

        path, _, query = self.path.partition("?")
        payload = dict(p.split("=", 1) for p in query.split("&") if "=" in p)
        self.handle_request(path, payload)

    def do_POST(self):

        self.handle_request(self.path.partition("?")[0], self.body())

    def log_message(self, fmt, *args):

        pass


class Server(ThreadingHTTPServer):
    daemon_threads = True

    def handle_error(self, request, client_address):

        exc_type, _, _ = sys.exc_info()
        if exc_type and issubclass(exc_type, (BrokenPipeError, ConnectionResetError)):
            return
        super().handle_error(request, client_address)


def main():

    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0] if __doc__ else "")
    ap.add_argument("--port", type=int, default=8770)
    ap.add_argument("--robot", default="spot", help="preload this robot's scene")
    args = ap.parse_args()

    print(f"compiling {args.robot}…")
    ctx = scene(args.robot)
    print(f"  tasks: {', '.join(factories(ctx['cfg']))}")
    print(f"trajectory editor on http://localhost:{args.port}")
    print("open  http://localhost:5173/sim/hiveboard-sim.html?edit=1")
    Server(("127.0.0.1", args.port), Handler).serve_forever()


if __name__ == "__main__":
    main()
