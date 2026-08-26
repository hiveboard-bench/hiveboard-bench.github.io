#!/usr/bin/env python3
"""Drag a trajectory's waypoints around in the browser. TEMPORARY dev tool.

The tasks in sim_trajectories.py are authored as a handful of task-space
keyframes -- stand off the lever, come down onto it, close, swing -- and every
one of those is derived from the compiled scene. That is what keeps them
honest, but it also means the only way to move one has been to reason back
from a knob like `valve_grasp` to where the hand ends up. This serves the
other direction: the widget draws the keyframes as beads on the path it
already shows, you drag one, and this re-solves that task with the real IK and
the real acceptance replay -- the same code the build runs, so a path that
looks right here is a path that passes there.

    python3 tools/traj_edit.py                  # then, in another terminal
    npm run dev
    open http://localhost:5173/sim/hiveboard-sim.html?edit=1

Drag a bead, or nudge the selected one with the arrow keys; the task re-solves
in about a second and plays back straight away. `Save` writes the offsets to
tools/traj_edits.json and republishes that robot's .traj.json, so the widget
shows them without the editor too, and the next full
`python3 tools/build-sim-assets.py` picks them up rather than solving them
away again.

Nothing here is served to visitors: the widget only loads the editor when the
page is opened with `?edit=1`, and this server is not part of the site.

To remove the editor entirely, delete:
    tools/traj_edit.py, tools/traj_edits.json, public/sim/edit.js,
    the "Hand edits" block and its two callers in tools/sim_trajectories.py,
    the `?edit=1` block at the bottom of public/sim/hiveboard-sim.html.
"""
import argparse
import gzip
import importlib.util
import json
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
MODELS = REPO / "public/sim/models"

sys.path.insert(0, str(REPO / "tools"))
import sim_trajectories as st                                    # noqa: E402

# build-sim-assets.py is not importable by name (the hyphens), and it is the
# one place the per-robot config lives -- reach into it rather than keeping a
# second copy of Spot's numbers here that would quietly drift.
_spec = importlib.util.spec_from_file_location(
    "build_sim_assets", REPO / "tools/build-sim-assets.py")
build_sim_assets = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(build_sim_assets)

# One MuJoCo model per robot, compiled from the scene the build already wrote,
# and one lock around it: MjData is not re-entrant and the browser will happily
# fire two solves at once.
_scenes = {}
_lock = threading.Lock()


def scene(name):
    """Compile a robot's published scene once, with its build-time config."""
    import mujoco

    if name not in _scenes:
        cfg = next((r for r in build_sim_assets.ROBOTS if r["name"] == name), None)
        if cfg is None or cfg.get("soon"):
            raise KeyError(f"no such robot: {name}")
        path = MODELS / f"{name}.xml"
        if not path.exists():
            raise FileNotFoundError(f"{path} -- run tools/build-sim-assets.py first")
        # emit_robot() hands the solver the board's facing rather than its
        # quaternion, and every task's approach direction comes off that, so
        # the same derivation has to happen here.
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
    """The task factories this robot runs, keyed by module name."""
    out = {}
    for factory in st.TASKS:
        if factory.__name__.endswith("_for"):
            factory = factory(cfg)
        module = factory.__name__.split("_")[0]
        if module not in cfg.get("skip", ()):
            out[module] = factory
    return out


def published(name):
    """That robot's shipped trajectories, for the board angles they were solved at."""
    path = MODELS / f"{name}.traj.json"
    return json.loads(path.read_text()) if path.exists() else {}


# ═══════════════════════════════════════════════════════════════════════════
#  Keyframes, as the editor sees them
# ═══════════════════════════════════════════════════════════════════════════
def describe(task, home, off=()):
    """Where each authored keyframe puts the hand, once the path is expanded.

    A pose key sits exactly where it says it does, but an `arc` key only says
    how far to sweep -- its endpoint falls out of the path. Both get a bead,
    so run the sampler and read the position off the end of each segment.

    Keys switched off are already gone from the path by the time this runs, so
    they come back as placeholder rows at the index they came from: the editor
    numbers its edits against the task as authored, and a row that renumbered
    itself when its neighbour was switched off would move somebody else's
    offset onto the wrong keyframe.
    """
    keys = task["keys"]
    path = st.sample_path(keys)
    off = set(off)
    live = [i for i in range(len(keys) + len(off)) if i not in off]
    at, out = 0, [{"index": i, "kind": "off", "off": True, "pos": None,
                   "secs": None, "grip": None, "transit": False, "home": False,
                   "angle": None, "rise": None, "draggable": False} for i in sorted(off)]
    for i, key in enumerate(keys):
        if i:
            at += max(int(round(key["secs"] * st.RATE)), 1)
        arc = key.get("arc")
        pos = np.asarray(path[at][0], float)
        # A key sitting on the arm's own rest pose is the trajectory leaving
        # or coming back to home. Where that is belongs to the robot's config,
        # not to one task, so it is shown but not draggable.
        home_key = bool(np.linalg.norm(pos - home) < 1e-6)
        out.append({
            "index": live[i],
            "off": False,
            "kind": "arc" if arc else "pose",
            "pos": [round(float(v), 5) for v in pos],
            "secs": round(float(key.get("secs", 0.0)), 3),
            "grip": round(float(key["grip"]), 4),
            "transit": bool(key.get("transit")),
            "home": home_key,
            "angle": round(float(arc["angle"]), 4) if arc else None,
            "rise": round(float(arc.get("rise", 0.0)), 5) if arc else None,
            "draggable": arc is None and not home_key,
        })
    return sorted(out, key=lambda row: row["index"])


def solve(name, module, edits, spin=0.0):
    """Solve and replay one task with the given edits. The build's own code."""
    ctx = scene(name)
    cfg, model, data = ctx["cfg"], ctx["model"], ctx["data"]
    factory = factories(cfg).get(module)
    if factory is None:
        raise KeyError(f"{name} has no {module} task")

    result = st.attempt(model, data, ctx["site"], cfg, factory,
                        spin, ctx["spin_adr"], edits)
    task = result["task"]
    home = st.home_key(model, data, cfg, 0.0)["pos"]
    out = {
        "module": module,
        "ok": bool(result["ok"]),
        "why": result["why"],
        "keys": describe(task, home, st.dropped(edits)),
    }
    if "samples" in result:
        out["traj"] = st.package(model, ctx["site"], cfg, result)
    return out


def republish(name, edits):
    """Re-solve the edited tasks and rewrite the .traj.json the widget loads.

    Only the tasks that were actually edited: a published trajectory can
    predate a later change to the robot's config, and re-solving one nobody
    touched would quietly swap it for a different motion in the same commit as
    an unrelated tweak. `python3 tools/build-sim-assets.py` is still the way to
    bring the whole robot up to date.

    The widget fetches the gzipped copy, so both are written -- as
    build-sim-assets.py's manifest step does, minus the meshes, which have not
    changed and take a minute to rebuild.
    """
    tasks = published(name)
    report = []
    for module in factories(scene(name)["cfg"]):
        if not edits.get(module, {}).get("keys"):
            report.append({"module": module, "ok": True, "why": "left as published"})
            continue
        spin = float(tasks.get(module, {}).get("spin", 0.0))
        result = solve(name, module, edits[module], spin)
        report.append({"module": module, "ok": result["ok"], "why": result["why"]})
        if "traj" in result:
            tasks[module] = result["traj"]

    path = MODELS / f"{name}.traj.json"
    st.dump(tasks, path)
    gz = path.with_suffix(path.suffix + ".gz")
    gz.write_bytes(gzip.compress(path.read_bytes(), 9))
    return report


def read_edits():
    return st.load_edits()


def write_edits(all_edits):
    st.EDITS_FILE.write_text(json.dumps(all_edits, indent=1, sort_keys=True) + "\n")


# ═══════════════════════════════════════════════════════════════════════════
#  HTTP
# ═══════════════════════════════════════════════════════════════════════════
class Handler(BaseHTTPRequestHandler):
    def reply(self, payload, code=200):
        body = json.dumps(payload).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def body(self):
        return json.loads(self.rfile.read(int(self.headers["Content-Length"] or 0)) or "{}")

    def route(self, path, payload):
        name = payload.get("robot", "spot")

        if path == "/state":
            return {"robot": name,
                    "modules": list(factories(scene(name)["cfg"])),
                    "edits": read_edits().get(name, {})}

        if path == "/solve":
            return solve(name, payload["module"], payload.get("edits", {}),
                         float(payload.get("spin", 0.0)))

        if path == "/save":
            all_edits = read_edits()
            all_edits[name] = {k: v for k, v in payload.get("edits", {}).items()
                               if v.get("keys")}
            if not all_edits[name]:
                all_edits.pop(name)
            write_edits(all_edits)
            report = republish(name, payload.get("edits", {}))
            for line in report:
                print(f"  saved {line['module']:8s} {line['why']} "
                      f"[{'ok' if line['ok'] else 'MISSED'}]")
            return {"saved": str(st.EDITS_FILE.relative_to(REPO)), "report": report}

        return None

    def handle_request(self, path, payload):
        try:
            with _lock:
                result = self.route(path, payload)
        except Exception as err:                       # bad geometry, typo, …
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
        pass                                            # the solves log enough


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", type=int, default=8770)
    ap.add_argument("--robot", default="spot", help="preload this robot's scene")
    args = ap.parse_args()

    print(f"compiling {args.robot}…")
    ctx = scene(args.robot)
    print(f"  tasks: {', '.join(factories(ctx['cfg']))}")
    print(f"trajectory editor on http://localhost:{args.port}")
    print("open  http://localhost:5173/sim/hiveboard-sim.html?edit=1")
    ThreadingHTTPServer(("127.0.0.1", args.port), Handler).serve_forever()


if __name__ == "__main__":
    main()
