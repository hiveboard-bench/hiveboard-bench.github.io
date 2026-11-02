#!/usr/bin/env python3
"""Rebuild published FR3/Spot tasks and require MuJoCo acceptance replay."""

from __future__ import annotations

import os
import json
import runpy
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
TOOLS_ROOT = REPO_ROOT / "tools"


def main() -> None:
    os.chdir(REPO_ROOT)
    sys.path.insert(0, str(TOOLS_ROOT))
    catalogue = json.loads((REPO_ROOT / "public/sim/models/robots.json").read_text())
    names = [entry["name"] for entry in catalogue]
    if len(names) != len(set(names)):
        raise SystemExit(f"duplicate robot entries in catalogue: {names}")
    anymal = next((entry for entry in catalogue if entry["name"] == "anymal"), None)
    if not anymal or not anymal.get("tasks"):
        raise SystemExit("ANYmal is missing from the simulation catalogue or has no tasks")
    anymal_traj = REPO_ROOT / "public/sim/models" / anymal["traj"]
    if not anymal_traj.is_file():
        raise SystemExit(f"ANYmal trajectory file is missing: {anymal_traj}")
    saved_animations = json.loads(anymal_traj.read_text())
    missing_animations = set(anymal["tasks"]) - saved_animations.keys()
    empty_animations = [
        task for task in anymal["tasks"]
        if not saved_animations.get(task, {}).get("qpos")
    ]
    if missing_animations or empty_animations:
        raise SystemExit(
            "ANYmal animation data is incomplete: "
            f"missing={sorted(missing_animations)}, empty={empty_animations}"
        )

    namespace = runpy.run_path(str(TOOLS_ROOT / "build-sim-assets.py"))
    trajectory_builder = namespace["sim_trajectories"]
    robots = {robot["name"]: robot for robot in namespace["ROBOTS"]}
    published = {
        entry["name"]: set(entry.get("tasks", []))
        for entry in catalogue
    }

    failures = []
    for name in ("fr3", "spot"):
        scene = REPO_ROOT / "public" / "sim" / "models" / f"{name}.xml"
        results = trajectory_builder.build(scene, robots[name])
        accepted = {module for module, result in results.items() if result.get("ok")}
        required = published[name] if name == "fr3" else {"valve", "lamp", "breaker"}
        failed_required = sorted(required - accepted)
        empty_paths = sorted(
            module for module, result in results.items()
            if not result.get("qpos") or not result.get("tcp")
        )
        previews = len(set(results) - accepted)
        print(
            f"{name}: {len(accepted)}/{len(results)} trajectories pass MuJoCo replay; "
            f"{previews} best-effort previews remain available for editing"
        )
        if failed_required or empty_paths:
            failures.append((name, failed_required, empty_paths))

    if failures:
        raise SystemExit(f"Tasks failed acceptance replay: {failures}")


if __name__ == "__main__":
    main()
