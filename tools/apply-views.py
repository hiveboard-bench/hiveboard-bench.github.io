#!/usr/bin/env python3
"""Fold saved camera views into the built trajectories.

The asset build already does this, but it re-solves every trajectory to get
there, which is minutes of work to move a camera.  Saving a view only changes
one field per task, so this rewrites just that field in the shipped JSON.

    python tools/apply-views.py
"""
import gzip
import json
from pathlib import Path

MODELS = Path(__file__).resolve().parent.parent / "public/sim/models"
VIEWS = Path(__file__).with_name("task_views.json")


def main():

    views = json.loads(VIEWS.read_text() or "{}") if VIEWS.exists() else {}
    if not views:
        print(f"no saved views in {VIEWS}")

    touched = 0
    for path in sorted(MODELS.glob("*.traj.json")):
        robot = path.name[: -len(".traj.json")]
        trajectories = json.loads(path.read_text())
        saved = views.get(robot, {})

        changed = []
        for task, traj in trajectories.items():
            want = saved.get(task)
            if want == traj.get("view"):
                continue
            if want:
                traj["view"] = want
            else:
                traj.pop("view", None)
            changed.append(task)

        missing = sorted(set(saved) - set(trajectories))
        if changed:
            text = json.dumps(trajectories, separators=(",", ":")) + "\n"
            path.write_text(text)
            (path.parent / f"{path.name}.gz").write_bytes(
                gzip.compress(text.encode(), 9, mtime=0))
            touched += 1
        print(f"  {robot:11s} {len(saved):2d} saved, {len(changed):2d} updated"
              + (f"   (no such task: {', '.join(missing)})" if missing else ""))

    print(f"{touched} file(s) rewritten"
          + ("" if touched else " — already up to date"))


if __name__ == "__main__":
    main()
