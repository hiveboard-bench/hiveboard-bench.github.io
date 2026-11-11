#!/usr/bin/env python3
"""Load every published scene and smoke-test saved poses with MuJoCo (no GPU).

This checks asset/trajectory compatibility and finite physics, not task success.
Run with the environment from tools/requirements-simulation.txt.
"""
import json
from pathlib import Path

import mujoco
import numpy as np

MODELS = Path(__file__).resolve().parents[1] / "public/sim/models"


def main():
    for robot in json.loads((MODELS / "robots.json").read_text()):
        model = mujoco.MjModel.from_xml_path(str(MODELS / robot["scene"]))
        data = mujoco.MjData(model)
        trajectories = json.loads((MODELS / robot["traj"]).read_text())
        grip = model.actuator(robot["grip"]["actuator"]).id
        assert model.nu > robot["arm"] and grip >= 0
        contact = (model.geom_contype.copy(), model.geom_conaffinity.copy())
        for name in robot["tasks"]:
            task = trajectories[name]
            states = np.asarray(task.get("stateQpos", []))
            if states.size:
                assert states.shape == (len(task["qpos"]), model.nq), (robot["name"], name, states.shape, model.nq)
                assert np.isfinite(states).all()
            assert model.joint(task["watch"]).id >= 0
            # Hide inactive module collisions, as the browser does.
            model.geom_contype[:] = contact[0]
            model.geom_conaffinity[:] = contact[1]
            for g in range(model.ngeom):
                body = model.body(int(model.geom_bodyid[g])).name
                if any(body.startswith(key + "_") for key in trajectories if key != name):
                    model.geom_contype[g] = model.geom_conaffinity[g] = 0
            for sample in np.linspace(0, len(task["qpos"]) - 1, 5, dtype=int):
                mujoco.mj_resetDataKeyframe(model, data, 0)
                if states.size:
                    data.qpos[:] = states[sample]
                else:
                    data.qpos[:robot["arm"]] = task["qpos"][sample]
                data.ctrl[:robot["arm"]] = task["qpos"][sample]
                data.ctrl[grip] = task["grip"][sample]
                mujoco.mj_forward(model, data)
                for _ in range(10):
                    mujoco.mj_step(model, data)
                assert np.isfinite(data.qpos).all() and np.isfinite(data.xpos).all(), (robot["name"], name, sample)
                assert not data.warning.number.any(), (robot["name"], name, sample, data.warning.number)
        print(f"PASS {robot['name']}: scene, {len(robot['tasks'])} trajectories, saved-state dimensions and physics smoke test", flush=True)


if __name__ == "__main__":
    main()
