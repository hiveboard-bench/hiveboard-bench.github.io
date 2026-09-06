#!/usr/bin/env python3
"""Check the exported robot against its URDF and exercise the gripper/editor.

Run with the same Python environment as build-sim-assets.py. No Isaac Sim or
USD runtime is needed; --isaaclab-repo locates the reference arm URDF.
"""
import argparse
import gzip
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

import anymal_model
import traj_edit

REPO = Path(__file__).resolve().parent.parent


def rotation(axis, angle):
    axis = np.asarray(axis, float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + np.sin(angle) * cross + (1 - np.cos(angle)) * cross @ cross


def transform(pos, rot):
    out = np.eye(4)
    out[:3, :3], out[:3, 3] = rot, pos
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--isaaclab-repo", type=Path, default=REPO.parent.parent)
    args = parser.parse_args()
    ctx = traj_edit.scene("anymal")
    model, data = ctx["model"], ctx["data"]
    arm = anymal_model.ARM_JOINTS
    assert [model.joint(i).name for i in range(6)] == arm
    assert [model.actuator(i).name for i in range(7)] == arm + ["finger_joint"]
    assert model.nu == 7 and model.nq == 18
    assert len(traj_edit.joint_info(ctx)[0]) == 6
    assert traj_edit.grip_range(ctx) == [0.0, 0.7]

    urdf = ET.parse(args.isaaclab_repo / "source/isaaclab_hiveboard/isaaclab_hiveboard/assets/anymal/urdf/dynaarm.urdf")
    joints = urdf.findall("joint")
    rng = np.random.default_rng(19)
    worst = 0.0
    for _ in range(20):
        mujoco.mj_resetDataKeyframe(model, data, 0)
        q = rng.uniform(model.jnt_range[:6, 0], model.jnt_range[:6, 1])
        data.qpos[:6] = q
        mujoco.mj_forward(model, data)
        frames = {"arm_mount": transform((0, 0, 0.12), np.diag([-1, -1, 1]))}
        remaining = list(joints)
        while remaining:
            ready = [j for j in remaining if j.find("parent").get("link") in frames]
            assert ready, "URDF contains a disconnected joint tree"
            for joint in ready:
                parent = joint.find("parent").get("link")
                child = joint.find("child").get("link")
                origin = joint.find("origin")
                xyz = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
                r, p, y = np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")
                rot = rotation((0, 0, 1), y) @ rotation((0, 1, 0), p) @ rotation((1, 0, 0), r)
                motion = np.eye(4)
                if joint.get("name") in arm:
                    motion[:3, :3] = rotation(np.fromstring(joint.find("axis").get("xyz"), sep=" "),
                                              q[arm.index(joint.get("name"))])
                frames[child] = frames[parent] @ transform(xyz, rot) @ motion
                remaining.remove(joint)
        base = model.body("base").id
        base_world = transform(data.xpos[base], data.xmat[base].reshape(3, 3))
        for name, expected in frames.items():
            body = model.body(name).id
            actual = np.linalg.inv(base_world) @ transform(data.xpos[body], data.xmat[body].reshape(3, 3))
            worst = max(worst, float(np.max(np.abs(actual - expected))))
            np.testing.assert_allclose(actual, expected, atol=2e-6)
        flange, palm = (model.body(n).id for n in ("dynaarm_flange", "robotiq_base_link"))
        np.testing.assert_allclose(data.xpos[flange], data.xpos[palm], atol=1e-8)
        np.testing.assert_allclose(data.xmat[flange], data.xmat[palm], atol=1e-8)
        tcp = data.xpos[palm] + data.xmat[palm].reshape(3, 3) @ [0, 0, 0.2]
        np.testing.assert_allclose(data.site_xpos[ctx["site"]], tcp, atol=1e-8)

    # Exercise the actual constraints, including the two closed four-bar loops.
    mujoco.mj_resetDataKeyframe(model, data, 0)
    data.ctrl[:6] = ctx["cfg"]["home"]
    for target in (0.0, 0.35, 0.7, 0.0):
        data.ctrl[6] = target
        for _ in range(2000):
            mujoco.mj_step(model, data)
        assert np.isfinite(data.qpos).all() and not data.warning.number.any()
        for joint, multiplier in (("finger_joint", 1), ("right_outer_knuckle_joint", 1),
                                  ("left_inner_finger_joint", -1), ("right_inner_finger_joint", -1)):
            np.testing.assert_allclose(data.joint(joint).qpos, target * multiplier, atol=2e-4)
        eq = data.efc_type == mujoco.mjtConstraint.mjCNSTR_EQUALITY
        assert np.max(np.abs(data.efc_pos[eq])) < 1e-5

    models = REPO / "public/sim/models"
    manifest = json.loads((models / "manifest.json").read_text())
    for name in ["anymal.xml", "anymal.traj.json", "robots.json", "manifest.json"] + [
            f for f in manifest if f.startswith("assets/anymal/")]:
        path = models / name
        assert path.exists()
        assert gzip.decompress(path.with_suffix(path.suffix + ".gz").read_bytes()) == path.read_bytes()
    for mesh in ET.parse(models / "anymal.xml").findall("./asset/mesh"):
        assert "assets/" + mesh.get("file") in manifest
    tasks = json.loads((models / "anymal.traj.json").read_text())
    assert set(tasks) == set(traj_edit.factories(ctx["cfg"]))
    for task in tasks.values():
        assert len(task["qpos"]) == len(task["grip"]) == len(task["tcp"]) > 1
        assert np.asarray(task["qpos"]).shape[1] == 6
        assert np.isfinite(task["qpos"]).all()
    print(f"PASS: 20 URDF FK poses (max matrix error {worst:.2g}), flange/TCP alignment,")
    print("      four gripper commands, loop constraints, editor config, trajectories and gzip/manifest.")


if __name__ == "__main__":
    main()
