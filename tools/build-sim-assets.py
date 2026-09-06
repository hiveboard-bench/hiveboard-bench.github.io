#!/usr/bin/env python3
import argparse
import gzip
import mujoco
import json
import math
import os
import re
import shutil
import fast_simplification
import sim_trajectories
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

REPO = Path(__file__).resolve().parent.parent
OUT = REPO / "public/sim/models"
CACHE = REPO / ".cache"
MENAGERIE_URL = "https://github.com/google-deepmind/mujoco_menagerie.git"

DECIMATE_ABOVE = 3000
KEEP_RATIO = 0.35
BOARD_POS = (0.52, 0.0, 0.2)
BOARD_FLAT = (0.0, math.sqrt(0.5), 0.0, math.sqrt(0.5))
BOARD_UPRIGHT = (0.0, 0.0, 0.0, 1.0)
BOARD_QUAT = BOARD_FLAT

CELLS = [
    (0.0, 0.0),
    (0.086603, 0.0),
    (0.043301, 0.075),
    (-0.043301, 0.075),
    (-0.086603, 0.0),
    (-0.043301, -0.075),
    (0.043301, -0.075),
]


LAY_FLAT = (0.0, math.pi / 2, 0.0)      # plate normal +Z -> +X
LAY_SIDE = (0.0, 0.0, math.pi / 2)      # plate normal -Y -> +X

SPIN = {"stiffness": "0", "damping": "0.02", "armature": "0.0008",
        "limited": "true"}
LOCKED = {"stiffness": "0", "damping": "5", "limited": "true",
          "range": "-0.0005 0.0005"}

MODULES = [
    {"name": "valve", "urdf": "Valves/Lever Valve/Ball Valve/Ball_Valve.urdf", "cell": 2,
     "joints": {"RevoluteJoint": {"stiffness": "0", "damping": "0.1",
                                  "frictionloss": "0.02"}}},
    {"name": "lamp", "urdf": "Lamp/Lamp_Assembly.urdf", "cell": 4,
     "joints": {"PrismaticJoint": {"stiffness": "0", "damping": "1",
                                  "range": "0 0.05"},
                "RevoluteJoint": {"stiffness": "0", "damping": "0.02",
                                  "frictionloss": "0.002", "armature": "0.0005",
                                  "range": "0 6.1", "limited": "true"}}},
    {"name": "breaker", "urdf": "Circuit Breaker/Circuit_Breaker_Assembly.urdf", "cell": 6,
     "joints": {"RevoluteJoint": {"stiffness": "0"}}},

    {"name": "high-valve",
     "urdf": "Valves/Gate Valve/High Torque Valve/High_Torque_Valve.urdf",
     "cell": 0, "align": LAY_FLAT,
     "joints": {"RevoluteJoint": dict(SPIN, range="-6.4 6.4", frictionloss="0.05"),
                "PrismaticJoint": LOCKED}},
    {"name": "small-valve",
     "urdf": "Valves/Gate Valve/Small Valve/Small_Valve.urdf",
     "cell": 0, "align": LAY_SIDE,
     "joints": {"RevoluteJoint": dict(SPIN, range="-6.4 6.4", frictionloss="0.02"),
                "PrismaticJoint": LOCKED}},
    {"name": "thread-m30", "urdf": "Threads/M30 Thread/M30.urdf",
     "cell": 0, "align": LAY_FLAT,
     "joints": {"RevoluteJoint": dict(SPIN, range="0 6.4", frictionloss="0.01"),
                "PrismaticJoint": LOCKED},
     "add": {"Nut": {"name": "RiseJoint", "type": "slide", "axis": "0 0 1",
                     "range": "0 0.04", "damping": "1", "stiffness": "0"}},
     "couple": ("RiseJoint", "RevoluteJoint", 0.0035)},
    {"name": "thread-m8", "urdf": "Threads/M8 Thread/M8_Assy.urdf",
     "cell": 0,
     "joints": {"RevoluteJoint": dict(SPIN, range="0 6.4", frictionloss="0.002"),
                "PrismaticJoint": {"stiffness": "0", "damping": "0.2",
                                   "range": "0 0.02"}},
     "couple": ("PrismaticJoint", "RevoluteJoint", 0.00125)},
    {"name": "peg-insertion", "urdf": "Peg Insertion/Peg_insertion_1.urdf",
     "cell": 0, "align": LAY_FLAT,
     "split": {"child": "peg", "meshes": ["Al__a"],
               "joint": {"name": "PrismaticJoint", "type": "slide",
                         "axis": "0 0 1", "range": "0 0.04",
                         "damping": "1", "stiffness": "0"}}},
    {"name": "button-cover", "urdf": "Button/Button_Assembly.urdf", "cell": 0,
     # the released cover hinges from the far edge; swung round it opens
     # toward the arm instead, leaving the button clear
     "turn": {"lid_pivot": 180, "World": 180},
     "pose": {"RevoluteJoint": -1.5708},
     "joints": {"PrismaticJoint": {"stiffness": "800", "damping": "2",
                                   "range": "-0.01 0"},
                "RevoluteJoint": {"stiffness": "0", "damping": "0.05",
                                  "frictionloss": "0.03", "armature": "0.002"}}},
    {"name": "key-lock", "urdf": "Key/Key_assembly.urdf", "cell": 0,
     "split": {"child": "key",
               "meshes": ["Key_pivot", "Cylinder", "Cube_05"],
               "joints": [
                   {"name": "PrismaticJoint", "type": "slide",
                    "axis": "1 0 0", "range": "0 0.024",
                    "damping": "0.5", "stiffness": "0",
                    "frictionloss": "0.02"},
                   {"name": "RevoluteJoint", "type": "hinge",
                    "axis": "1 0 0", "range": "0 1.5708",
                    "damping": "0.05", "stiffness": "0",
                    "frictionloss": "0.005"}]}},
    {"name": "drawer", "urdf": "Drawer/Drawer_Assembly.urdf", "cell": 0,
     "split": {"child": "drawer",
               "meshes": ["tn__Gaveta1", "Cylinder_02"],
               "joint": {"name": "PrismaticJoint", "type": "slide",
                         "axis": "1 0 0", "range": "0 0.05",
                         "damping": "2", "stiffness": "0"}}},
    {"name": "shock-absorber",
     "urdf": "Shock Absorber/Shock_Absorber_Assembly.urdf",
     "cell": 0, "align": LAY_FLAT,
     "split": {"child": "rod", "meshes": ["Corpo3"],
               "joint": {"name": "PrismaticJoint", "type": "slide",
                         "axis": "1 0 0", "range": "0 0.03",
                         "damping": "2", "stiffness": "120"}}},
]

SWAP_CELL = [m["name"] for m in MODULES if m["cell"] == 0]

PANEL_URDF = "Honeycomb/Honeycomb_Panel.urdf"

ROBOTS = [
    {
        "name": "fr3",
        "label": "Franka FR3",
        "note": "reference arm",
        "arm": [f"fr3_joint{i}" for i in range(1, 8)],
        "grip": {"actuator": "gripper", "open": 0.034, "grasp": 0.002, "fist": 0.0},
        "home": [0.0, -0.0881, 0.0, -2.1491, 0.0, 2.0611, 0.79],
        # held back for now; both show as coming soon
        "skip": ["drawer", "button-cover"],
        # the breaker is thrown for real, not demonstrated
        "physical": ["breaker"],
        # the bulb has to leave its socket, not clear it by a further tenth
        "lamp_tolerance": 1.0,
        "board": (0.52, 0.0, 0.20),
        "bench": {"half": 0.19, "top": 0.20},
        "tcp": ("hand", (0.0, 0.0, 0.1034)),
    },
    {
        "name": "spot",
        "label": "Spot + Spot Arm",
        "note": "Platform A",
        "source": "boston_dynamics_spot/spot_arm.xml",
        "arm": ["arm_sh0", "arm_sh1", "arm_el0", "arm_el1", "arm_wr0", "arm_wr1"],
        "grip": {"actuator": "arm_f1x", "open": -1.5, "grasp": 0.0, "fist": 0.0},
        "jaws": ("arm_link_fngr", "arm_link_wr1"),
        "stow": True,
        "home": [0.0, -1.9, 2.0, 0.0, -0.6, 0.0],
        "board": (1.08, 0.0, 0.70),
        "board_quat": BOARD_UPRIGHT,
        "stand": {"top": 0.70, "half": 0.17},
        "grip_depth": 0.95,
        "framing": "side",
        # Spot keeps the three original modules; the swappable centre cell
        # stays coming soon.  It works them for real rather than
        # demonstrating them, so its modules stay collidable and move only as
        # far as the arm actually pushes them.
        "skip": SWAP_CELL,
        "scripted": False,
        "lamp_standoff": 0.01,
        "lamp_wind": 1.5,
        "lamp_approach": 0.25,
        "lamp_demo": True,
        "valve_grasp": 0.015,
        "valve_shy": -0.04,
    },
    {
        "name": "so101",
        "label": "LeRobot SO-101",
        "note": "Platform B",
        "source": "robotstudio_so101/so101.xml",
        "arm": ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"],
        "grip": {"actuator": "gripper", "open": 1.6, "grasp": -0.175, "fist": -0.175},
        "jaws": ("moving_jaw_so101_v1", "gripper"),
        "home": [0.0, 0.0, 0.0, 0.0, 0.0],
        "ik_home": [0.0, -0.6, 0.9, 0.6, 0.0],
        "board": (0.29, 0.0, 0.01),
        "bench": {"half": 0.17, "top": 0.01},
        "tilt": 1.3,
        "clearance": 0.05,
        "rot_weight": 0.05,
        "grip_depth": 0.85,
        "spin_board": True,
        # The SO-101 is a small 5-DOF arm: it leaves the swappable centre cell
        # alone, and cannot reach the bulb, so the lamp stays coming soon too.
        # What it does keep it works for real rather than demonstrating, so
        # its modules stay collidable and move only as far as it pushes them.
        "skip": SWAP_CELL + ["lamp"],
        "scripted": False,
        "task_spins": {"valve": math.radians(-90), "lamp": math.radians(97)},
        "lamp_grasp_radius": 0.042,
        "lamp_tolerance": 1.0,
        "lamp_ik_tolerance": 0.007,
        "lamp_demo": True,
    },
    {
        "name": "platform_c",
        "label": "ANYmal + DynaArm",
        "note": "Platform C",
        "source": "anybotics_anymal_c/anymal_c_dynaarm.xml",
        "arm": ["dynaarm_shoulder_rotation", "dynaarm_shoulder_flexion",
                "dynaarm_elbow_flexion", "dynaarm_forearm_rotation",
                "dynaarm_wrist_flexion", "dynaarm_wrist_rotation"],
        "grip": {"actuator": "gripper", "open": 0.0, "grasp": 0.04, "fist": 0.04},
        "tcp": ("dynaarm_palm", (0.0, 0.0, 0.09)),
        "stow": True,
        "home": [0.0, 0.8, 1.4, 0.0, 0.6, 0.0, 0.0],
        "board": (0.95, 0.0, 0.85),
        "board_quat": BOARD_UPRIGHT,
        "stand": {"top": 0.85, "half": 0.20},
        "mount": (0.0, 0.0, 0.0),
        "mount_quat": (1.0, 0.0, 0.0, 0.0),
        "framing": "side",
        "skip": [mod["name"] for mod in MODULES],
        # No solved trajectories yet -- a distinct arm rest pose per task so
        # each reads as addressing its own cell. 6 DynaArm joints, no gripper.
        "task_home": {
            "valve":   [0.18, 0.74, 1.30, 0.0, 0.78, 0.0],
            "lamp":    [-0.20, 0.92, 1.36, 0.0, 0.50, 0.0],
            "breaker": [0.12, 0.99, 1.18, 0.0, 0.44, 0.0],
        },
    },
    {
        "name": "macao", "label": "Macao hand", "note": "Platform D",
        "arm": ["macao_x", "macao_y", "macao_z", "macao_roll", "macao_pitch", "macao_yaw"],
        "grip": {"actuator": "macao_grip", "open": 0.0, "grasp": 0.8, "fist": 1.1},
        "home": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "board": (0.85, 0.0, 0.55),
        "board_quat": BOARD_UPRIGHT,
        "stand": {"top": 0.55, "half": 0.17},
        "framing": "side",
        "mount": (0.70, 0.0, 0.52),
        "tcp": ("macao_hand", (0.0, 0.0, 0.11)),
        "skip": ["toggle", "button", "dial"],
    },
]
URDF_HINT = (
    '<mujoco><compiler meshdir="." balanceinertia="true" '
    'discardvisual="false" strippath="false" fusestatic="false" '
    'inertiagrouprange="0 0"/></mujoco>'
)


def obj_read(path):

    verts, faces = [], []
    for line in path.read_text().splitlines():
        if line.startswith("v "):
            verts.append([float(x) for x in line.split()[1:4]])
        elif line.startswith("f "):
            idx = [int(tok.split("/")[0]) for tok in line.split()[1:]]
            idx = [i - 1 if i > 0 else len(verts) + i for i in idx]
            for k in range(1, len(idx) - 1):
                faces.append([idx[0], idx[k], idx[k + 1]])
    return np.asarray(verts, np.float64), np.asarray(faces, np.int64)


def obj_write(path, verts, faces):

    out = ["v %.6g %.6g %.6g" % tuple(v) for v in verts]
    out += ["f %d %d %d" % tuple(f + 1) for f in faces]
    path.write_text("\n".join(out) + "\n")


def weld(verts, faces, tol=1e-7):

    keys = np.round(verts / tol).astype(np.int64)
    _, first, inverse = np.unique(keys, axis=0, return_index=True, return_inverse=True)
    order = np.argsort(first)
    remap = np.empty(len(first), np.int64)
    remap[order] = np.arange(len(first))
    faces = remap[inverse.ravel()][faces]
    verts = verts[first[order]]
    keep = (faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2]) & (faces[:, 0] != faces[:, 2])
    return verts, faces[keep]


def decimate(verts, faces):

    verts, faces = weld(verts, faces)
    if len(faces) <= DECIMATE_ABOVE:
        return verts, faces
    

    v, f = fast_simplification.simplify(
        verts.astype(np.float32), faces.astype(np.int32), 1.0 - KEEP_RATIO
    )
    return np.asarray(v, np.float64), np.asarray(f, np.int64)


def stl_read(path):

    raw = path.read_bytes()
    if raw[:5].lower() == b"solid" and b"facet" in raw[:512]:
        verts = [[float(v) for v in line.split()[1:4]]
                 for line in raw.decode("utf-8", "replace").splitlines()
                 if line.strip().startswith("vertex")]
        verts = np.asarray(verts, np.float64)
    else:
        count = int.from_bytes(raw[80:84], "little")
        block = np.frombuffer(raw[84:84 + count * 50], dtype=np.uint8).reshape(count, 50)
        verts = block[:, 12:48].copy().view(np.float32).reshape(-1, 3).astype(np.float64)
    faces = np.arange(len(verts), dtype=np.int64).reshape(-1, 3)
    return verts, faces


def emit_mesh(src: Path, dst_dir: Path, name: str = None, simplify=True) -> str:

    dst_dir.mkdir(parents=True, exist_ok=True)
    stl = src.suffix.lower() == ".stl"
    verts, faces = (stl_read(src) if stl else obj_read(src))
    if simplify:
        verts, faces = decimate(verts, faces)
    out = name or (src.stem + ".obj")
    obj_write(dst_dir / out, verts, faces)
    return out


DYNAARM_LINKS = [

    ("dynaarm_base", "base", None),
    ("dynaarm_shoulder", "shoulder",
     ("dynaarm_shoulder_rotation", (0, 0, 0.1105), (0, 0, 0), (-4.3124, 4.3124))),
    ("dynaarm_upperarm", "upperarm",
     ("dynaarm_shoulder_flexion", (0, 0, 0.047), (1.570796, -1.570796, 0),
      (-1.7208, 1.7208))),
    ("dynaarm_elbow", "elbow",
     ("dynaarm_elbow_flexion", (0.4127, 0, 0), (0, 0, 0), (0.0, 3.09159))),
    ("dynaarm_forearm", "forearm",
     ("dynaarm_forearm_rotation", (0.0262, -0.0855, 0),
      (1.570796, 1.570796, 1.570796), (-4.7124, 4.7124))),
    ("dynaarm_wrist_1", "wrist1",
     ("dynaarm_wrist_flexion", (0.0295, 0, 0.4207), (0, -1.570796, 0),
      (-1.8208, 1.8208))),
    ("dynaarm_wrist_2", "wrist2",
     ("dynaarm_wrist_rotation", (0.117, 0.0, 0.0295),
      (-1.570796, 1.570796, -1.570796), (-1.570796, 4.7124))),
]

DYNAARM_STAND = {
    "dynaarm_shoulder_rotation": 0.0, "dynaarm_shoulder_flexion": -0.7,
    "dynaarm_elbow_flexion": 1.4, "dynaarm_forearm_rotation": 0.0,
    "dynaarm_wrist_flexion": 0.0, "dynaarm_wrist_rotation": 0.0,
}


ANYMAL_C_STAND = {
    "LF_HAA": 0.03, "LF_HFE": 0.4, "LF_KFE": -0.8,
    "RF_HAA": -0.03, "RF_HFE": 0.4, "RF_KFE": -0.8,
    "LH_HAA": -0.03, "LH_HFE": -0.4, "LH_KFE": 0.8,
    "RH_HAA": 0.03, "RH_HFE": -0.4, "RH_KFE": 0.8,
}


def _rpy_quat(rpy):

    r, p, y = rpy
    m = axis_matrix((0, 0, 1), y) @ axis_matrix((0, 1, 0), p) @ axis_matrix((1, 0, 0), r)
    return matrix_quat(m)


def prepare_platform_c(menagerie_dir: Path):

    anymal_dir = menagerie_dir / "anybotics_anymal_c"
    src = anymal_dir / "anymal_c.xml"
    out_xml = anymal_dir / "anymal_c_dynaarm.xml"
    if not src.exists():
        return
    for obj in (REPO / "tools/assets/anymal").glob("dynaarm_*.obj"):
        shutil.copy(obj, anymal_dir / "assets" / obj.name)

    tree = ET.parse(src)
    root = tree.getroot()
    asset = root.find("asset")

    ANYMAL_RGBA = {
        "base": "0.14 0.15 0.17 1", "top_shell": "0.72 0.055 0.065 1",
        "bottom_shell": "0.20 0.22 0.24 1", "hip_l": "0.14 0.15 0.17 1",
        "hip_r": "0.14 0.15 0.17 1", "thigh": "0.12 0.13 0.15 1",
        "shank": "0.14 0.15 0.17 1", "shank_l": "0.14 0.15 0.17 1",
        "foot": "0.08 0.09 0.10 1", "hatch": "0.72 0.055 0.065 1",
        "remote": "0.14 0.15 0.17 1", "handle": "0.45 0.48 0.52 1",
        "face": "0.12 0.13 0.14 1", "depth_camera": "0.08 0.08 0.08 1",
        "wide_angle_camera": "0.08 0.08 0.08 1", "battery": "0.14 0.15 0.17 1",
        "lidar_cage": "0.08 0.08 0.08 1", "lidar": "0.08 0.08 0.08 1",
        "drive": "0.72 0.055 0.065 1",
    }
    for tex in asset.findall("texture"):
        asset.remove(tex)
    for mat in root.findall(".//material"):
        mat.attrib.pop("texture", None)
        if mat.get("name") in ANYMAL_RGBA:
            mat.set("rgba", ANYMAL_RGBA[mat.get("name")])
        mat.attrib.setdefault("rgba", "0.5 0.5 0.5 1")
    DYNAARM_PARTS = {
        "base": [("dynaarm_base_dark", "arm_dark"), ("dynaarm_base_metal", "arm_metal")],
        "shoulder": [("dynaarm_shoulder_dark", "arm_dark"), ("dynaarm_shoulder_metal", "arm_metal")],
        "upperarm": [("dynaarm_upperarm_dark", "arm_carbon"), ("dynaarm_upperarm_metal", "arm_metal")],
        "elbow": [("dynaarm_elbow_dark", "arm_dark"), ("dynaarm_elbow_metal", "arm_metal")],
        "forearm": [("dynaarm_forearm_dark", "arm_carbon"), ("dynaarm_forearm_metal", "arm_metal")],
        "wrist1": [("dynaarm_wrist1_dark", "arm_dark"), ("dynaarm_wrist1_metal", "arm_metal")],
        "wrist2": [("dynaarm_wrist2_metal", "arm_metal")],
    }
    for parts in DYNAARM_PARTS.values():
        for mesh_name, _ in parts:
            if root.find(f'.//mesh[@name="{mesh_name}"]') is None:
                ET.SubElement(asset, "mesh",
                              {"name": mesh_name, "file": f"{mesh_name}.obj"})
    for name, rgba in (("arm_carbon", "0.14 0.15 0.17 1"),
                       ("arm_metal", "0.78 0.80 0.84 1"),
                       ("arm_dark", "0.15 0.16 0.18 1"),
                       ("arm_accent", "0.93 0.69 0.10 1"),
                       ("arm_red", "0.72 0.055 0.065 1")):
        if root.find(f'.//material[@name="{name}"]') is None:
            ET.SubElement(asset, "material", {"name": name, "rgba": rgba})

    base = root.find('.//body[@name="base"]')
    parent = ET.SubElement(base, "body",
                           {"name": DYNAARM_LINKS[0][0], "pos": "0.12 0 0.08",
                            "quat": "0 0 0 1"})
    for mesh_name, mat in DYNAARM_PARTS["base"]:
        ET.SubElement(parent, "geom", {"type": "mesh", "mesh": mesh_name,
                                       "material": mat, "class": "visual"})
    ET.SubElement(parent, "geom", {"type": "cylinder", "size": "0.06 0.05",
                                   "pos": "0 0 0.05", "class": "collision"})
    arm_bodies = [DYNAARM_LINKS[0][0]]
    for link_name, stem, joint in DYNAARM_LINKS[1:]:
        jn, xyz, rpy, jr = joint
        child = ET.SubElement(parent, "body", {
            "name": link_name, "pos": fmt(xyz), "quat": fmt(_rpy_quat(rpy))})
        ET.SubElement(child, "joint", {
            "name": jn, "axis": "0 0 1", "range": fmt(jr),
            "damping": "3", "armature": "0.1"})
        for mesh_name, mat in DYNAARM_PARTS.get(stem, []):
            ET.SubElement(child, "geom", {"type": "mesh", "mesh": mesh_name,
                                          "material": mat, "class": "visual"})
        ET.SubElement(child, "geom", {"type": "capsule", "size": "0.045 0.06",
                                      "class": "collision"})
        parent = child
        arm_bodies.append(link_name)

    flange = ET.SubElement(parent, "body",
                           {"name": "dynaarm_flange", "pos": "0 0 0.009",
                            "quat": fmt(_rpy_quat((0, 0, 1.570796)))})
    ET.SubElement(flange, "geom", {"type": "cylinder", "size": "0.042 0.004",
                                   "material": "arm_metal", "class": "visual"})
    palm = ET.SubElement(flange, "body", {"name": "dynaarm_palm", "pos": "0 0 0.02"})
    ET.SubElement(palm, "geom", {"type": "box", "size": "0.038 0.042 0.016",
                                 "material": "arm_dark", "class": "visual"})
    ET.SubElement(palm, "geom", {"type": "box", "size": "0.032 0.035 0.006", "pos": "0 0 0.012",
                                 "material": "arm_metal", "class": "visual"})
    ET.SubElement(palm, "geom", {"type": "box", "size": "0.045 0.05 0.02",
                                 "class": "collision"})
    jaw_l = ET.SubElement(palm, "body", {"name": "dynaarm_jaw", "pos": "0 0.03 0.055"})
    ET.SubElement(jaw_l, "joint", {"name": "gripper", "type": "slide", "axis": "0 -1 0",
                                   "range": "0 0.04", "damping": "8"})
    ET.SubElement(jaw_l, "geom", {"type": "box", "size": "0.012 0.008 0.042",
                                  "material": "arm_dark", "class": "visual"})
    ET.SubElement(jaw_l, "geom", {"type": "box", "size": "0.010 0.003 0.038", "pos": "0 -0.006 0",
                                  "material": "arm_accent", "class": "visual"})
    ET.SubElement(jaw_l, "geom", {"type": "box", "size": "0.008 0.01 0.045",
                                  "class": "collision"})
    jaw_r = ET.SubElement(palm, "body", {"name": "dynaarm_jaw_fixed", "pos": "0 -0.03 0.055"})
    ET.SubElement(jaw_r, "geom", {"type": "box", "size": "0.012 0.008 0.042",
                                  "material": "arm_dark", "class": "visual"})
    ET.SubElement(jaw_r, "geom", {"type": "box", "size": "0.010 0.003 0.038", "pos": "0 0.006 0",
                                  "material": "arm_accent", "class": "visual"})
    ET.SubElement(jaw_r, "geom", {"type": "box", "size": "0.008 0.01 0.045",
                                  "class": "collision"})

    actuator = root.find("actuator")
    if actuator is None:
        actuator = ET.SubElement(root, "actuator")
    for _, _, joint in DYNAARM_LINKS[1:]:
        ET.SubElement(actuator, "position",
                      {"name": joint[0], "joint": joint[0], "kp": "150"})
    ET.SubElement(actuator, "position", {"name": "gripper", "joint": "gripper",
                                         "kp": "200", "ctrlrange": "0 0.04"})

    contact = root.find("contact")
    if contact is None:
        contact = ET.SubElement(root, "contact")
    chain = ["base"] + arm_bodies + ["dynaarm_flange", "dynaarm_palm",
                                     "dynaarm_jaw", "dynaarm_jaw_fixed"]
    for a, b in zip(chain, chain[1:]):
        ET.SubElement(contact, "exclude", {"body1": a, "body2": b})
    ET.SubElement(contact, "exclude", {"body1": "base", "body2": "dynaarm_shoulder"})

    for kf in root.findall("keyframe"):
        root.remove(kf)
    tree.write(str(out_xml))

    m = mujoco.MjModel.from_xml_path(str(out_xml))
    stance = {**ANYMAL_C_STAND, **DYNAARM_STAND, "gripper": 0.0}
    qpos = np.zeros(m.nq)
    for j in range(m.njnt):
        name = m.joint(j).name
        adr = m.jnt_qposadr[j]
        if m.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE:
            qpos[adr:adr + 7] = (0.0, 0.0, 0.62, 1.0, 0.0, 0.0, 0.0)
        else:
            qpos[adr] = stance.get(name, 0.0)
    kf = ET.SubElement(root, "keyframe")
    ET.SubElement(kf, "key", {"name": "home", "qpos": fmt(qpos)})
    tree.write(str(out_xml))
    print(f"  platform_c source: {out_xml.name} (nq={m.nq})")


def ensure_menagerie() -> Path:

    root = CACHE / "mujoco_menagerie"
    if not (root / "franka_fr3/fr3.xml").exists():
        CACHE.mkdir(exist_ok=True)
        shutil.rmtree(root, ignore_errors=True)
        subprocess.run(
            ["git", "clone", "--depth", "1", "--filter=blob:none", "--sparse",
             MENAGERIE_URL, str(root)],
            check=True,
        )
        subprocess.run(
            ["git", "sparse-checkout", "set", "franka_fr3", "franka_emika_panda",
             "boston_dynamics_spot", "robotstudio_so101", "anybotics_anymal_c"],
            cwd=root, check=True,
        )
    prepare_platform_c(root)
    return root


def flat_meshes(mesh_dir: Path):

    out = set()
    for obj in sorted(mesh_dir.glob("*.obj")):
        verts = [[float(x) for x in line.split()[1:4]]
                 for line in obj.read_text().splitlines() if line.startswith("v ")]
        if not verts or np.ptp(np.array(verts), axis=0).min() < 1e-6:
            out.add(obj.name)
    return out


def load_urdf(urdf: Path, workdir: Path, align=None):

    staged = workdir / urdf.parent.name
    if not staged.exists():
        shutil.copytree(urdf.parent, staged)
    path = staged / urdf.name
    text = path.read_text()
    if "<mujoco>" not in text:
        for name in flat_meshes(staged / "meshes") if (staged / "meshes").is_dir() else ():
            text = re.sub(r"\s*<(visual|collision)>(?:(?!</\1>).)*?"
                          + re.escape(name) + r"(?:(?!</\1>).)*?</\1>",
                          "", text, flags=re.S)
        text = re.sub(r"(<robot[^>]*>)", r"\1\n  " + URDF_HINT, text, count=1)
        path.write_text(text)

    model = mujoco.MjModel.from_xml_path(str(path))
    saved = workdir / (urdf.stem + ".mjcf.xml")
    mujoco.mj_saveLastXML(str(saved), model)
    rot = rpy_matrix(align) if align else np.eye(3)
    return (ET.parse(saved).getroot(), urdf.parent / "meshes",
            base_depth(model, rot), reach_direction(model, rot))


def rpy_matrix(rpy):

    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
            @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def adopt_robot(cfg, menagerie: Path, workdir: Path, out_assets: Path):
    src = menagerie / cfg["source"]
    model = mujoco.MjModel.from_xml_path(str(src))
    saved = workdir / f"{cfg['name']}.mjcf.xml"
    mujoco.mj_saveLastXML(str(saved), model)
    root = ET.parse(saved).getroot()

    meshes = []
    for mesh in root.findall("./asset/mesh"):
        fname = Path(mesh.get("file")).name
        out = emit_mesh(src.parent / "assets" / fname, out_assets)
        mesh.set("file", f"{cfg['name']}/{out}")
        mesh.attrib.pop("content_type", None)
        meshes.append(mesh)

    body = root.find("./worldbody/body")
    body.set("pos", fmt(cfg.get("mount", (0, 0, 0))))
    if cfg.get("mount_quat"):
        body.set("quat", fmt(cfg["mount_quat"]))

    extras = {tag: root.find(tag) for tag in
              ("actuator", "contact", "equality", "tendon", "default")}
    return body, meshes, root.findall("./asset/material"), extras


def quat_matrix(q):

    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_quat(m):

    w = math.sqrt(max(1 + m[0, 0] + m[1, 1] + m[2, 2], 0)) / 2
    if w > 1e-6:
        return np.array([w, (m[2, 1] - m[1, 2]) / (4 * w),
                         (m[0, 2] - m[2, 0]) / (4 * w),
                         (m[1, 0] - m[0, 1]) / (4 * w)])
    i = int(np.argmax([m[0, 0], m[1, 1], m[2, 2]]))
    j, k = (i + 1) % 3, (i + 2) % 3
    r = math.sqrt(max(1 + m[i, i] - m[j, j] - m[k, k], 0))
    q = np.zeros(4)
    q[0] = (m[k, j] - m[j, k]) / (2 * r)
    q[i + 1], q[j + 1], q[k + 1] = r / 2, (m[j, i] + m[i, j]) / (2 * r), (m[k, i] + m[i, k]) / (2 * r)
    return q


def axis_matrix(axis, angle):

    k = np.asarray(axis, float)
    k = k / np.linalg.norm(k)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * (K @ K)


def freeze_joints(body, keep, pose):

    dropped = []
    for parent in body.iter("body"):
        for joint in list(parent.findall("joint")) + list(parent.findall("freejoint")):
            name = joint.get("name")
            if name in keep:
                continue
            parent.remove(joint)
            dropped.append(name or "<free>")

            angle = pose.get(name)
            if angle is None or joint.tag == "freejoint":
                continue

            pos = np.array([float(v) for v in (parent.get("pos") or "0 0 0").split()])
            quat = np.array([float(v) for v in (parent.get("quat") or "1 0 0 0").split()])
            rot = quat_matrix(quat)
            anchor_local = np.array([float(v) for v in (joint.get("pos") or "0 0 0").split()])
            axis_local = np.array([float(v) for v in (joint.get("axis") or "0 0 1").split()])
            spin = axis_matrix(rot @ axis_local, angle)
            offset = rot @ anchor_local
            parent.set("pos", fmt(pos + offset - spin @ offset))
            parent.set("quat", fmt(matrix_quat(spin @ rot)))
    return dropped


def reach_direction(model, rot=None) -> tuple:
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    rot = np.eye(3) if rot is None else rot
    total = np.zeros(2)
    weight = 0.0
    for g in range(model.ngeom):
        body = model.geom_bodyid[g]
        if body <= 1 or model.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        mesh = model.geom_dataid[g]
        adr, num = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
        world = model.mesh_vert[adr:adr + num] @ data.geom_xmat[g].reshape(3, 3).T
        world = (world + data.geom_xpos[g]) @ rot.T
        total += world[:, 1:].mean(axis=0) * num
        weight += num

    if weight == 0:
        return (0.0, 0.0)
    vec = total / weight
    norm = float(np.linalg.norm(vec))
    return tuple(vec / norm) if norm > 1e-6 else (0.0, 0.0)


def base_depth(model, rot=None) -> float:
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    rot = np.eye(3) if rot is None else rot
    lo = 0.0
    for g in range(model.ngeom):
        if model.geom_bodyid[g] != 1 or model.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        mesh = model.geom_dataid[g]
        adr, num = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
        world = model.mesh_vert[adr:adr + num] @ data.geom_xmat[g].reshape(3, 3).T
        lo = min(lo, float(((world + data.geom_xpos[g]) @ rot.T)[:, 0].min()))
    return -lo


def split_body(root, prefix, spec):

    base = root
    wanted = tuple(f"{prefix}_{name}" for name in spec["meshes"])
    moving = [g for g in base.findall("geom")
              if (g.get("mesh") or "").startswith(wanted)]
    if not moving:
        raise SystemExit(f"split: no geom matched {wanted}")

    child = ET.SubElement(base, "body",
                          {"name": f"{prefix}_{spec['child']}", "pos": "0 0 0"})
    for joint in spec.get("joints", [spec["joint"]] if "joint" in spec else []):
        ET.SubElement(child, "joint", dict(
            joint, name=f"{prefix}_{joint['name']}",
            pos=spec.get("pos", "0 0 0")))
    for geom in moving:
        base.remove(geom)
        child.append(geom)
        geom.set("material", "hb_accent")
        if geom.get("group") == "3":
            geom.set("contype", "2")
            geom.set("conaffinity", "1")
            geom.set("solref", "0.004 1")
            geom.set("solimp", "0.98 0.999 0.0005")
    for body, mass, inertia in ((base, "0.4", "1e-3"), (child, "0.01", "1e-5")):
        for old in body.findall("inertial"):
            body.remove(old)
        ET.SubElement(body, "inertial", {
            "pos": "0 0 0", "mass": mass,
            "diaginertia": " ".join([inertia] * 3)})


def turn_child(root, prefix, spec):

    # Rotate part of a module about its mounting normal (local +X).  Naming a
    # child body carries its joint anchor and axis round with it -- enough to
    # hinge a cover from the opposite edge.  Naming the module's own root
    # spins just that body's geometry, leaving the parts hung off it where
    # they are, so the housing can turn without moving the button or the lid.
    for name, degrees in spec.items():
        body = next((b for b in root.iter("body")
                     if b.get("name") == f"{prefix}_{name}"), None)
        if body is None:
            raise SystemExit(f"turn: no body named {prefix}_{name}")
        rot = rpy_matrix((math.radians(degrees), 0.0, 0.0))
        targets = body.findall("geom") if body is root else [body]
        for elem in targets:
            pos = [float(v) for v in (elem.get("pos") or "0 0 0").split()]
            quat = [float(v) for v in (elem.get("quat") or "1 0 0 0").split()]
            elem.set("pos", fmt(rot @ np.array(pos)))
            elem.set("quat", fmt(matrix_quat(rot @ quat_matrix(quat))))


def add_joints(root, prefix, spec):

    for body_name, joint in spec.items():
        for body in root.iter("body"):
            if body.get("name") == f"{prefix}_{body_name}":
                body.insert(0, ET.Element("joint", dict(
                    joint, name=f"{prefix}_{joint['name']}")))
                break
        else:
            raise SystemExit(f"add: no body named {prefix}_{body_name}")


def adopt(root, prefix, mesh_src: Path, meshes: dict, shell="hb_shell", pose=None, tune=None):

    mesh_elems = root.findall("./asset/mesh")
    for mesh in mesh_elems:
        fname = Path(mesh.get("file")).name
        out_name = f"{prefix}_{Path(fname).stem}.obj"
        meshes[f"{prefix}_{mesh.get('name')}"] = (mesh_src / fname, out_name)
        mesh.set("name", f"{prefix}_{mesh.get('name')}")
        mesh.set("file", f"hb/{out_name}")
        mesh.attrib.pop("content_type", None)

    base = root.find("./worldbody/body")
    for body in root.iter("body"):
        material = shell if body is base else "hb_accent"
        for geom in body.findall("geom"):
            if geom.get("mesh"):
                geom.set("mesh", f"{prefix}_{geom.get('mesh')}")
            visual = geom.get("group") == "1"
            geom.set("group", "2" if visual else "3")
            geom.set("material", material)
            if not visual:
                geom.set("rgba", "0 0 0 0")
                geom.set("friction", "1.6 0.02 0.001")
                if body is base:
                    geom.set("contype", "0")
                    geom.set("conaffinity", "0")
                else:
                    geom.set("contype", "2")
                    geom.set("conaffinity", "1")
        body.set("name", f"{prefix}_{body.get('name')}")

    for joint in root.iter("joint"):
        original = joint.get("name")
        rest = (pose or {}).get(original, 0.0)
        if original:
            joint.set("name", f"{prefix}_{original}")
        slide = joint.get("type") == "slide"
        joint.set("springref", "%.6g" % rest)
        joint.set("stiffness", "400" if slide else "0.35")
        joint.set("damping", "4" if slide else "0.06")
        joint.set("armature", "0.01" if slide else "0.002")
        joint.set("frictionloss", "0.01")
        for attr, value in (tune or {}).get(original, {}).items():
            joint.set(attr, value)

    return base, mesh_elems


def cell_pose(cell, lift, reach):

    y, z = CELLS[cell]
    want = (y, z) if (y or z) else (0.0, -1.0)
    norm = math.hypot(*want)
    want = (want[0] / norm, want[1] / norm)

    if reach == (0.0, 0.0):
        spin = 0.0
    else:
        spin = math.atan2(reach[0] * want[1] - reach[1] * want[0],
                          reach[0] * want[0] + reach[1] * want[1])

    half = spin / 2.0
    return (lift, y, z), (math.cos(half), math.sin(half), 0.0, 0.0)


def fmt(vals):

    return " ".join("%.6g" % v for v in vals)


def home_qpos(cfg, scene, path):

    rest = {f"{mod['name']}_{joint}": value
            for mod in MODULES for joint, value in mod.get("pose", {}).items()}
    if not rest:
        return cfg["home"]

    ET.indent(scene, "  ")
    path.write_text(ET.tostring(scene, encoding="unicode") + "\n")
    model = mujoco.MjModel.from_xml_path(str(path))
    qpos = model.qpos0.copy()
    qpos[:len(cfg["home"])] = cfg["home"]
    for name, value in rest.items():
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid >= 0:
            qpos[model.jnt_qposadr[jid]] = value
    return qpos


def build_board(hiveboard: Path):

    meshes, elems, fragments = {}, [], []
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        panel_root, panel_meshes, _, _ = load_urdf(hiveboard / PANEL_URDF, tmp)
        panel_body, panel_elems = adopt(panel_root, "panel", panel_meshes, meshes,
                                        shell="hb_panel")
        panel_body.set("pos", "0 0 0")
        fragments.append(panel_body)
        elems += panel_elems

        for mod in MODULES:
            align = mod.get("align")
            root, mesh_dir, lift, reach = load_urdf(hiveboard / mod["urdf"], tmp,
                                                    align)
            body, mod_elems = adopt(root, mod["name"], mesh_dir, meshes,
                                    pose=mod.get("pose"), tune=mod.get("joints"))
            if "split" in mod:
                split_body(body, mod["name"], mod["split"])
            if "add" in mod:
                add_joints(body, mod["name"], mod["add"])
            if "turn" in mod:
                turn_child(body, mod["name"], mod["turn"])
            pos, quat = cell_pose(mod["cell"], lift, reach)
            if align:
                quat = matrix_quat(quat_matrix(quat) @ rpy_matrix(align))
            body.set("pos", fmt(pos))
            body.set("quat", fmt(quat))
            fragments.append(body)
            elems += mod_elems

    for name, (src, out_name) in meshes.items():
        emit_mesh(src, OUT / "assets/hb", out_name)

    threads = [("lamp", "PrismaticJoint", "RevoluteJoint",
                sim_trajectories.LAMP_PITCH)]
    threads += [(m["name"], *m["couple"]) for m in MODULES if "couple" in m]
    equality = [ET.Element("joint", {
        "joint1": f"{name}_{slide}", "joint2": f"{name}_{turn}",
        "polycoef": "0 %.6g 0 0 0" % pitch,
        "solref": "0.01 1", "solimp": "0.95 0.999 0.001"})
        for name, slide, turn, pitch in threads]
    return {"fragments": fragments, "meshes": elems, "equality": equality}


def fr3_parts(menagerie: Path):

    fr3, panda = menagerie / "franka_fr3", menagerie / "franka_emika_panda"
    visual = (
        [f"link0_{i}.obj" for i in range(7)]
        + ["link1.obj", "link2.obj", "link3_0.obj", "link3_1.obj",
           "link4_0.obj", "link4_1.obj", "link5_0.obj", "link5_1.obj", "link5_2.obj"]
        + [f"link6_{i}.obj" for i in range(8)]
        + [f"link7_{i}.obj" for i in range(4)]
    )
    collision = [f"link{i}.stl" for i in range(8)]
    hand = [f"hand_{i}.obj" for i in range(5)] + ["finger_0.obj", "finger_1.obj"]

    for f in visual + collision:
        emit_mesh(fr3 / "assets" / f, OUT / "assets/fr3")
    for f in hand + ["hand.stl"]:
        emit_mesh(panda / "assets" / f, OUT / "assets/fr3")

    elems = [ET.Element("mesh", {"file": f"fr3/{f}"}) for f in visual + hand]
    elems += [ET.Element("mesh", {"name": Path(f).stem + "_coll",
                                  "file": f"fr3/{Path(f).stem}.obj"}) for f in collision]
    elems.append(ET.Element("mesh", {"name": "hand_c", "file": "fr3/hand.obj"}))

    extras = {"default": ET.fromstring(FR3_DEFAULTS),
              "actuator": ET.fromstring(FR3_ACTUATORS),
              "equality": ET.fromstring(FR3_EQUALITY),
              "contact": ET.fromstring(
                  '<contact><exclude body1="fr3_link0" body2="fr3_link1"/></contact>'),
              "tendon": None}
    return ET.fromstring(ARM_BODY), elems, [], extras


def macao_parts(cfg):

    src = REPO / "tools/assets/macao"
    names = {
        "forearm": "Short Forearm.stl",
        "forearm_base": "Short Forearm Base Lid.stl",
        "forearm_lid": "Short Forearm Arduino Cavity Lid.stl",
        "palm_middle": "HandPalm Middle Segment.stl",
        "palm_outer": "HandPalm Outer Segment.stl",
        "palm_cover": "Palm Cover.stl",
        "finger_base": "Finger Base.stl",
        "finger_first": "Finger First Phalange.stl",
        "finger_first_pad": "Finger First Pad.stl",
        "finger_medial": "Finger Medial Phalange.stl",
        "finger_medial_pad": "Finger Medial Pad.stl",
        "finger_distal": "Finger Distal Phalange.stl",
        "finger_distal_pad": "Finger Distal Pad.stl",
    }
    meshes = []

    def add_mesh(name, filename):

        out = emit_mesh(src / filename, OUT / "assets/macao", name=f"{name}.obj")
        meshes.append(ET.Element("mesh", {
            "name": name, "file": f"macao/{out}", "scale": "0.001 0.001 0.001"}))

    for key, filename in names.items():
        add_mesh(key, filename)

    def visual(parent, mesh, material="macao_shell", pos=None, collision=False):

        attrs = {
            "type": "mesh", "mesh": mesh, "material": material,
            "contype": "1" if collision else "0",
            "conaffinity": "2" if collision else "0", "group": "2"}
        if pos:
            attrs["pos"] = pos
        ET.SubElement(parent, "geom", attrs)

    body = ET.Element("body", {"name": "macao_hand", "pos": fmt(cfg.get("mount", (0.15, 0.0, 0.52))),
                                "quat": "0.707107 0 0.707107 0", "gravcomp": "1"})
    for name, kind, axis, limits in (
        ("macao_x", "slide", "1 0 0", "-0.12 0.12"),
        ("macao_y", "slide", "0 1 0", "-0.10 0.10"),
        ("macao_z", "slide", "0 0 1", "-0.10 0.10"),
        ("macao_roll", "hinge", "1 0 0", "-0.6 0.6"),
        ("macao_pitch", "hinge", "0 1 0", "-0.6 0.6"),
        ("macao_yaw", "hinge", "0 0 1", "-0.8 0.8"),
    ):
        ET.SubElement(body, "joint", {"name": name, "type": kind, "axis": axis,
                                       "range": limits, "damping": "2"})

    shell = ET.SubElement(body, "body", {
        "name": "macao_arm_wrist", "quat": "0 0 0 1"})

    visual(shell, "forearm", "macao_wrist")
    visual(shell, "forearm_base", "macao_base")
    for key in ("palm_middle", "palm_outer", "palm_cover", "forearm_lid"):
        visual(shell, key, "macao_shell")

    finger_joints = []

    def digit(parent, prefix, master=False):

        visual(parent, "finger_base", collision=True)

        proximal = ET.SubElement(parent, "body", {"name": f"{prefix}_proximal", "pos": "0 0 0.016"})
        proximal_joint = "macao_grip" if master else f"{prefix}_prox_joint"
        ET.SubElement(proximal, "joint", {"name": proximal_joint, "type": "hinge",
                                           "axis": "-1 0 0", "range": "0 1.15", "damping": "0.35"})
        finger_joints.append((proximal_joint, 1.0))
        visual(proximal, "finger_first", pos="0 0 -0.016", collision=True)
        visual(proximal, "finger_first_pad", "macao_pad", "0 0 -0.016", collision=True)

        medial = ET.SubElement(proximal, "body", {"name": f"{prefix}_medial", "pos": "0 0 0.030"})
        medial_joint = f"{prefix}_medial_joint"
        ET.SubElement(medial, "joint", {"name": medial_joint, "type": "hinge",
                                         "axis": "-1 0 0", "range": "0 1.0", "damping": "0.3"})
        finger_joints.append((medial_joint, 0.78))
        visual(medial, "finger_medial", pos="0 0 -0.046", collision=True)
        visual(medial, "finger_medial_pad", "macao_pad", "0 0 -0.046", collision=True)

        distal = ET.SubElement(medial, "body", {"name": f"{prefix}_distal", "pos": "0 0 0.0245"})
        distal_joint = f"{prefix}_distal_joint"
        ET.SubElement(distal, "joint", {"name": distal_joint, "type": "hinge",
                                         "axis": "-1 0 0", "range": "0 0.9", "damping": "0.25"})
        finger_joints.append((distal_joint, 0.65))
        visual(distal, "finger_distal", pos="0 0 -0.0705", collision=True)
        visual(distal, "finger_distal_pad", "macao_pad", "0 0 -0.0705", collision=True)

    finger_height_offsets = (-0.02, -0.005, 0.005, -0.015)
    for i, x in enumerate((-0.033, -0.011, 0.011, 0.033)):
        mount = ET.SubElement(body, "body", {
            "name": f"macao_finger_body_{i}",
            "pos": f"{x} 0.01 {finger_height_offsets[i] + 0.04}"})
        digit(mount, f"macao_finger_{i}", master=(i == 0))

    thumb = ET.SubElement(body, "body", {
        "name": "macao_thumb_body", "pos": "0.018 0.063 -0.005",
        "quat": "0 0 0.573576 0.819152"})
    digit(thumb, "macao_thumb")

    materials = [
        ET.Element("material", {"name": "macao_shell", "rgba": "0.08 0.09 0.11 1"}),
        ET.Element("material", {"name": "macao_base", "rgba": "0.7059 0.7804 0.3490 1"}),
        ET.Element("material", {"name": "macao_wrist", "rgba": "0.8196 0.3529 0.0118 1"}),
        ET.Element("material", {"name": "macao_pad", "rgba": "0.82 0.84 0.86 1"}),
    ]
    actuators = ET.Element("actuator")
    for joint in ("macao_x", "macao_y", "macao_z"):
        ET.SubElement(actuators, "position", {"name": joint, "joint": joint, "kp": "900", "kv": "80"})
    for joint in ("macao_roll", "macao_pitch", "macao_yaw"):
        ET.SubElement(actuators, "position", {"name": joint, "joint": joint, "kp": "80", "kv": "18"})
    ET.SubElement(actuators, "position", {"name": "macao_grip", "joint": finger_joints[0][0],
                                           "kp": "30", "ctrlrange": "0 1.1"})

    equality = ET.Element("equality")
    for joint, ratio in finger_joints[1:]:
        ET.SubElement(equality, "joint", {"joint1": joint, "joint2": finger_joints[0][0],
                                           "polycoef": f"0 {ratio} 0 0 0", "solref": "0.005 1"})

    for elem in body.iter("body"):
        elem.set("gravcomp", "1")
    return body, meshes, materials, {"actuator": actuators, "equality": equality}


def robot_parts(cfg, menagerie: Path, workdir: Path):

    if cfg["name"] == "macao":
        return macao_parts(cfg)
    if "source" not in cfg:
        return fr3_parts(menagerie)

    (OUT / "assets" / cfg["name"]).mkdir(parents=True, exist_ok=True)
    body, meshes, materials, extras = adopt_robot(
        cfg, menagerie, workdir, OUT / "assets" / cfg["name"])

    if cfg["name"] == "so101":
        for material in materials:
            if material.get("name", "").endswith("_material") and "sts3215" not in material.get("name", ""):
                material.set("rgba", "0.094118 0.611765 0.792157 1")

    if cfg.get("stow"):
        
        src = mujoco.MjModel.from_xml_path(str(menagerie / cfg["source"]))
        stance = {mujoco.mj_id2name(src, mujoco.mjtObj.mjOBJ_JOINT, j):
                  float(src.key_qpos[0][src.jnt_qposadr[j]]) for j in range(src.njnt)}
        keep = set(cfg["arm"]) | {cfg["grip"]["actuator"]}
        freeze_joints(body, keep, stance)

        contact = extras.get("contact")
        if contact is None:
            contact = ET.Element("contact")
            extras["contact"] = contact
        for child in body.findall("body"):
            ET.SubElement(contact, "exclude",
                          {"body1": body.get("name"), "body2": child.get("name")})

    driven = set(cfg["arm"]) | {cfg["grip"]["actuator"]}
    actuators = ET.Element("actuator")
    for act in (extras["actuator"] if extras.get("actuator") is not None else []):
        if act.get("joint") in driven:
            actuators.append(act)
    extras["actuator"] = actuators

    for elem in body.iter("body"):
        elem.set("gravcomp", "1")

    if cfg.get("jaws"):
        for parent in body.iter("body"):
            if parent.get("name") not in cfg["jaws"]:
                continue
            for geom in parent.findall("geom"):
                if geom.get("contype") == "0":
                    continue
                geom.set("friction", "2 0.05 0.0002")
                geom.set("solimp", "0.95 0.99 0.001")
                geom.set("solref", "0.005 1")
    return body, meshes, materials, extras


def drop_to_floor(cfg, scene, path: Path, body):

    path.write_text(ET.tostring(scene, encoding="unicode"))
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    low = 0.0
    for g in range(model.ngeom):
        if model.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        mesh = model.geom_dataid[g]
        adr, num = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
        world = model.mesh_vert[adr:adr + num] @ data.geom_xmat[g].reshape(3, 3).T
        low = min(low, float((world + data.geom_xpos[g])[:, 2].min()))

    pos = [float(v) for v in (body.get("pos") or "0 0 0").split()]
    body.set("pos", fmt([pos[0], pos[1], pos[2] - low]))


def jaw_centre(cfg, path: Path):
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    moving, fixed = cfg["jaws"]
    grip = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, cfg["grip"]["actuator"])
    joint = model.actuator_trnid[grip][0]
    data.qpos[model.jnt_qposadr[joint]] = cfg["grip"]["grasp"]
    mujoco.mj_forward(model, data)

    pivot = data.xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, fixed)]

    def cloud(name):

        chunks = []
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        for g in range(model.ngeom):
            if model.geom_bodyid[g] != bid or model.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
                continue
            mesh = model.geom_dataid[g]
            adr, num = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
            chunks.append(model.mesh_vert[adr:adr + num] @ data.geom_xmat[g].reshape(3, 3).T
                          + data.geom_xpos[g])
        return np.vstack(chunks)

    depth = cfg.get("grip_depth")
    near_m, near_f = cloud(moving), cloud(fixed)
    for _ in range(1):
        reach_m = np.linalg.norm(near_m - pivot, axis=1)
        reach_f = np.linalg.norm(near_f - pivot, axis=1)
        near_m = near_m[reach_m >= np.quantile(reach_m, 0.55)]
        near_f = near_f[reach_f >= np.quantile(reach_f, 0.55)]
    gaps = np.linalg.norm(near_m[:, None, :] - near_f[None, :, :], axis=2)
    i, j = np.unravel_index(int(np.argmin(gaps)), gaps.shape)
    tip_m, tip_f = near_m[i], near_f[j]
    world = (tip_m + tip_f) / 2

    far_m = cloud(moving)
    far_f = cloud(fixed)
    reach_m = np.linalg.norm(far_m - pivot, axis=1)
    reach_f = np.linalg.norm(far_f - pivot, axis=1)
    tip_m = far_m[reach_m >= np.quantile(reach_m, 0.8)].mean(axis=0)
    tip_f = far_f[reach_f >= np.quantile(reach_f, 0.8)].mean(axis=0)

    if depth is not None:
        world = pivot + ((tip_m + tip_f) / 2 - pivot) * depth

    axis_z = world - pivot
    axis_z = axis_z / np.linalg.norm(axis_z)
    axis_y = tip_m - tip_f
    axis_y = axis_y - axis_z * (axis_y @ axis_z)
    axis_y = axis_y / np.linalg.norm(axis_y)
    rot_world = np.column_stack([np.cross(axis_y, axis_z), axis_y, axis_z])

    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, fixed)
    body_rot = data.xmat[bid].reshape(3, 3)
    local = body_rot.T @ (world - data.xpos[bid])
    return fixed, local, matrix_quat(body_rot.T @ rot_world)


def spin_address(model):
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "board_spin")
    return int(model.jnt_qposadr[jid]) if jid >= 0 else -1


def board_normal(cfg):

    if cfg.get("name") == "macao":
        return (-1.0, 0.0, 0.0)
    return tuple(quat_matrix(np.array(cfg.get("board_quat", BOARD_FLAT))) @ [1.0, 0, 0])


def emit_robot(cfg, menagerie: Path, board, hiveboard: Path):
    scene = ET.fromstring(SCENE_HEAD.replace("__NAME__", cfg["name"]))
    asset = scene.find("asset")
    worldbody = scene.find("worldbody")

    if "stand" in cfg:
        stand = cfg["stand"]
        ET.SubElement(worldbody, "geom", {
            "name": "stand_post", "type": "box", "rgba": "0.34 0.38 0.45 1",
            "size": fmt((0.016, 0.016, stand["top"] / 2)),
            "pos": fmt((cfg["board"][0] + 0.03, cfg["board"][1], stand["top"] / 2))})
        ET.SubElement(worldbody, "geom", {
            "name": "stand_foot", "type": "box", "rgba": "0.44 0.48 0.55 1",
            "size": fmt((stand["half"], stand["half"], 0.01)),
            "pos": fmt((cfg["board"][0] + 0.03, cfg["board"][1], 0.01))})
    else:
        bench = cfg["bench"]
        ET.SubElement(worldbody, "geom", {
            "name": "bench_top", "type": "box", "rgba": "0.44 0.48 0.55 1",
            "size": fmt((bench["half"], bench["half"], 0.008)),
            "pos": fmt((cfg["board"][0], cfg["board"][1], bench["top"] - 0.008))})
        for sx, sy in ([(1, 1), (1, -1), (-1, 1), (-1, -1)] if bench["top"] > 0.10 else []):
            ET.SubElement(worldbody, "geom", {
                "name": f"bench_leg{sx}{sy}".replace("-", "n"), "type": "box",
                "rgba": "0.34 0.38 0.45 1",
                "size": fmt((0.012, 0.012, (bench["top"] - 0.016) / 2)),
                "pos": fmt((cfg["board"][0] + sx * (bench["half"] - 0.02),
                            cfg["board"][1] + sy * (bench["half"] - 0.02),
                            (bench["top"] - 0.016) / 2))})

    with tempfile.TemporaryDirectory() as tmp:
        body, meshes, materials, extras = robot_parts(cfg, menagerie, Path(tmp))

    for tag in ("default", "actuator", "contact", "tendon"):
        elem = extras.get(tag)
        if elem is not None and (tag == "default" or len(elem)):
            scene.append(elem)

    equality = ET.Element("equality")
    for elem in (list(extras["equality"]) if extras.get("equality") is not None else []) + board["equality"]:
        equality.append(elem)
    if len(equality):
        scene.append(equality)
    existing_mat_names = {m.get("name") for m in asset.findall("material")}
    existing_mesh_names = {m.get("name") for m in asset.findall("mesh")}
    for elem in meshes + board["meshes"]:
        name = elem.get("name")
        if not name or name not in existing_mesh_names:
            asset.append(elem)
            if name:
                existing_mesh_names.add(name)
    for elem in materials:
        name = elem.get("name")
        if not name or name not in existing_mat_names:
            asset.append(elem)
            if name:
                existing_mat_names.add(name)

    worldbody.append(body)
    holder = ET.SubElement(worldbody, "body", {
        "name": "hiveboard", "pos": fmt(cfg["board"]),
        "quat": fmt(cfg.get("board_quat", BOARD_FLAT))})
    if cfg.get("spin_board"):
        ET.SubElement(holder, "joint", {
            "name": "board_spin", "type": "hinge", "axis": "1 0 0",
            "damping": "20", "frictionloss": "40", "armature": "0.5"})
    for frag in board["fragments"]:
        holder.append(frag)

    path = OUT / f"{cfg['name']}.xml"
    ET.indent(scene, "  ")
    path.write_text(ET.tostring(scene, encoding="unicode") + "\n")

    if cfg.get("stow"):
        drop_to_floor(cfg, scene, path, body)

    if "tcp" in cfg:
        tcp_body, tcp_pos, tcp_quat = (*cfg["tcp"], (1, 0, 0, 0))
    else:
        tcp_body, tcp_pos, tcp_quat = jaw_centre(cfg, path)
    for elem in scene.iter("body"):
        if elem.get("name") == tcp_body:
            ET.SubElement(elem, "site", {"name": "tcp", "pos": fmt(tcp_pos),
                                         "quat": fmt(tcp_quat), "size": "0.005",
                                         "group": "4"})
            break

    key = ET.SubElement(ET.SubElement(scene, "keyframe"), "key",
                        {"name": "home", "qpos": fmt(home_qpos(cfg, scene, path))})
    ET.indent(scene, "  ")
    path.write_text(ET.tostring(scene, encoding="unicode") + "\n")

    settle(cfg, path, key)
    pin_modules(path, key)
    ET.indent(scene, "  ")
    path.write_text(ET.tostring(scene, encoding="unicode") + "\n")

    model = mujoco.MjModel.from_xml_path(str(path))
    print(f"  {cfg['name']:6s} compiles: nq={model.nq} nu={model.nu} "
          f"nbody={model.nbody} nmesh={model.nmesh}")

    cfg = dict(cfg, board_normal=board_normal(cfg))
    tasks = sim_trajectories.build(path, cfg)
    sim_trajectories.dump(tasks, OUT / f"{cfg['name']}.traj.json")

    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    lo = np.array([1e9, 1e9, 1e9])
    hi = -lo.copy()
    for g in range(model.ngeom):
        if model.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        mesh = model.geom_dataid[g]
        adr, num = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
        world = (model.mesh_vert[adr:adr + num] @ data.geom_xmat[g].reshape(3, 3).T
                 + data.geom_xpos[g])
        lo, hi = np.minimum(lo, world.min(axis=0)), np.maximum(hi, world.max(axis=0))
    centre = (lo + hi) / 2
    span = float(np.linalg.norm(hi - lo))

    return {
        "name": cfg["name"], "label": cfg["label"], "note": cfg["note"],
        "scene": f"{cfg['name']}.xml", "traj": f"{cfg['name']}.traj.json",
        "arm": len(cfg["arm"]), "grip": cfg["grip"], "home": cfg["home"],
        "gripIndex": int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR,
                                           cfg["grip"]["actuator"])),
        "spinIndex": spin_address(model),
        "board": list(cfg["board"]),
        "view": {"centre": [round(v, 4) for v in centre], "span": round(span, 4)},
        "framing": cfg.get("framing", "over"),
        "boardNormal": [round(v, 4) for v in board_normal(cfg)],
        "tasks": list(tasks),
        **({"taskHome": cfg["task_home"]} if cfg.get("task_home") else {}),
    }


def build(hiveboard: Path):

    menagerie = ensure_menagerie()
    shutil.rmtree(OUT, ignore_errors=True)
    (OUT / "assets/fr3").mkdir(parents=True)
    (OUT / "assets/hb").mkdir(parents=True)

    board = build_board(hiveboard)

    catalogue = []
    for cfg in ROBOTS:
        if cfg.get("soon"):
            catalogue.append({"name": cfg["name"], "label": cfg["label"],
                              "note": cfg["note"], "soon": True})
            print(f"  {cfg['name']:6s} (soon)")
            continue
        catalogue.append(emit_robot(cfg, menagerie, board, hiveboard))

    (OUT / "robots.json").write_text(json.dumps(catalogue, indent=1) + "\n")
    manifest()
    vendor()


def vendor():

    dst = REPO / "public/sim/vendor"
    dst.mkdir(parents=True, exist_ok=True)
    mj = REPO / "node_modules/@mujoco/mujoco"
    three = REPO / "node_modules/three"

    shutil.copyfile(mj / "mujoco.js", dst / "mujoco.js")
    (dst / "mujoco.wasm.gz").write_bytes(gzip.compress((mj / "mujoco.wasm").read_bytes(), 9))
    for src in [three / "build/three.module.min.js",
                three / "build/three.core.min.js",
                three / "examples/jsm/controls/OrbitControls.js"]:
        shutil.copyfile(src, dst / src.name)

    versions = {
        "mujoco": (mj / "package.json"),
        "three": (three / "package.json"),
    }
    for name, pkg in versions.items():
        version = re.search(r'"version":\s*"([^"]+)"', pkg.read_text()).group(1)
        print(f"vendored {name} {version}")


def pin_modules(path: Path, key):

    rest = {f"{mod['name']}_{joint}": value
            for mod in MODULES for joint, value in mod.get("pose", {}).items()}
    model = mujoco.MjModel.from_xml_path(str(path))
    qpos = np.array([float(v) for v in key.get("qpos").split()])
    for j in range(model.njnt):
        name = model.joint(j).name or ""
        if name.split("_", 1)[0] in {mod["name"] for mod in MODULES}:
            qpos[model.jnt_qposadr[j]] = rest.get(name, 0.0)
    key.set("qpos", fmt(qpos))


def settle(cfg, path: Path, key):

    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    held = len(cfg["home"])
    arm = np.array(cfg["home"])
    grip_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR,
                                cfg["grip"]["actuator"])
    grip_joint = int(model.actuator_trnid[grip_id][0]) if grip_id >= 0 else -1
    grip_qpos = int(model.jnt_qposadr[grip_joint]) if grip_joint >= 0 else -1
    grip_dof = int(model.jnt_dofadr[grip_joint]) if grip_joint >= 0 else -1
    grip_open = cfg["grip"]["open"]
    for _ in range(4000):
        data.qpos[:held] = arm
        data.qvel[:held] = 0
        if grip_qpos >= 0:
            data.qpos[grip_qpos] = grip_open
            data.qvel[grip_dof] = 0
            data.ctrl[grip_id] = grip_open
        mujoco.mj_step(model, data)

    if grip_qpos >= 0:
        data.qpos[grip_qpos] = grip_open
    key.set("qpos", fmt(np.concatenate([arm, data.qpos[held:]])))


def manifest():

    def emit(path: Path) -> int:
        data = path.read_bytes()
        packed = gzip.compress(data, 9)
        gz = path.with_suffix(path.suffix + ".gz")
        if not gz.exists() or gzip.decompress(gz.read_bytes()) != data:
            gz.write_bytes(packed)
        return len(data), len(packed)

    files, raw, comp = [], 0, 0
    for path in sorted(OUT.rglob("*")):
        if path.is_dir() or path.suffix == ".gz" or path.name == "manifest.json":
            continue
        size, packed = emit(path)
        files.append(str(path.relative_to(OUT)))
        raw += size
        comp += packed

    listing = OUT / "manifest.json"
    listing.write_text("[\n" + ",\n".join(f'  "{f}"' for f in files) + "\n]\n")
    emit(listing)

    for stale in OUT.rglob("*.gz"):
        if not stale.with_suffix("").exists():
            stale.unlink()

    print(f"{len(files)} files: {raw / 1048576:.2f} MB -> {comp / 1048576:.2f} MB gzipped")


SCENE_HEAD = """<mujoco model="hiveboard __NAME__">
  <compiler angle="radian" meshdir="assets" autolimits="true"/>
  <option integrator="implicitfast" timestep="0.002"/>
  <size memory="24M"/>

  <visual>
    <global offwidth="1920" offheight="1080"/>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.35 0.35 0.35" specular="0 0 0"/>
  </visual>

  <asset>
    <material name="white" rgba="1 1 1 1"/>
    <material name="off_white" rgba="0.901961 0.921569 0.929412 1"/>
    <material name="black" rgba="0.25 0.25 0.25 1"/>
    <material name="green" rgba="0 1 0 1"/>
    <material name="light_blue" rgba="0.039216 0.541176 0.780392 1"/>
    <material name="red" rgba="1 0 0 1"/>
    <material name="hb_shell" rgba="0.925 0.933 0.945 1" specular="0.3" shininess="0.2"/>
    <material name="hb_panel" rgba="0.76 0.79 0.83 1" specular="0.25" shininess="0.15"/>
    <material name="hb_accent" rgba="0.929 0.686 0.098 1" specular="0.4" shininess="0.3"/>
  </asset>

  <worldbody>
    <light pos="0 0 2.4" dir="0 0 -1" directional="true"/>
    <geom name="floor" type="plane" size="0 0 0.05" rgba="0.62 0.66 0.72 1"/>
  </worldbody>
</mujoco>
"""

FR3_DEFAULTS = """<default>
  <default class="fr3">
    <material specular="0.5" shininess="0.25"/>
    <joint armature="0.1" damping="1" axis="0 0 1"/>
    <default class="visual">
      <geom type="mesh" contype="0" conaffinity="0" group="2"/>
    </default>
    <default class="collision">
      <geom type="mesh" group="3" rgba="0 0 0 0"/>
    </default>
    <default class="finger">
      <joint axis="0 1 0" type="slide" range="0 0.04" armature="0.01" damping="12"/>
    </default>
    <default class="pad">
      <geom type="box" group="3" rgba="0 0 0 0" friction="2 0.05 0.0002"
            solimp="0.95 0.99 0.001" solref="0.005 1"/>
    </default>
  </default>
</default>
"""

FR3_ACTUATORS = """<actuator>
  <position name="fr3_joint1" joint="fr3_joint1" kp="900" kv="55" forcerange="-87 87"/>
  <position name="fr3_joint2" joint="fr3_joint2" kp="900" kv="55" forcerange="-87 87"/>
  <position name="fr3_joint3" joint="fr3_joint3" kp="800" kv="45" forcerange="-87 87"/>
  <position name="fr3_joint4" joint="fr3_joint4" kp="800" kv="45" forcerange="-87 87"/>
  <position name="fr3_joint5" joint="fr3_joint5" kp="300" kv="14" forcerange="-12 12"/>
  <position name="fr3_joint6" joint="fr3_joint6" kp="250" kv="12" forcerange="-12 12"/>
  <position name="fr3_joint7" joint="fr3_joint7" kp="120" kv="6" forcerange="-12 12"/>
  <position name="gripper" joint="finger_joint1" ctrlrange="0 0.04" kp="5000" kv="90"
            forcerange="-150 150"/>
</actuator>
"""

FR3_EQUALITY = """<equality>
  <joint joint1="finger_joint2" joint2="finger_joint1" polycoef="0 1 0 0 0"/>
</equality>
"""

ARM_BODY = """<body name="fr3_link0" childclass="fr3">
  <geom mesh="link0_0" material="black" class="visual"/>
  <geom mesh="link0_1" material="white" class="visual"/>
  <geom mesh="link0_2" material="white" class="visual"/>
  <geom mesh="link0_3" material="white" class="visual"/>
  <geom mesh="link0_4" material="white" class="visual"/>
  <geom mesh="link0_5" material="red" class="visual"/>
  <geom mesh="link0_6" material="black" class="visual"/>
  <geom name="fr3_link0_collision" mesh="link0_coll" class="collision"/>
  <body name="fr3_link1" pos="0 0 0.333" gravcomp="1">
    <inertial pos="4.128e-07 -0.0181251 -0.0386036" quat="0.998098 -0.0605364 0.00380499 0.0110109"
      mass="2.92747" diaginertia="0.0239286 0.0227246 0.00610634"/>
    <joint name="fr3_joint1" range="-2.7437 2.7437" actuatorfrcrange="-87 87" armature="0.16" frictionloss="0.35"/>
    <geom material="white" mesh="link1" class="visual"/>
    <geom name="fr3_link1_collision" class="collision" mesh="link1_coll"/>
    <body name="fr3_link2" quat="1 -1 0 0" gravcomp="1">
      <inertial pos="0.00318289 -0.0743222 0.00881461" quat="0.502599 0.584437 -0.465998 0.434366"
        mass="2.93554" diaginertia="0.0629567 0.0411924 0.0246371"/>
      <joint name="fr3_joint2" range="-1.7837 1.7837" actuatorfrcrange="-87 87" armature="0.16" frictionloss="0.35"/>
      <geom material="white" mesh="link2" class="visual"/>
      <geom name="fr3_link2_collision" class="collision" mesh="link2_coll"/>
      <body name="fr3_link3" pos="0 -0.316 0" quat="1 1 0 0" gravcomp="1">
        <inertial pos="0.0407016 -0.00482006 -0.0289731" quat="0.921025 -0.244161 0.155272 0.260745"
          mass="2.2449" diaginertia="0.0267409 0.0189869 0.0171587"/>
        <joint name="fr3_joint3" range="-2.9007 2.9007" actuatorfrcrange="-87 87" armature="0.16" frictionloss="0.35"/>
        <geom material="white" mesh="link3_0" class="visual"/>
        <geom material="black" mesh="link3_1" class="visual"/>
        <geom name="fr3_link3_collision" class="collision" mesh="link3_coll"/>
        <body name="fr3_link4" pos="0.0825 0 0" quat="1 1 0 0" gravcomp="1">
          <inertial pos="-0.0459101 0.0630493 -0.00851879" quat="0.438393 0.803505 0.00937859 0.402334"
            mass="2.6156" diaginertia="0.05139 0.0392202 0.0221056"/>
          <joint name="fr3_joint4" range="-3.0421 -0.1518" actuatorfrcrange="-87 87" armature="0.16" frictionloss="0.35"/>
          <geom material="white" mesh="link4_0" class="visual"/>
          <geom material="black" mesh="link4_1" class="visual"/>
          <geom name="fr3_link4_collision" class="collision" mesh="link4_coll"/>
          <body name="fr3_link5" pos="-0.0825 0.384 0" quat="1 -1 0 0" gravcomp="1">
            <inertial pos="-0.00160396 0.0292536 -0.0972966" quat="0.999707 0.000217874 -0.00506589 -0.0236221"
              mass="2.32712" diaginertia="0.0579335 0.0563653 0.00988706"/>
            <joint name="fr3_joint5" range="-2.8065 2.8065" actuatorfrcrange="-12 12" armature="0.06" frictionloss="0.2"/>
            <geom material="black" mesh="link5_0" class="visual"/>
            <geom material="white" mesh="link5_1" class="visual"/>
            <geom material="white" mesh="link5_2" class="visual"/>
            <geom name="fr3_link5_collision" class="collision" mesh="link5_coll"/>
            <body name="fr3_link6" quat="1 1 0 0" gravcomp="1">
              <inertial pos="0.0597131 -0.0410295 -0.0101693" quat="0.621301 0.552665 0.510011 0.220081"
                mass="1.81704" diaginertia="0.0175039 0.0161123 0.00193529"/>
              <joint name="fr3_joint6" range="0.5445 4.5169" actuatorfrcrange="-12 12" armature="0.06" frictionloss="0.2"/>
              <geom material="green" mesh="link6_0" class="visual"/>
              <geom material="white" mesh="link6_1" class="visual"/>
              <geom material="white" mesh="link6_2" class="visual"/>
              <geom material="white" mesh="link6_3" class="visual"/>
              <geom material="white" mesh="link6_4" class="visual"/>
              <geom material="white" mesh="link6_5" class="visual"/>
              <geom material="white" mesh="link6_6" class="visual"/>
              <geom material="light_blue" mesh="link6_7" class="visual"/>
              <geom name="fr3_link6_collision" class="collision" mesh="link6_coll"/>
              <body name="fr3_link7" pos="0.088 0 0" quat="1 1 0 0" gravcomp="1">
                <inertial pos="0.00452258 0.00862619 -0.0161633" quat="0.727579 0.0978688 -0.24906 0.63168"
                  mass="0.627143" diaginertia="0.000223836 0.000223642 5.64132e-07"/>
                <joint name="fr3_joint7" range="-3.0159 3.0159" actuatorfrcrange="-12 12" armature="0.074" frictionloss="0.248"/>
                <geom material="black" mesh="link7_0" class="visual"/>
                <geom material="white" mesh="link7_1" class="visual"/>
                <geom material="white" mesh="link7_2" class="visual"/>
                <geom material="black" mesh="link7_3" class="visual"/>
                <geom name="fr3_link7_collision" class="collision" mesh="link7_coll"/>
                <body name="hand" pos="0 0 0.107" quat="0.9238795 0 0 -0.3826834" gravcomp="1">
                  <inertial mass="0.73" pos="-0.01 0 0.03" diaginertia="0.001 0.0025 0.0017"/>
                  <geom mesh="hand_0" material="off_white" class="visual"/>
                  <geom mesh="hand_1" material="black" class="visual"/>
                  <geom mesh="hand_2" material="black" class="visual"/>
                  <geom mesh="hand_3" material="white" class="visual"/>
                  <geom mesh="hand_4" material="off_white" class="visual"/>
                  <geom name="hand_collision" mesh="hand_c" class="collision"/>
                  <body name="left_finger" pos="0 0 0.0584" gravcomp="1">
                    <inertial mass="0.015" pos="0 0 0" diaginertia="2.375e-6 2.375e-6 7.5e-7"/>
                    <joint name="finger_joint1" class="finger"/>
                    <geom mesh="finger_0" material="off_white" class="visual"/>
                    <geom mesh="finger_1" material="black" class="visual"/>
                    <geom name="left_pad" class="pad" size="0.009 0.004 0.019" pos="0 0.0065 0.045"/>
                  </body>
                  <body name="right_finger" pos="0 0 0.0584" quat="0 0 0 1" gravcomp="1">
                    <inertial mass="0.015" pos="0 0 0" diaginertia="2.375e-6 2.375e-6 7.5e-7"/>
                    <joint name="finger_joint2" class="finger"/>
                    <geom mesh="finger_0" material="off_white" class="visual"/>
                    <geom mesh="finger_1" material="black" class="visual"/>
                    <geom name="right_pad" class="pad" size="0.009 0.004 0.019" pos="0 0.0065 0.045"/>
                  </body>
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>
    </body>
  </body>
</body>
"""


def main():

    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--hiveboard",
        default=os.environ.get("HIVEBOARD_SIM", str(Path.home() / "HiveBoard/Simulation")),
        help="directory holding the released HiveBoard URDFs",
    )
    args = ap.parse_args()

    hiveboard = Path(args.hiveboard).expanduser()
    if not (hiveboard / PANEL_URDF).exists():
        sys.exit(f"no HiveBoard URDFs under {hiveboard} (pass --hiveboard)")
    build(hiveboard)


if __name__ == "__main__":
    main()
