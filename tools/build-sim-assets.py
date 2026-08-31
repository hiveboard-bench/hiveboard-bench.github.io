#!/usr/bin/env python3
"""Build the MuJoCo scenes the in-browser simulation widget loads.

public/sim/hiveboard-sim.html compiles an MJCF at runtime inside MuJoCo WASM,
so every mesh it names has to be fetched over the wire first. This script
assembles one scene per robot in ROBOTS, each working the same board:

  * the robots come from mujoco_menagerie (Apache-2.0), cloned into .cache/ on
    first run and grafted in by compiling them and taking the flattened MJCF
    MuJoCo writes back -- rather than transcribing Spot's twenty-two bodies;
  * the HiveBoard modules come from the URDF release, converted the same way.

Each robot's trajectories are then solved and verified by sim_trajectories, and
a robot only advertises the tasks it actually completed.

Menagerie ships CAD-resolution visual meshes -- 33 MB of ASCII OBJ for the arm
alone, which is more than the rest of the site put together. We drop the vertex
normals (MuJoCo recomputes them at compile time, honouring sharp edges) and
decimate anything dense, which lands the whole scene near 4 MB, then pre-gzip
so GitHub Pages does not have to.

Run after changing the scene layout or re-exporting any HiveBoard module, then
`npm run build` to republish:

    pip install mujoco numpy fast-simplification
    npm install
    python3 tools/build-sim-assets.py

It also copies the MuJoCo WASM engine and three.js out of node_modules into
public/sim/vendor/, so run `npm install` first and re-run this after bumping
either package.
"""
import argparse
import gzip
import json
import math
import os
import re
import shutil
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

# Faces above this get simplified; below it the mesh is already cheap.
DECIMATE_ABOVE = 3000
# Fraction of faces to keep for meshes that do get simplified.
KEEP_RATIO = 0.35

# ── Scene layout ────────────────────────────────────────────────────────────
# The board lies flat on the floor in front of the arm. Its authoring frame has
# the panel in the local YZ plane with modules protruding along +X, so the mount
# rotation maps local +X to world +Z (up) and local +Z to world +X.
BOARD_POS = (0.52, 0.0, 0.2)

# Two ways the board gets presented, both taken from the photographs. Its
# authoring frame has the panel in the local YZ plane with modules protruding
# along local +X, so the mount rotation is really a choice of where +X points.
#   FLAT     lies the panel on a bench, modules pointing up  -- local +X -> +Z
#   UPRIGHT  stands it on a post, modules pointing at the robot -- local +X -> -X
BOARD_FLAT = (0.0, math.sqrt(0.5), 0.0, math.sqrt(0.5))    # w x y z
BOARD_UPRIGHT = (0.0, 0.0, 0.0, 1.0)
BOARD_QUAT = BOARD_FLAT

# Hexagonal cell centres in the panel's own frame, straight out of
# Honeycomb_Panel.urdf. Index 0 is the middle cell.
CELLS = [
    (0.0, 0.0),
    (0.086603, 0.0),
    (0.043301, 0.075),
    (-0.043301, 0.075),
    (-0.086603, 0.0),
    (-0.043301, -0.075),
    (0.043301, -0.075),
]

# Which HiveBoard modules to seat, and in which cell. Each one is spun about
# its own axis at build time so its lever or lid points out of the board
# rather than across its neighbours -- see cell_pose().
MODULES = [
    # No return spring at all: a ball valve stays where it is put. Its hinge
    # axis is vertical, so gravity gives it nothing to fall back to, and the
    # friction holds it steady until the loop resets the scene.
    {"name": "valve", "urdf": "Valves/Lever Valve/Ball Valve/Ball_Valve.urdf", "cell": 2,
     "joints": {"RevoluteJoint": {"stiffness": "0", "damping": "0.1",
                                  "frictionloss": "0.02"}}},
    # Turning the bulb has to lift it through the thread equality, so its
    # travel must not also be fighting a return spring -- with one fitted, the
    # bulb stayed pinned at the bottom of its stroke and the gripper just slid
    # around it.
    {"name": "lamp", "urdf": "Lamp/Lamp_Assembly.urdf", "cell": 4,
     "joints": {"PrismaticJoint": {"stiffness": "0", "damping": "1",
                                  # Enough travel to lift the bulb's base clear
                                  # of the socket rim 30 mm above it, so the
                                  # lamp visibly comes out rather than just
                                  # loosening. The trajectory only commands as
                                  # much of it as one wrist can turn.
                                  "range": "0 0.05"},
                # A thread turns freely once it is off its seat. With the
                # default hinge spring fitted, the gripper had to fight it and
                # slipped: 2.7 rad of wrist bought 1.0 rad of bulb. Free, the
                # same grasp carries it 2.1 rad. Gravity still screws it back
                # down afterwards, through the thread equality.
                # Bounded by the thread it rides: 50 mm of travel at
                # LAMP_PITCH is 6.1 rad and no more. Left unlimited, a gripper
                # that loses its grip spins the bulb freely and the equality
                # drags the travel 110 mm past a 50 mm stop.
                "RevoluteJoint": {"stiffness": "0", "damping": "0.02",
                                  "frictionloss": "0.002", "armature": "0.0005",
                                  "range": "0 6.1", "limited": "true"}}},
    # No spring: a breaker toggle latches. The URDF's default was stiff enough
    # to stall the arm at 0.33 of the toggle's 0.52 rad throw, and softening it
    # to 0.10 only made the snap-back slower -- released at its stop the toggle
    # still walked back to a third of the throw, so every run ended with the
    # breaker in the position it started in and the task undone. Its own stop
    # holds it now, the way the ball valve's does; the settled rest angle is
    # unchanged at about a degree, on a bench and on an upright board alike.
    {"name": "breaker", "urdf": "Circuit Breaker/Circuit_Breaker_Assembly.urdf", "cell": 6,
     "joints": {"RevoluteJoint": {"stiffness": "0"}}},
]

PANEL_URDF = "Honeycomb/Honeycomb_Panel.urdf"

# ── Robots ──────────────────────────────────────────────────────────────────
# One scene per robot, all working the same board. Everything upstream is
# pulled from mujoco_menagerie and grafted by adopt_robot(); the FR3 is the
# exception because its Panda hand is an assembly menagerie does not ship.
#
# `board` is where the board's centre sits in that robot's world, chosen for
# its reach; `bench` is the height of the surface under it. `grip` gives the
# gripper actuator's command in its own units -- metres of finger travel for
# the FR3's parallel jaws, radians for the hinged jaws on Spot and the SO-101.
ROBOTS = [
    {
        "name": "fr3",
        "label": "Franka FR3",
        "note": "reference arm",
        "arm": [f"fr3_joint{i}" for i in range(1, 8)],
        "grip": {"actuator": "gripper", "open": 0.034, "grasp": 0.002, "fist": 0.0},
        "home": [0.0, -0.0881, 0.0, -2.1491, 0.0, 2.0611, 0.79],
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
        "stow": True,          # weld the legs and the base, drive the arm only
        "home": [0.0, -1.9, 2.0, 0.0, -0.6, 0.0],
        # Spot works the board the way the lab rig presents it: stood upright
        # on a slim post at chest height, panel facing the robot, modules
        # sticking out towards it. Reaching horizontally into a vertical board
        # is what its arm is shaped for -- laying the board flat on a bench had
        # it stooping over the far side of its own workspace.
        "board": (1.08, 0.0, 0.70),
        "board_quat": BOARD_UPRIGHT,
        "stand": {"top": 0.70, "half": 0.17},
        # Pinch with the very tip of the claw. Aimed at the closing point deep
        # in its throat it swept past the lever entirely; aimed halfway down it
        # swallowed the lever and parked the whole claw over the board.
        "grip_depth": 0.95,
        # Watched from the flank at a distance scaled to the robot; the fixed
        # over-the-board framing that suits an arm on a bench puts the camera
        # between Spot's own legs.
        "framing": "side",
        # The lamp, held 20 mm short of where the FR3 takes it: a claw that
        # closes past its own fingertips hooks the glass on the way out if it
        # reaches as far in. Its wrist also lands a different angle for the same
        # hand orientation, so the unscrew starts wound further back to fit the
        # whole turn inside one wrist.
        "lamp_standoff": 0.01,
        "lamp_wind": 1.5,
        "lamp_approach": 0.25,
        "lamp_demo": True,
        # And the valve lever held 15 mm further down the bar, towards the hub:
        # up at the middle of it the claw reads as hovering over the lever's tip
        # rather than holding it. Lower down there is less to slip against, so
        # the swing finishes on the stop rather than just shy of it -- which
        # matters here, since on an upright board a lever released short of its
        # stop simply falls back open.
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
        # Park folded clear of every board angle. The former home posture sat
        # over the lamp at its task angle and pushed the bulb out during reset,
        # before the trajectory had even begun.
        "home": [0.0, 0.0, 0.0, 0.0, 0.0],
        # Keep the old, well-conditioned posture as the IK nullspace bias only;
        # the authored path still starts and ends at the collision-free home.
        "ik_home": [0.0, -0.6, 0.9, 0.6, 0.0],
        # Desk-sized: the board sits on the same surface as the arm's own base
        # rather than up on a stand.
        "board": (0.29, 0.0, 0.01),
        "bench": {"half": 0.17, "top": 0.01},
        "tilt": 1.3,
        # A desk-sized arm has no headroom to stand 100 mm off the board before
        # descending, and hardly any orientation to spare either: five joints
        # cannot hit a pose exactly, so the solver is told to spend what little
        # it has on reaching the point rather than on the wrist angle.
        "clearance": 0.05,
        "rot_weight": 0.05,
        # Grip near the fingertips: its jaws are long enough that the throat is
        # nowhere anything actually sits.
        "grip_depth": 0.85,
        # Turn the stand to bring each module round to the near side.
        "spin_board": True,
        # Put each task directly in front of the short arm. The geometric
        # facing angle is only a starting point: the valve works best one cell
        # clockwise from it, while the bulb needs a small extra turn to keep
        # the elbow clear of the panel through the whole orbiting grasp.
        "task_spins": {"valve": math.radians(-90), "lamp": math.radians(97)},
        # Five joints cannot keep an arbitrary gripper roll while tracking the
        # bulb axis. Hold it off-axis instead: orbiting this 42 mm radius turns
        # the bulb mechanically without asking for a sixth wrist DOF.
        "lamp_grasp_radius": 0.042,
        # The clearance goal already includes 4 mm above the socket rim. The
        # larger generic multiplier was added to mask Python/WASM differences;
        # this path is accepted from its task-only motion instead of the old
        # reset transient, so the measured clearance itself is the threshold.
        "lamp_tolerance": 1.0,
        # The gripper stays securely in contact, but the under-actuated wrist
        # trades 6-7 mm of TCP position for its approach direction on the arc.
        "lamp_ik_tolerance": 0.007,
        "lamp_demo": True,
    },
    {
        "name": "platform_c",
        "label": "ANYmal with DynaArm",
        "note": "Platform C",
        "soon": True,
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
        "mount": (0.45, 0.0, 0.52),
        "tcp": ("macao_hand", (0.0, 0.0, 0.11)),
        "skip": ["valve", "lamp", "breaker", "toggle", "button", "dial"],
    },
]

# MuJoCo needs a <mujoco> block inside the URDF to pick up mesh paths and to be
# told to keep the visual geoms, which it discards by default on URDF import.
URDF_HINT = (
    '<mujoco><compiler meshdir="." balanceinertia="true" '
    'discardvisual="false" strippath="false" fusestatic="false" '
    'inertiagrouprange="0 0"/></mujoco>'
)


# ═══════════════════════════════════════════════════════════════════════════
#  OBJ handling
# ═══════════════════════════════════════════════════════════════════════════
def obj_read(path):
    """Parse an OBJ into (Nx3 float verts, Mx3 int faces). Normals are dropped."""
    verts, faces = [], []
    for line in path.read_text().splitlines():
        if line.startswith("v "):
            verts.append([float(x) for x in line.split()[1:4]])
        elif line.startswith("f "):
            idx = [int(tok.split("/")[0]) for tok in line.split()[1:]]
            # OBJ indices are 1-based and may be negative (relative to the end).
            idx = [i - 1 if i > 0 else len(verts) + i for i in idx]
            for k in range(1, len(idx) - 1):  # fan-triangulate
                faces.append([idx[0], idx[k], idx[k + 1]])
    return np.asarray(verts, np.float64), np.asarray(faces, np.int64)


def obj_write(path, verts, faces):
    """Write a normal-free OBJ, trimming coordinates to micrometre precision."""
    out = ["v %.6g %.6g %.6g" % tuple(v) for v in verts]
    out += ["f %d %d %d" % tuple(f + 1) for f in faces]
    path.write_text("\n".join(out) + "\n")


def weld(verts, faces, tol=1e-7):
    """Merge coincident vertices and drop the faces that collapse as a result.

    The menagerie OBJs are exported unwelded -- link1 alone carries 11516
    vertices for 8028 faces, one per face corner. A simplifier cannot collapse
    an edge whose endpoints are duplicated, so decimating the raw mesh shatters
    it into confetti. Welding first restores the shared topology.
    """
    keys = np.round(verts / tol).astype(np.int64)
    _, first, inverse = np.unique(keys, axis=0, return_index=True, return_inverse=True)
    order = np.argsort(first)               # keep the original vertex order
    remap = np.empty(len(first), np.int64)
    remap[order] = np.arange(len(first))
    faces = remap[inverse.ravel()][faces]
    verts = verts[first[order]]
    keep = (faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2]) & (faces[:, 0] != faces[:, 2])
    return verts, faces[keep]


def decimate(verts, faces):
    """Collapse a dense mesh down to KEEP_RATIO of its faces."""
    verts, faces = weld(verts, faces)
    if len(faces) <= DECIMATE_ABOVE:
        return verts, faces
    import fast_simplification

    v, f = fast_simplification.simplify(
        verts.astype(np.float32), faces.astype(np.int32), 1.0 - KEEP_RATIO
    )
    return np.asarray(v, np.float64), np.asarray(f, np.int64)


def stl_read(path):
    """Parse a binary or ASCII STL into (verts, faces). Every face is loose."""
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
    """Copy one mesh into the output tree, decimating it on the way.

    STLs used to be passed through untouched, on the grounds that the only ones
    in the scene were the FR3's already-coarse collision hulls. The SO-101 ships
    its *visual* meshes as STL, and 18 of them came to 17 MB -- more than every
    other robot put together. Everything is decimated now, and STL comes out the
    other side as OBJ.
    """
    dst_dir.mkdir(parents=True, exist_ok=True)
    stl = src.suffix.lower() == ".stl"
    verts, faces = (stl_read(src) if stl else obj_read(src))
    if simplify:
        verts, faces = decimate(verts, faces)
    out = name or (src.stem + ".obj")
    obj_write(dst_dir / out, verts, faces)
    return out


# ═══════════════════════════════════════════════════════════════════════════
#  Sources
# ═══════════════════════════════════════════════════════════════════════════
def prepare_anymal_arm(menagerie_dir: Path):
    """Assemble ANYmal C + DynaArm model and a symmetric standing stance.

    The arm meshes are published in each link's own frame.  The previous
    reconstruction chained them along a guessed +X axis, even though DynaArm's
    forearm is authored along +Z and two wrist frames are rotated.  That made
    intact meshes look like disconnected or missing parts.  The transforms
    below follow Duatic's DynaArm description (baracuda12/corydoras12).
    """
    import mujoco
    anymal_dir = menagerie_dir / "anybotics_anymal_c"
    assets_dir = anymal_dir / "assets"
    assets_dir.mkdir(parents=True, exist_ok=True)

    src_dyna = REPO / "tools/assets/dynaarm"
    if src_dyna.exists():
        for f in src_dyna.glob("*.obj"):
            shutil.copy(f, assets_dir / f.name)

    xml_path = anymal_dir / "anymal_c.xml"
    arm_xml_path = anymal_dir / "anymal_c_arm.xml"
    if not xml_path.exists():
        return

    tree = ET.parse(xml_path)
    root = tree.getroot()

    asset = root.find("asset")
    mesh_names = [
        "0_base_alma_mesh", "1000_dynaarm_interface_parts",
        "100_shoulder_mesh", "200_upperarm_mesh",
        "300_elbow_mesh", "400_forearm_mesh", "500_wrist1_mesh"
    ]
    for m in mesh_names:
        if root.find(f'.//mesh[@name="{m}"]') is None:
            attrs = {"name": m, "file": f"{m}.obj"}
            # The ALMA adapter is the one legacy mesh exported in millimetres.
            if m == "0_base_alma_mesh":
                attrs["scale"] = "0.001 0.001 0.001"
            ET.SubElement(asset, "mesh", attrs)

    anymal_mats = {
        # Match the requested Spot-style red/grey livery.  The four materials
        # are deliberately repeated across the body and arm so the silhouette
        # stays readable without the upstream texture files in MuJoCo WASM.
        "base": "0.16 0.18 0.20 1",
        "top_shell": "0.72 0.055 0.065 1",
        "bottom_shell": "0.20 0.22 0.24 1",
        "hip_l": "0.18 0.20 0.22 1",
        "hip_r": "0.18 0.20 0.22 1",
        "thigh": "0.52 0.55 0.58 1",
        "shank_l": "0.25 0.27 0.29 1",
        "shank": "0.25 0.27 0.29 1",
        "foot": "0.10 0.11 0.12 1",
        "hatch": "0.72 0.055 0.065 1",
        "remote": "0.16 0.18 0.20 1",
        "handle": "0.55 0.58 0.61 1",
        "face": "0.12 0.13 0.14 1",
        "depth_camera": "0.1 0.1 0.1 1",
        "wide_angle_camera": "0.1 0.1 0.1 1",
        "battery": "0.52 0.55 0.58 1",
        "lidar_cage": "0.1 0.1 0.1 1",
        "lidar": "0.1 0.1 0.1 1",
        "drive": "0.72 0.055 0.065 1",
        "black_plastic": "0.10 0.11 0.12 1",
        "green": "0.72 0.055 0.065 1",
        "red": "0.28 0.30 0.32 1",
        "yellow": "0.58 0.61 0.64 1",
        "lwl": "0.76 0.78 0.80 1",
        "arm_dark": "0.12 0.14 0.16 1",
        "arm_grey": "0.56 0.59 0.62 1",
        "arm_light": "0.76 0.78 0.80 1",
        "arm_accent": "0.72 0.055 0.065 1",
    }
    for mat in root.findall(".//material"):
        mname = mat.get("name")
        mat.attrib.pop("texture", None)
        if mname in anymal_mats:
            mat.set("rgba", anymal_mats[mname])
    for tex in root.findall(".//texture"):
        asset.remove(tex)
    for mname, rgba in anymal_mats.items():
        if root.find(f'.//material[@name="{mname}"]') is None:
            ET.SubElement(asset, "material", {"name": mname, "rgba": rgba})

    base = root.find('.//body[@name="base"]')
    if root.find('.//body[@name="dynaarm_base"]') is None:
        dyna_base = ET.SubElement(base, "body", {"name": "dynaarm_base", "pos": "0.05 0 0.12"})
        ET.SubElement(dyna_base, "geom", {"type": "mesh", "mesh": "0_base_alma_mesh",
                                          "material": "arm_grey", "class": "visual"})
        # The legacy interface export is a partial ALMA adapter (it contains a
        # tall sensor bracket, not the DynaArm pedestal) and made the arm look
        # like it was growing out of a black tower.  Use a complete, legible
        # three-stage pedestal instead: mounting plate, red housing, and dark
        # shoulder collar.  The mesh remains in the asset set for compatibility
        # with older cached scenes, but is intentionally not drawn here.
        ET.SubElement(dyna_base, "geom", {"type": "cylinder", "size": "0.135 0.010",
                                          "pos": "0 0 0.015", "material": "arm_grey", "class": "visual"})
        ET.SubElement(dyna_base, "geom", {"type": "cylinder", "size": "0.105 0.045",
                                          "pos": "0 0 0.065", "material": "top_shell", "class": "visual"})
        ET.SubElement(dyna_base, "geom", {"type": "cylinder", "size": "0.080 0.014",
                                          "pos": "0 0 0.112", "material": "arm_dark", "class": "visual"})
        ET.SubElement(dyna_base, "geom", {"type": "cylinder", "size": "0.105 0.045",
                                          "pos": "0 0 0.065", "class": "collision"})
        ET.SubElement(dyna_base, "geom", {"type": "box", "size": "0.08 0.08 0.04",
                                          "pos": "0 0 0.04", "class": "collision"})

        sh = ET.SubElement(dyna_base, "body", {"name": "dynaarm_shoulder", "pos": "0 0 0.1105"})
        ET.SubElement(sh, "joint", {"name": "arm_joint1", "axis": "0 0 1", "range": "-3.14 3.14", "damping": "2"})
        ET.SubElement(sh, "geom", {"type": "mesh", "mesh": "100_shoulder_mesh",
                                   "material": "arm_dark", "class": "visual"})
        ET.SubElement(sh, "geom", {"type": "cylinder", "size": "0.045 0.05", "class": "collision"})

        up = ET.SubElement(sh, "body", {"name": "dynaarm_upperarm", "pos": "0 0 0.047",
                                         "euler": "1.570796 -1.570796 0"})
        ET.SubElement(up, "joint", {"name": "arm_joint2", "axis": "0 0 1", "range": "-1.7208 1.7208", "damping": "2"})
        ET.SubElement(up, "geom", {"type": "mesh", "mesh": "200_upperarm_mesh",
                                   "material": "arm_grey", "class": "visual"})
        ET.SubElement(up, "geom", {"type": "capsule", "size": "0.04 0.2", "pos": "0.2 0 0",
                                   "quat": "0.707 0 0.707 0", "class": "collision"})

        el = ET.SubElement(up, "body", {"name": "dynaarm_elbow", "pos": "0.4127 0 0"})
        ET.SubElement(el, "joint", {"name": "arm_joint3", "axis": "0 0 1", "range": "0 3.09159", "damping": "2"})
        ET.SubElement(el, "geom", {"type": "mesh", "mesh": "300_elbow_mesh",
                                   "material": "arm_dark", "class": "visual"})
        ET.SubElement(el, "geom", {"type": "cylinder", "size": "0.045 0.05", "quat": "0.707 0.707 0 0",
                                   "class": "collision"})

        fa = ET.SubElement(el, "body", {"name": "dynaarm_forearm", "pos": "0.0262 -0.0855 0",
                                         "euler": "1.570796 1.570796 1.570796"})
        ET.SubElement(fa, "joint", {"name": "arm_joint4", "axis": "0 0 1", "range": "-4.71239 4.71239", "damping": "1.5"})
        ET.SubElement(fa, "geom", {"type": "mesh", "mesh": "400_forearm_mesh",
                                   "material": "arm_grey", "class": "visual"})
        ET.SubElement(fa, "geom", {"type": "capsule", "size": "0.035 0.19", "pos": "0 0 0.21",
                                   "class": "collision"})

        w1 = ET.SubElement(fa, "body", {"name": "dynaarm_wrist1", "pos": "0.0295 0 0.4207",
                                         "euler": "0 -1.570796 0"})
        ET.SubElement(w1, "joint", {"name": "arm_joint5", "axis": "0 0 1", "range": "-1.8208 1.8208", "damping": "1"})
        ET.SubElement(w1, "geom", {"type": "mesh", "mesh": "500_wrist1_mesh",
                                   "material": "arm_dark", "class": "visual"})
        ET.SubElement(w1, "geom", {"type": "cylinder", "size": "0.035 0.04", "quat": "0.707 0.707 0 0",
                                   "class": "collision"})

        w2 = ET.SubElement(w1, "body", {"name": "dynaarm_wrist2", "pos": "0.117 0 0.0295",
                                         "euler": "-1.570796 1.570796 -1.570796"})
        ET.SubElement(w2, "joint", {"name": "arm_joint6", "axis": "0 0 1", "range": "-1.5708 4.71239", "damping": "1"})
        # The old asset bundle predates DynaArm's separate wrist-2 and flange
        # exports.  Build those small coaxial parts from primitives so the
        # published kinematic chain is visually complete instead of ending in
        # a floating cube.
        ET.SubElement(w2, "geom", {"type": "cylinder", "size": "0.050 0.034",
                                     "pos": "0 0 0.034", "material": "arm_grey", "class": "visual"})
        ET.SubElement(w2, "geom", {"type": "cylinder", "size": "0.043 0.038",
                                     "pos": "0 0 0.037", "material": "arm_dark", "class": "visual"})
        ET.SubElement(w2, "geom", {"type": "cylinder", "size": "0.037 0.012",
                                     "pos": "0 0 0.080", "material": "arm_accent", "class": "visual"})
        ET.SubElement(w2, "geom", {"type": "cylinder", "size": "0.049 0.006",
                                     "pos": "0 0 0.096", "material": "arm_light", "class": "visual"})
        ET.SubElement(w2, "geom", {"type": "cylinder", "size": "0.047 0.048",
                                     "pos": "0 0 0.048", "class": "collision"})

        palm = ET.SubElement(w2, "body", {"name": "dynaarm_gripper", "pos": "0 0 0.108"})
        ET.SubElement(palm, "geom", {"type": "box", "size": "0.045 0.038 0.018",
                                      "pos": "0 0 0.018", "material": "arm_grey", "class": "visual"})
        ET.SubElement(palm, "geom", {"type": "box", "size": "0.039 0.031 0.008",
                                      "pos": "0 0 0.043", "material": "arm_dark", "class": "visual"})
        ET.SubElement(palm, "geom", {"type": "box", "size": "0.045 0.038 0.018",
                                      "pos": "0 0 0.018", "class": "collision"})

        jaw1 = ET.SubElement(palm, "body", {"name": "dynaarm_jaw_left", "pos": "0 0.018 0.048"})
        ET.SubElement(jaw1, "joint", {"name": "gripper", "axis": "0 1 0", "range": "0 0.04",
                                      "type": "slide", "damping": "5"})
        ET.SubElement(jaw1, "geom", {"type": "box", "size": "0.010 0.010 0.050", "pos": "0 0 0.050",
                                     "material": "arm_dark", "class": "visual"})
        ET.SubElement(jaw1, "geom", {"type": "box", "size": "0.012 0.012 0.020", "pos": "0 -0.003 0.098",
                                     "material": "arm_accent", "class": "visual"})
        ET.SubElement(jaw1, "geom", {"type": "box", "size": "0.010 0.010 0.050", "pos": "0 0 0.050",
                                     "class": "collision"})

        jaw2 = ET.SubElement(palm, "body", {"name": "dynaarm_jaw_right", "pos": "0 -0.018 0.048"})
        ET.SubElement(jaw2, "geom", {"type": "box", "size": "0.010 0.010 0.050", "pos": "0 0 0.050",
                                     "material": "arm_dark", "class": "visual"})
        ET.SubElement(jaw2, "geom", {"type": "box", "size": "0.012 0.012 0.020", "pos": "0 0.003 0.098",
                                     "material": "arm_accent", "class": "visual"})
        ET.SubElement(jaw2, "geom", {"type": "box", "size": "0.010 0.010 0.050", "pos": "0 0 0.050",
                                     "class": "collision"})

        actuator = root.find("actuator")
        for i in range(1, 7):
            ET.SubElement(actuator, "position", {"name": f"arm_joint{i}", "joint": f"arm_joint{i}",
                                                 "class": "affine", "kp": "200"})
        ET.SubElement(actuator, "position", {"name": "gripper", "joint": "gripper",
                                             "class": "affine", "kp": "300", "ctrlrange": "0 0.04"})

        # Welding the quadruped base for this stationary demo removes MuJoCo's
        # usual parent/child collision filtering.  Explicit exclusions keep the
        # arm from pushing against its own mounting block while its position
        # servos hold the parked pose.
        contact = root.find("contact")
        if contact is None:
            contact = ET.SubElement(root, "contact")
        for body1, body2 in (
            ("dynaarm_base", "dynaarm_shoulder"),
            ("dynaarm_base", "dynaarm_upperarm"),
            ("dynaarm_shoulder", "dynaarm_upperarm"),
            ("dynaarm_upperarm", "dynaarm_elbow"),
            ("dynaarm_elbow", "dynaarm_forearm"),
            ("dynaarm_forearm", "dynaarm_wrist1"),
            ("dynaarm_wrist1", "dynaarm_wrist2"),
            ("dynaarm_wrist2", "dynaarm_gripper"),
            ("dynaarm_gripper", "dynaarm_jaw_left"),
            ("dynaarm_gripper", "dynaarm_jaw_right"),
        ):
            ET.SubElement(contact, "exclude", {"body1": body1, "body2": body2})

    kf = root.find("keyframe")
    if kf is None:
        kf = ET.SubElement(root, "keyframe")

    tree.write(str(arm_xml_path))
    m = mujoco.MjModel.from_xml_path(str(arm_xml_path))
    jnames = [m.joint(i).name for i in range(m.njnt)]
    stance_dict = {
        # ANYmal C's front and rear feet have an 11.8 mm authored frame offset,
        # so a numerically mirrored knee angle leaves the front pair hovering.
        # This calibrated front angle puts all four contact spheres on one
        # plane while retaining a left/right symmetric, level stance.
        "LF_HAA": 0.03, "LF_HFE": 0.4, "LF_KFE": -0.513,
        "RF_HAA": -0.03, "RF_HFE": 0.4, "RF_KFE": -0.513,
        "LH_HAA": -0.03, "LH_HFE": -0.4, "LH_KFE": 0.6,
        "RH_HAA": 0.03, "RH_HFE": -0.4, "RH_KFE": 0.6,
        "arm_joint1": 0.0, "arm_joint2": -0.8, "arm_joint3": 1.8,
        "arm_joint4": 0.0, "arm_joint5": -1.0, "arm_joint6": 0.0,
        "gripper": 0.02
    }
    # qpos is not one scalar per joint: the floating base consumes seven slots
    # (xyz + unit quaternion).  Building one value per joint shifted every leg
    # command, producing the mangled stance seen in the browser after the leg
    # joints were baked into fixed body transforms.
    qpos_list = np.zeros(m.nq)
    for j in range(m.njnt):
        name = m.joint(j).name
        adr = m.jnt_qposadr[j]
        if m.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE:
            qpos_list[adr:adr + 7] = (0.0, 0.0, 0.62, 1.0, 0.0, 0.0, 0.0)
        else:
            qpos_list[adr] = stance_dict.get(name, 0.0)
    for k in kf.findall("key"):
        kf.remove(k)
    ET.SubElement(kf, "key", {"name": "standing", "qpos": fmt(qpos_list)})
    tree.write(str(arm_xml_path))


def ensure_menagerie() -> Path:
    """Sparse-clone the menagerie models we need, once."""
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
    prepare_anymal_arm(root)
    return root


def load_urdf(urdf: Path, workdir: Path):
    """Compile a HiveBoard URDF through MuJoCo and hand back the MJCF it emits."""
    import mujoco

    staged = workdir / urdf.parent.name
    if not staged.exists():
        shutil.copytree(urdf.parent, staged)
    path = staged / urdf.name
    text = path.read_text()
    if "<mujoco>" not in text:
        text = re.sub(r"(<robot[^>]*>)", r"\1\n  " + URDF_HINT, text, count=1)
        path.write_text(text)

    model = mujoco.MjModel.from_xml_path(str(path))
    saved = workdir / (urdf.stem + ".mjcf.xml")
    mujoco.mj_saveLastXML(str(saved), model)
    return (ET.parse(saved).getroot(), urdf.parent / "meshes",
            base_depth(model), reach_direction(model))


def adopt_robot(cfg, menagerie: Path, workdir: Path, out_assets: Path):
    """Graft a menagerie robot into our scene, meshes and actuators and all.

    Same trick as the HiveBoard URDFs: compile the upstream model, ask MuJoCo
    to write it back out, and take the flattened MJCF. Defaults are baked in by
    then, so what comes back is self-contained -- which beats transcribing
    Spot's twenty-two bodies by hand and then maintaining them.
    """
    import mujoco

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

    # Everything the robot needs that is not geometry.
    extras = {tag: root.find(tag) for tag in
              ("actuator", "contact", "equality", "tendon", "default")}
    return body, meshes, root.findall("./asset/material"), extras


def quat_matrix(q):
    """Rotation matrix from a MuJoCo (w, x, y, z) quaternion."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_quat(m):
    """MuJoCo (w, x, y, z) quaternion from a rotation matrix."""
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
    """Delete every joint the robot will not be driving, baking in its angle.

    Spot arrives as a free-floating quadruped. Parked on a stand with only its
    arm in play, twelve leg joints and a free base are twelve joints of drift
    and a base that can tip over. Deleting them outright welds the legs at
    whatever angle the body frames were authored with, which for Spot is bolt
    upright -- a stance it never actually stands in. So each dropped joint's
    standing angle is folded into its child body's transform first.
    """
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

            # Rotate the body about the joint's axis through its anchor, both
            # expressed in the parent's frame.
            spin = axis_matrix(rot @ axis_local, angle)
            offset = rot @ anchor_local
            parent.set("pos", fmt(pos + offset - spin @ offset))
            parent.set("quat", fmt(matrix_quat(spin @ rot)))
    return dropped


def reach_direction(model) -> tuple:
    """Which way a module's moving parts stick out, in its own (y, z) plane.

    A ball valve's lever and a button's lid both swing well outside the hex
    footprint. Seating them at whatever angle the CAD happened to use drops
    them across the neighbouring cells, so the scene builder rotates each
    module until this vector points away from the middle of the board.
    """
    import mujoco

    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    total = np.zeros(2)
    weight = 0.0
    for g in range(model.ngeom):
        body = model.geom_bodyid[g]
        if body <= 1 or model.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
            continue  # body 1 is the fixed base; only what hangs off a joint counts
        mesh = model.geom_dataid[g]
        adr, num = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
        world = model.mesh_vert[adr:adr + num] @ data.geom_xmat[g].reshape(3, 3).T
        world = world + data.geom_xpos[g]
        total += world[:, 1:].mean(axis=0) * num
        weight += num

    if weight == 0:
        return (0.0, 0.0)
    vec = total / weight
    norm = float(np.linalg.norm(vec))
    return tuple(vec / norm) if norm > 1e-6 else (0.0, 0.0)


def base_depth(model) -> float:
    """How far a module's base plate sits below its own origin, along local +X.

    The modules are authored with the mounting plate hanging into negative X --
    Button_Assembly, for instance, spans x = -0.021..0.058 -- while a honeycomb
    cell occupies x = 0..0.02. Seating a module by this offset drops its plate
    into the cell instead of through the floor.
    """
    import mujoco

    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    lo = 0.0
    for g in range(model.ngeom):
        if model.geom_bodyid[g] != 1 or model.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
            continue  # body 1 is the URDF root link, i.e. the mounting plate
        mesh = model.geom_dataid[g]
        adr, num = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
        world = model.mesh_vert[adr:adr + num] @ data.geom_xmat[g].reshape(3, 3).T
        lo = min(lo, float((world + data.geom_xpos[g])[:, 0].min()))
    return -lo


# ═══════════════════════════════════════════════════════════════════════════
#  URDF → scene fragment
# ═══════════════════════════════════════════════════════════════════════════
def adopt(root, prefix, mesh_src: Path, meshes: dict, shell="hb_shell", pose=None, tune=None):
    """Namespace one converted module and normalise its geom groups.

    MuJoCo's URDF importer emits visual geoms as group 1 and collision geoms as
    group 0. The widget draws every geom below group 3, so visuals move to
    group 2 and collisions to group 3 -- the menagerie convention, which keeps
    the arm and the board on the same footing.

    Returns the module's root body and its <mesh> elements; the elements are
    carried over whole because several HiveBoard meshes are authored in
    millimetres and only their `scale` attribute says so.
    """
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
        # A geom on the module's own base is fixed hardware; anything deeper
        # hangs off a joint, so it gets the accent colour and the widget's
        # "what actually moves here" reading comes for free.
        material = shell if body is base else "hb_accent"
        for geom in body.findall("geom"):
            if geom.get("mesh"):
                geom.set("mesh", f"{prefix}_{geom.get('mesh')}")
            visual = geom.get("group") == "1"
            geom.set("group", "2" if visual else "3")
            geom.set("material", material)
            if not visual:
                # Collision geoms stay collidable but are never drawn.
                geom.set("rgba", "0 0 0 0")
                # MuJoCo collides the *convex hull* of a mesh, so a hollow
                # housing swallows whatever presses into it: the lamp sat
                # 11 mm inside its own socket, the lid 5 mm inside the panel.
                # That standing penetration pumps energy into the board -- it
                # spun the lamp's unlimited screw joint up to 154000 rad while
                # settling.
                #
                # So the board's fixed shells carry no contact at all: their
                # hulls are fiction, and a solid hull over a socket also walls
                # the gripper out of the very features it is meant to reach.
                # The moving parts stay collidable against the arm (contype 2
                # meets the arm's conaffinity 1) but not against each other.
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
        # URDF describes the kinematics but not the return springs and detents
        # the printed modules actually have, so without these a lid or lever
        # just falls to whichever joint limit gravity points at and lies across
        # the board. Sprung to its rest pose, each one instead springs back
        # after the arm -- or a visitor's cursor -- pushes it.
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
    """Pose of a module seated in `cell`, expressed in the board's own frame.

    The module is spun about its own axis until `reach` -- the direction its
    moving parts stick out -- points away from the middle of the board. The
    centre cell has no outward direction, so it faces the arm instead.
    """
    y, z = CELLS[cell]
    want = (y, z) if (y or z) else (0.0, -1.0)   # centre cell faces -Z, i.e. the arm
    norm = math.hypot(*want)
    want = (want[0] / norm, want[1] / norm)

    if reach == (0.0, 0.0):
        spin = 0.0
    else:
        # Signed angle from `reach` to `want` about the module's local +X.
        spin = math.atan2(reach[0] * want[1] - reach[1] * want[0],
                          reach[0] * want[0] + reach[1] * want[1])

    half = spin / 2.0
    return (lift, y, z), (math.cos(half), math.sin(half), 0.0, 0.0)


def fmt(vals):
    return " ".join("%.6g" % v for v in vals)


# ═══════════════════════════════════════════════════════════════════════════
#  Scene assembly
# ═══════════════════════════════════════════════════════════════════════════
def build_board(hiveboard: Path):
    """Convert the board and its modules once; every robot scene reuses them."""
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
            root, mesh_dir, lift, reach = load_urdf(hiveboard / mod["urdf"], tmp)
            body, mod_elems = adopt(root, mod["name"], mesh_dir, meshes,
                                    pose=mod.get("pose"), tune=mod.get("joints"))
            pos, quat = cell_pose(mod["cell"], lift, reach)
            body.set("pos", fmt(pos))
            body.set("quat", fmt(quat))
            fragments.append(body)
            elems += mod_elems

    for name, (src, out_name) in meshes.items():
        emit_mesh(src, OUT / "assets/hb", out_name)

    import sim_trajectories

    # The lamp screws in. URDF gives its rotation and its travel as two
    # independent joints, so the thread tying them together is added here, at
    # the pitch the unscrew trajectory climbs at.
    #
    # Stiff, because a thread is: on MuJoCo's default equality softness a
    # gripper that had hold of the glass could pull the bulb 30 mm straight out
    # of a thread it had barely turned, which is not unscrewing it -- and the
    # travel is what the task is scored on. At this stiffness the bulb comes out
    # only as far as it is turned, which is what the constraint is there to say.
    thread = ET.Element("joint", {
        "joint1": "lamp_PrismaticJoint", "joint2": "lamp_RevoluteJoint",
        "polycoef": "0 %.6g 0 0 0" % sim_trajectories.LAMP_PITCH,
        "solref": "0.01 1", "solimp": "0.95 0.999 0.001"})
    return {"fragments": fragments, "meshes": elems, "equality": [thread]}


def fr3_parts(menagerie: Path):
    """The FR3 arm with a Panda hand -- an assembly menagerie does not ship."""
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
    """Import the Macao STLs as an articulated hand and forearm.

    The upstream project publishes individual parts rather than a robot
    description. Their coordinates are already in millimetres in the assembly
    frame, so preserve that layout and apply MuJoCo's metre scale once. The
    published finger is instanced for four fingers and a separately posed,
    opposed thumb.
    """
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

    def visual(parent, mesh, material="macao_shell", pos=None):
        attrs = {
            "type": "mesh", "mesh": mesh, "material": material,
            "contype": "0", "conaffinity": "0", "group": "2"}
        if pos:
            attrs["pos"] = pos
        ET.SubElement(parent, "geom", attrs)

    # The CAD assembly points its fingers along local +Z. Rotate it so they
    # point along world +X, toward the upright board, with the forearm behind.
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

    # Flip only the forearm/wrist/palm shell around the hand's longitudinal
    # axis. Finger and thumb bodies remain siblings, so their corrected side,
    # roots, hinge axes and trajectories are unchanged.
    shell = ET.SubElement(body, "body", {
        "name": "macao_arm_wrist", "quat": "0 0 0 1"})

    visual(shell, "forearm", "macao_wrist")
    visual(shell, "forearm_base", "macao_base")
    for key in ("palm_middle", "palm_outer", "palm_cover", "forearm_lid"):
        visual(shell, key, "macao_shell")

    finger_joints = []

    def digit(parent, prefix, master=False):
        """Build the printed finger around its three physical hinge axes."""
        visual(parent, "finger_base")

        proximal = ET.SubElement(parent, "body", {"name": f"{prefix}_proximal", "pos": "0 0 0.016"})
        proximal_joint = "macao_grip" if master else f"{prefix}_prox_joint"
        ET.SubElement(proximal, "joint", {"name": proximal_joint, "type": "hinge",
                                           "axis": "-1 0 0", "range": "0 1.15", "damping": "0.35"})
        finger_joints.append((proximal_joint, 1.0))
        visual(proximal, "finger_first", pos="0 0 -0.016")
        visual(proximal, "finger_first_pad", "macao_pad", "0 0 -0.016")

        medial = ET.SubElement(proximal, "body", {"name": f"{prefix}_medial", "pos": "0 0 0.030"})
        medial_joint = f"{prefix}_medial_joint"
        ET.SubElement(medial, "joint", {"name": medial_joint, "type": "hinge",
                                         "axis": "-1 0 0", "range": "0 1.0", "damping": "0.3"})
        finger_joints.append((medial_joint, 0.78))
        visual(medial, "finger_medial", pos="0 0 -0.046")
        visual(medial, "finger_medial_pad", "macao_pad", "0 0 -0.046")

        distal = ET.SubElement(medial, "body", {"name": f"{prefix}_distal", "pos": "0 0 0.0245"})
        distal_joint = f"{prefix}_distal_joint"
        ET.SubElement(distal, "joint", {"name": distal_joint, "type": "hinge",
                                         "axis": "-1 0 0", "range": "0 0.9", "damping": "0.25"})
        finger_joints.append((distal_joint, 0.65))
        visual(distal, "finger_distal", pos="0 0 -0.0705")
        visual(distal, "finger_distal_pad", "macao_pad", "0 0 -0.0705")

    # Four fingers across the palm. Lift their bases out to the shell edge.
    # The hand rotation makes local X appear as vertical height in the viewer.
    # Add one hardcoded correction per finger here, in metres. Positive values
    # move that finger downward in world Z.
    finger_height_offsets = (-0.02, -0.005, 0.005, -0.015)
    for i, x in enumerate((-0.033, -0.011, 0.011, 0.033)):
        mount = ET.SubElement(body, "body", {
            "name": f"macao_finger_body_{i}",
            "pos": f"{x} 0.01 {finger_height_offsets[i] + 0.04}"})
        digit(mount, f"macao_finger_{i}", master=(i == 0))


    # The source has no separate thumb STL. Macao uses the same printed finger
    # mechanism, mounted under the palm in an opposed orientation.
    thumb = ET.SubElement(body, "body", {
        "name": "macao_thumb_body", "pos": "0.018 0.063 -0.005",
        # Keep the previous 180-degree local-X flip, then add a 180-degree
        # local-Y turn. The composed quaternion leaves all thumb joints intact.
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

    # This custom platform returns before robot_parts()' shared gravity
    # compensation pass, so apply it to the shell and every articulated digit
    # here as well as the root body.
    for elem in body.iter("body"):
        elem.set("gravcomp", "1")
    return body, meshes, materials, {"actuator": actuators, "equality": equality}


def robot_parts(cfg, menagerie: Path, workdir: Path):
    """Whatever this robot contributes to a scene: a body, meshes, actuators."""
    if cfg["name"] == "macao":
        return macao_parts(cfg)
    if "source" not in cfg:
        return fr3_parts(menagerie)

    (OUT / "assets" / cfg["name"]).mkdir(parents=True, exist_ok=True)
    body, meshes, materials, extras = adopt_robot(
        cfg, menagerie, workdir, OUT / "assets" / cfg["name"])

    if cfg["name"] == "so101":
        # Match the requested SO-101 body color while retaining the dark servo
        # housings and fasteners.
        for material in materials:
            if material.get("name", "").endswith("_material") and "sts3215" not in material.get("name", ""):
                material.set("rgba", "0.094118 0.611765 0.792157 1")

    if cfg.get("stow"):
        import mujoco
        src = mujoco.MjModel.from_xml_path(str(menagerie / cfg["source"]))
        stance = {mujoco.mj_id2name(src, mujoco.mjtObj.mjOBJ_JOINT, j):
                  float(src.key_qpos[0][src.jnt_qposadr[j]]) for j in range(src.njnt)}
        keep = set(cfg["arm"]) | {cfg["grip"]["actuator"]}
        freeze_joints(body, keep, stance)

        # MuJoCo stops filtering parent-child contacts once the parent is welded
        # to the world, which welding the base is exactly what does. Spot's
        # shoulder then ground against its own chassis with 100 N.m of
        # constraint force, and the servo lost: the arm sat 0.2 rad off every
        # pose the solver gave it and touched nothing on the board all run.
        contact = extras.get("contact")
        if contact is None:
            contact = ET.Element("contact")
            extras["contact"] = contact
        for child in body.findall("body"):
            ET.SubElement(contact, "exclude",
                          {"body1": body.get("name"), "body2": child.get("name")})

    # Only the joints we drive keep an actuator.
    driven = set(cfg["arm"]) | {cfg["grip"]["actuator"]}
    actuators = ET.Element("actuator")
    for act in (extras["actuator"] if extras.get("actuator") is not None else []):
        if act.get("joint") in driven:
            actuators.append(act)
    extras["actuator"] = actuators

    # Gravity compensation, as the FR3 block already carries. Without it a
    # position servo holds a pose only as well as its gain allows, and Spot's
    # arm sagged 21 cm below where the solver had put it -- the trajectory was
    # right and the robot simply was not where it was told to be.
    for elem in body.iter("body"):
        elem.set("gravcomp", "1")

    # A hinged jaw closing on a moulded lever needs grip, same as the FR3's pads.
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
    """Sit the robot on the ground rather than trusting an authored mount height."""
    import mujoco

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
    """Where the fingertips meet, in the frame of whatever body carries them.

    Trajectories are authored against this point, so it has to be measured off
    the real jaws rather than guessed -- Spot's and the SO-101's are hinged and
    curved, and neither has a tool frame sitting where the grip actually is.
    """
    import mujoco

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

    # Where the jaws actually meet when shut, not the midpoint between their
    # far ends. Spot's clamshell is lopsided enough that the two differ by
    # centimetres, and aiming at the wrong one had it sweeping straight past
    # every lever on the board without touching one.
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

    # Orientation still comes from the far ends: the closing pair are almost on
    # top of each other, so they say nothing about which way the jaws open.
    far_m = cloud(moving)
    far_f = cloud(fixed)
    reach_m = np.linalg.norm(far_m - pivot, axis=1)
    reach_f = np.linalg.norm(far_f - pivot, axis=1)
    tip_m = far_m[reach_m >= np.quantile(reach_m, 0.8)].mean(axis=0)
    tip_f = far_f[reach_f >= np.quantile(reach_f, 0.8)].mean(axis=0)

    # Where along the jaw the grip actually happens. The closest-approach point
    # is right for a clamshell that shuts onto itself, but on a long hinged jaw
    # it lands deep in the throat where nothing ever sits -- the SO-101 closed
    # straight past a 23 mm lever without ever touching it. `grip_depth` slides
    # the tool frame out toward the fingertips instead.
    if depth is not None:
        world = pivot + ((tip_m + tip_f) / 2 - pivot) * depth

    # And which way the tool points. Trajectories are authored as "reach along
    # the site's Z, open the jaws along its Y", so every robot needs a site
    # oriented that way -- the bodies these hang off are aimed differently on
    # every arm, and taking their raw axes had Spot trying to grasp the board
    # edge-on and missing by 99 mm.
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
    """qpos slot of the board's turntable joint, or -1 if this board is fixed."""
    import mujoco

    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "board_spin")
    return int(model.jnt_qposadr[jid]) if jid >= 0 else -1


def board_normal(cfg):
    """Which way the board faces: the world direction its modules stick out."""
    return tuple(quat_matrix(np.array(cfg.get("board_quat", BOARD_FLAT))) @ [1.0, 0, 0])


def macao_motions(model):
    """Looping demonstrations for the hand pose and synchronized digits."""
    import mujoco

    rate = 50

    def make(label, caption, seconds, pose, close):
        count = rate * seconds + 1
        data = mujoco.MjData(model)
        site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "tcp")
        qpos, grip, tcp = [], [], []
        for i in range(count):
            phase = 2 * math.pi * i / (count - 1)
            q = pose(phase)
            g = close(phase)
            data.qpos[:6] = q
            mujoco.mj_forward(model, data)
            qpos.append([round(v, 6) for v in q])
            grip.append(round(g, 6))
            tcp.append([round(float(v), 6) for v in data.site_xpos[site]])
        return {
            "label": label, "caption": caption, "rate": rate,
            "watch": "macao_yaw", "goal": 0.0, "left": 0.0, "spin": 0.0,
            "qpos": qpos, "grip": grip, "tcp": tcp,
        }

    sweep = lambda p: [
        0.055 * math.sin(p), 0.035 * math.sin(2 * p), 0.045 * math.sin(p),
        0.22 * math.sin(2 * p), 0.30 * math.sin(p), 0.42 * math.sin(p),
    ]
    still = lambda _p: [0.0] * 6
    open_hand = lambda _p: 0.0
    one_close = lambda p: 0.55 * (1 - math.cos(p))
    two_closes = lambda p: 0.55 * (1 - math.cos(2 * p))

    return {
        "finger_flex": make(
            "Flex fingers",
            "Close and reopen all four fingers and the opposed thumb twice.",
            7, still, two_closes),
        "hand_sweep": make(
            "Move whole hand",
            "Translate and rotate the complete Macao hand while the fingers remain open.",
            8, sweep, open_hand),
        "hand_motion": make(
            "Move hand and fingers",
            "Move the complete Macao hand through space while all four fingers and the opposed thumb close and reopen.",
            10, sweep, one_close),
    }


def emit_robot(cfg, menagerie: Path, board, hiveboard: Path):
    """Assemble, settle, verify and solve one robot's scene."""
    import mujoco
    import sim_trajectories

    scene = ET.fromstring(SCENE_HEAD.replace("__NAME__", cfg["name"]))
    asset = scene.find("asset")
    worldbody = scene.find("worldbody")

    if "stand" in cfg:
        # A post and a foot, as in the lab: the panel is bolted to a column
        # rather than laid on anything.
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

    # The board's own constraints -- the lamp's thread -- belong to every scene.
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

    # The robot goes in first so its driven joints own the low qpos indices;
    # the widget indexes them positionally.
    worldbody.append(body)
    holder = ET.SubElement(worldbody, "body", {
        "name": "hiveboard", "pos": fmt(cfg["board"]),
        "quat": fmt(cfg.get("board_quat", BOARD_FLAT))})
    if cfg.get("spin_board"):
        # The stand turns. A short arm cannot reach across a 260 mm board, so
        # each task rotates the board to bring its own module round to the near
        # side first -- which is what you would do with the real thing rather
        # than dragging the robot around it. Stiff enough that nothing the
        # modules do can nudge it off the angle it was set to.
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

    # A tool frame between the fingertips, measured off the jaws.
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
                        {"name": "home", "qpos": fmt(cfg["home"])})
    ET.indent(scene, "  ")
    path.write_text(ET.tostring(scene, encoding="unicode") + "\n")

    settle(cfg, path, key)
    ET.indent(scene, "  ")
    path.write_text(ET.tostring(scene, encoding="unicode") + "\n")

    model = mujoco.MjModel.from_xml_path(str(path))
    print(f"  {cfg['name']:6s} compiles: nq={model.nq} nu={model.nu} "
          f"nbody={model.nbody} nmesh={model.nmesh}")

    cfg = dict(cfg, board_normal=board_normal(cfg))
    tasks = (macao_motions(model) if cfg["name"] == "macao"
             else sim_trajectories.build(path, cfg))
    sim_trajectories.dump(tasks, OUT / f"{cfg['name']}.traj.json")

    # These robots differ in size by a factor of five, so the widget is handed a
    # framing measured off each scene rather than one camera for all of them.
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
        # The index, not the name: looking an actuator up by name in the widget
        # needs mjtObj's numbering, and getting that constant wrong fails
        # silently -- mj_name2id returns -1, ctrl[-1] writes nowhere, and the
        # gripper simply never opens.
        "gripIndex": int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR,
                                           cfg["grip"]["actuator"])),
        "spinIndex": spin_address(model),
        "board": list(cfg["board"]),
        "view": {"centre": [round(v, 4) for v in centre], "span": round(span, 4)},
        "framing": cfg.get("framing", "over"),
        # Which way the board faces, so the widget can put the camera on the
        # side the modules are on rather than behind the panel.
        "boardNormal": [round(v, 4) for v in board_normal(cfg)],
        "tasks": list(tasks),
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
    """Copy the runtime libraries out of node_modules into public/.

    Vite only publishes public/ verbatim, and the widget is a standalone page
    outside the bundle, so it cannot resolve bare specifiers. MuJoCo's 10 MB
    .wasm ships gzipped only: GitHub Pages compresses JavaScript but makes no
    promise about application/wasm, and the widget hands the inflated bytes to
    the module factory as `wasmBinary` anyway.
    """
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


def settle(cfg, path: Path, key):
    """Rewrite the home keyframe with the pose the modules actually rest in.

    Lids and levers start at whatever angle the URDF happened to author, which
    is rarely their resting angle. Dropping them under gravity once here means
    the widget opens on a settled scene instead of on a board that twitches for
    the first second.
    """
    import mujoco

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
        data.qpos[:held] = arm   # hold the robot; only the modules are settling
        data.qvel[:held] = 0
        if grip_qpos >= 0:
            # The hand is part of the robot, not one of the loose modules being
            # settled.  Letting an uncommanded jaw drift against its stop wrote
            # an out-of-range gripper value into the browser's home keyframe.
            data.qpos[grip_qpos] = grip_open
            data.qvel[grip_dof] = 0
            data.ctrl[grip_id] = grip_open
        mujoco.mj_step(model, data)

    if grip_qpos >= 0:
        data.qpos[grip_qpos] = grip_open
    key.set("qpos", fmt(np.concatenate([arm, data.qpos[held:]])))


def manifest():
    """List every runtime file and pre-gzip it, the way tools/compress-models.py does.

    The widget reads manifest.json first and copies each listed file into
    MuJoCo's in-memory filesystem before compiling scene.xml, so the list has
    to name everything the compiler will reach for -- and the manifest itself
    is fetched the same gzipped way as the rest.
    """
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


# ═══════════════════════════════════════════════════════════════════════════
#  MJCF templates
# ═══════════════════════════════════════════════════════════════════════════
# Everything outside the generated asset/body lists. The arm block is lifted
# from mujoco_menagerie's franka_fr3, with the Panda hand from
# franka_emika_panda attached at the flange; both are Apache-2.0.
# Everything every scene shares: solver settings, the shared palette, the
# floor. Robot-specific defaults, actuators and bodies are grafted on top.
SCENE_HEAD = """<mujoco model="hiveboard __NAME__">
  <compiler angle="radian" meshdir="assets" autolimits="true"/>
  <option integrator="implicitfast" timestep="0.002"/>
  <size memory="24M"/>

  <!-- Only used when the scene is opened in a desktop MuJoCo viewer; the widget
       renders through three.js and ignores this block. -->
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

  <!-- The widget draws its own ground and grid in three.js and skips plane
       geoms, so this floor only ever shows up in physics. -->
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
    <!-- Boxes, not the finger mesh: a hull grip on a 12 mm lever is all edge
         contact and slides. Flat pads with high friction actually hold. -->
    <default class="pad">
      <geom type="box" group="3" rgba="0 0 0 0" friction="2 0.05 0.0002"
            solimp="0.95 0.99 0.001" solref="0.005 1"/>
    </default>
  </default>
</default>
"""

# Position servos, matching the gains the trajectories were verified against.
# Spot and the SO-101 arrive from menagerie already position-controlled, so
# every robot in the widget takes joint angles as its command.
FR3_ACTUATORS = """<actuator>
  <position name="fr3_joint1" joint="fr3_joint1" kp="900" kv="55" forcerange="-87 87"/>
  <position name="fr3_joint2" joint="fr3_joint2" kp="900" kv="55" forcerange="-87 87"/>
  <position name="fr3_joint3" joint="fr3_joint3" kp="800" kv="45" forcerange="-87 87"/>
  <position name="fr3_joint4" joint="fr3_joint4" kp="800" kv="45" forcerange="-87 87"/>
  <position name="fr3_joint5" joint="fr3_joint5" kp="300" kv="14" forcerange="-12 12"/>
  <position name="fr3_joint6" joint="fr3_joint6" kp="250" kv="12" forcerange="-12 12"/>
  <position name="fr3_joint7" joint="fr3_joint7" kp="120" kv="6" forcerange="-12 12"/>
  <!-- One command drives both fingers; ctrl is the half-opening in metres. -->
  <position name="gripper" joint="finger_joint1" ctrlrange="0 0.04" kp="1600" kv="60"
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
    ap = argparse.ArgumentParser(description=__doc__)
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
