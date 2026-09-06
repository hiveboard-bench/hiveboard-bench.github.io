"""Convert the Isaac ANYmal-D / DynaArm / 2F-140 assembly to browser MJCF.

USD joint frames, rather than authored world poses, define the zero pose.
Meshes include instance proxies and are baked into their rigid body's frame.
This is a fixed-base manipulation model, with gravity compensation and convex
mesh contacts; it does not reproduce PhysX drives or locomotion dynamics.
"""
from pathlib import Path
from urllib.parse import urljoin
from urllib.request import urlretrieve
import math
import xml.etree.ElementTree as ET

import numpy as np

ASSET_ROOT = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0/Isaac/"
ANYMAL_USD = "IsaacLab/Robots/ANYbotics/ANYmal-D/anymal_d.usd"
GRIPPER_USD = "Robots/Robotiq/2F-140/Robotiq_2F_140_physics_edit.usd"
ARM_JOINTS = ["dynaarm_" + n for n in (
    "shoulder_rotation", "shoulder_flexion", "elbow_flexion",
    "forearm_rotation", "wrist_flexion", "wrist_rotation")]


def fetch_layers(url, path):
    """Cache USD composition dependencies, preserving relative asset paths.

    MDL shaders/textures are not needed: the web viewer uses solid materials.
    """
    from pxr import Sdf
    if not path.exists():
        path.parent.mkdir(parents=True, exist_ok=True)
        print(f"  downloading {url}", flush=True)
        temporary = path.with_suffix(path.suffix + ".download")
        urlretrieve(url, temporary)
        temporary.replace(path)
    layer = Sdf.Layer.FindOrOpen(str(path))
    for ref in layer.GetExternalReferences():
        if Path(ref).suffix.lower() in (".usd", ".usda", ".usdc"):
            fetch_layers(urljoin(url, ref), (path.parent / ref).resolve())
    return path


def pose_matrix(pos, quat):
    from pxr import Gf
    matrix = Gf.Matrix4d(1)
    matrix.SetRotate(Gf.Quatd(quat))
    matrix.SetTranslateOnly(Gf.Vec3d(*pos))
    return matrix


def pose_attrs(matrix):
    from pxr import Gf
    t = Gf.Transform(matrix)
    q = t.GetRotation().GetQuat()
    return {"pos": fmt(t.GetTranslation()), "quat": fmt((q.GetReal(), *q.GetImaginary()))}


def fmt(values):
    return " ".join(f"{float(v):.9g}" for v in values)


# The USD ships ANYmal's livery as texture maps, which the web viewer does not
# load, so a straight conversion arrives uniformly grey. These colours are
# sampled from the photographs of the real platform in assets-src/anymal.png
# and public/assets/posters/: a red body shell, a black carbon DynaArm, light
# grey hip actuators, and a near-black frame, feet and gripper. The names are
# load-bearing too -- hiveboard-sim.html reads its PBR parameters out of the
# material name, keying on "shell", "dark" and "metal".
PALETTE = {
    "anymal_red_shell":    "0.76 0.13 0.14 1",   # body covers
    "anymal_shell":        "0.78 0.78 0.80 1",   # hip actuators, thighs
    "anymal_dark":         "0.13 0.13 0.13 1",   # frame, shanks
    "anymal_foot_dark":    "0.06 0.06 0.06 1",   # rubber feet
    "dynaarm_carbon_dark": "0.10 0.09 0.09 1",   # carbon arm links
    "dynaarm_joint_metal": "0.17 0.17 0.18 1",   # arm joint housings
    "robotiq_dark_metal":  "0.09 0.09 0.10 1",   # gripper
}


def materials():
    return [ET.Element("material", {"name": name, "rgba": rgba})
            for name, rgba in PALETTE.items()]


def material_name(prim, component):
    """Which palette entry this geom belongs to, from its prim path."""
    path = str(prim.GetPath()).lower()
    if component == "gripper" or any(s in path for s in ("robotiq", "knuckle", "finger")):
        return "robotiq_dark_metal"
    if component == "arm" or "dynaarm" in path:
        return "dynaarm_joint_metal" if "wrist_2" in path else "dynaarm_carbon_dark"
    if "foot" in path:
        return "anymal_foot_dark"
    if "hip" in path:
        return "anymal_shell"
    if "thigh" in path or "shank" in path:
        return "anymal_dark"
    # The body: the outer covers are the red ones, everything else is frame.
    return "anymal_red_shell" if "shell" in path or "cover" in path else "anymal_dark"


class Converter:
    def __init__(self, out, simplify, write_obj):
        self.out, self.simplify, self.write_obj = out, simplify, write_obj
        self.meshes = []
        self.equality = ET.Element("equality")
        self.out.mkdir(parents=True, exist_ok=True)

    def component(self, path, root_name, component, frozen=None):
        from pxr import Gf, Usd, UsdGeom, UsdPhysics
        stage = Usd.Stage.Open(str(path))
        if not math.isclose(UsdGeom.GetStageMetersPerUnit(stage), 1, abs_tol=1e-6):
            raise ValueError(f"Expected metre USD units: {path}")
        prims = list(stage.Traverse(Usd.TraverseInstanceProxies()))
        rigid = {p.GetPath(): p for p in prims if p.HasAPI(UsdPhysics.RigidBodyAPI)}
        joints, loops = {}, []
        for prim in prims:
            if not prim.IsA(UsdPhysics.Joint):
                continue
            joint = UsdPhysics.Joint(prim)
            if not joint.GetJointEnabledAttr().Get():
                continue
            parents, children = joint.GetBody0Rel().GetTargets(), joint.GetBody1Rel().GetTargets()
            if not parents or not children:  # standalone world weld is replaced by our mount
                continue
            if joint.GetExcludeFromArticulationAttr().Get():
                loops.append(joint)
                continue
            if children[0] in joints:
                raise ValueError(f"Multiple tree parents for {children[0]}")
            joints[children[0]] = (parents[0], joint)

        cache = UsdGeom.XformCache()
        bodies = {p: ET.Element("body", name=prim.GetName(), gravcomp="1") for p, prim in rigid.items()}
        root_path = next(p for p in rigid if p.name == root_name)
        for p, prim in rigid.items():
            body = bodies[p]
            mass = UsdPhysics.MassAPI(prim)
            inertia = mass.GetDiagonalInertiaAttr().Get()
            if mass.GetMassAttr().Get() and inertia is not None and min(inertia) > 0:
                q = mass.GetPrincipalAxesAttr().Get()
                if q.GetLength() < 1e-8:  # USD's unauthored principal-axis sentinel
                    q = Gf.Quatf(1)
                ET.SubElement(body, "inertial", mass=str(mass.GetMassAttr().Get()),
                              pos=fmt(mass.GetCenterOfMassAttr().Get()),
                              quat=fmt((q.GetReal(), *q.GetImaginary())), diaginertia=fmt(inertia))
            if p in joints:
                parent, joint = joints[p]
                m0 = pose_matrix(joint.GetLocalPos0Attr().Get(), joint.GetLocalRot0Attr().Get())
                m1 = pose_matrix(joint.GetLocalPos1Attr().Get(), joint.GetLocalRot1Attr().Get())
                angle = (frozen or {}).get(joint.GetPrim().GetName())
                rotation = Gf.Matrix4d(1)
                revolute = joint.GetPrim().IsA(UsdPhysics.RevoluteJoint)
                if revolute:
                    axis = Gf.Vec3d(*{"X": (1, 0, 0), "Y": (0, 1, 0), "Z": (0, 0, 1)}[
                        joint.GetPrim().GetAttribute("physics:axis").Get()])
                    if angle is not None:
                        rotation.SetRotate(Gf.Rotation(axis, math.degrees(angle)))
                    else:
                        attrs = dict(name=joint.GetPrim().GetName(), type="hinge",
                                     pos=fmt(joint.GetLocalPos1Attr().Get()),
                                     axis=fmt(m1.TransformDir(axis)), damping="0.1", armature="0.002")
                        limits = [joint.GetPrim().GetAttribute("physics:" + k + "Limit").Get()
                                  for k in ("lower", "upper")]
                        if all(math.isfinite(v) for v in limits):
                            attrs["range"] = fmt(np.radians(limits))
                        ET.SubElement(body, "joint", attrs)
                elif not joint.GetPrim().IsA(UsdPhysics.FixedJoint):
                    raise ValueError(f"Unsupported joint: {joint.GetPath()}")
                # USD uses row vectors: child->joint, joint motion, joint->parent.
                body.attrib.update(pose_attrs(m1.GetInverse() * rotation * m0))
                bodies[parent].append(body)
            elif p != root_path:
                raise ValueError(f"Disconnected rigid body {p}")

            inv_body = cache.GetLocalToWorldTransform(prim).GetInverse()
            for geom in Usd.PrimRange(prim, Usd.TraverseInstanceProxies()):
                if not geom.IsA(UsdGeom.Gprim):
                    continue
                owner = geom.GetParent()
                while owner and owner.GetPath() not in rigid:
                    owner = owner.GetParent()
                if owner.GetPath() != p:
                    continue
                local = cache.GetLocalToWorldTransform(geom) * inv_body
                collision = False
                cursor = geom
                while cursor and cursor.GetPath().HasPrefix(p):
                    if cursor.HasAPI(UsdPhysics.CollisionAPI):
                        collision = UsdPhysics.CollisionAPI(cursor).GetCollisionEnabledAttr().Get()
                        break
                    cursor = cursor.GetParent()
                imageable = UsdGeom.Imageable(geom)
                visible = imageable.ComputeVisibility() != "invisible" and imageable.ComputePurpose() != "guide"
                if not collision and not visible:
                    continue
                attrs = {"material": material_name(geom, component), "group": "2",
                         "contype": "0", "conaffinity": "0", "mass": "0"}
                if geom.IsA(UsdGeom.Mesh):
                    mesh = UsdGeom.Mesh(geom)
                    points = np.asarray(mesh.GetPointsAttr().Get(), dtype=float)
                    matrix = np.asarray(local)
                    points = points @ matrix[:3, :3] + matrix[3, :3]
                    indices = np.asarray(mesh.GetFaceVertexIndicesAttr().Get())
                    faces, offset = [], 0
                    for count in mesh.GetFaceVertexCountsAttr().Get():
                        polygon = indices[offset:offset + count]
                        faces.extend((polygon[0], polygon[k], polygon[k + 1]) for k in range(1, count - 1))
                        offset += count
                    faces = np.asarray(faces, dtype=np.int32)
                    if (mesh.GetOrientationAttr().Get() == "leftHanded") != (np.linalg.det(matrix[:3, :3]) < 0):
                        faces = faces[:, ::-1]
                    points, faces = self.simplify(points, faces)
                    name = f"anymal_{len(self.meshes):03d}_{prim.GetName()}"
                    self.write_obj(self.out / (name + ".obj"), points, faces)
                    self.meshes.append(ET.Element("mesh", name=name, file=f"anymal/{name}.obj"))
                    attrs.update(type="mesh", mesh=name)
                else:
                    transform = Gf.Transform(local)
                    scale = np.abs(transform.GetScale())
                    attrs.update(pose_attrs(local))
                    kind = geom.GetTypeName()
                    if kind == "Cube":
                        attrs.update(type="box", size=fmt(scale * UsdGeom.Cube(geom).GetSizeAttr().Get() / 2))
                    elif kind == "Sphere":
                        attrs.update(type="ellipsoid", size=fmt(scale * UsdGeom.Sphere(geom).GetRadiusAttr().Get()))
                    elif kind in ("Cylinder", "Capsule"):
                        shape = getattr(UsdGeom, kind)(geom)
                        axis = {"X": (1, 0, 0), "Y": (0, 1, 0), "Z": (0, 0, 1)}[shape.GetAxisAttr().Get()]
                        align = Gf.Matrix4d(1).SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), Gf.Vec3d(*axis)))
                        attrs.update(pose_attrs(align * local), type=kind.lower(),
                                     size=fmt((shape.GetRadiusAttr().Get() * max(scale),
                                               shape.GetHeightAttr().Get() / 2 * np.dot(scale, axis))))
                    else:
                        raise ValueError(f"Unsupported geometry {geom.GetPath()}: {kind}")
                if visible:
                    ET.SubElement(body, "geom", attrs)
                if collision:
                    attrs.pop("mass", None)
                    # MuJoCo pairs geoms when (contype1 & conaffinity2) or
                    # (contype2 & conaffinity1). contype 1 with conaffinity 2
                    # pairs with the modules (contype 2, conaffinity 1) and the
                    # board, but never with another ANYmal geom: this assembly
                    # has no contact exclusions, so self-collision jams the arm
                    # against its own shoulder instead of reaching the board.
                    attrs.update(group="3", contype="1", conaffinity="2", friction="2 0.05 0.0002",
                                 solref="0.005 1", solimp="0.95 0.99 0.001")
                    ET.SubElement(body, "geom", attrs)

        for joint in loops:
            # Tree hinges already constrain each linkage to a plane. Closing its
            # pivot with a connect constraint restores the four-bar mechanism.
            ET.SubElement(self.equality, "connect", name=joint.GetPrim().GetName(),
                          body1=joint.GetBody0Rel().GetTargets()[0].name,
                          body2=joint.GetBody1Rel().GetTargets()[0].name,
                          anchor=fmt(joint.GetLocalPos0Attr().Get()), solref="0.005 1")
        return bodies[root_path]


ARM_USD = "source/isaaclab_hiveboard/isaaclab_hiveboard/assets/anymal/usd/dynaarm.usd"


def arm_usd(source_root):
    """The DynaArm USD this conversion needs, or None when it is not there."""
    path = Path(source_root) / ARM_USD
    return path if path.exists() else None


def missing_usd_message(source_root):
    return (f"DynaArm USD not found: {Path(source_root) / ARM_USD}. Pass --isaaclab-repo "
            "to your checkout and spawn ANYmal once in Isaac Lab to generate its arm USD "
            "from the URDF, or build the other robots with --robot.")


def parts(output, source_root, usd_cache, simplify, write_obj):
    source_root = Path(source_root)
    cache = Path(usd_cache)
    arm_path = arm_usd(source_root)
    if arm_path is None:
        raise FileNotFoundError(missing_usd_message(source_root))
    converter = Converter(output / "assets/anymal", simplify, write_obj)
    stance = {f"{leg}_{joint}": value for leg in ("LF", "RF", "LH", "RH")
              for joint, value in (("HAA", 0), ("HFE", 0.4 if leg[1] == "F" else -0.4),
                                   ("KFE", -0.8 if leg[1] == "F" else 0.8))}
    body = converter.component(fetch_layers(ASSET_ROOT + ANYMAL_USD, cache / ANYMAL_USD),
                               "base", "anymal", stance)
    arm = converter.component(arm_path, "arm_mount", "arm")
    arm.set("pos", "0 0 0.12")
    arm.set("quat", "0 0 0 1")
    body.insert(0, arm)  # first six qpos and actuators belong to the arm
    gripper = converter.component(fetch_layers(ASSET_ROOT + GRIPPER_USD, cache / GRIPPER_USD),
                                  "robotiq_base_link", "gripper")
    arm.find('.//body[@name="dynaarm_flange"]').append(gripper)
    for name, ratio in (("right_outer_knuckle_joint", 1), ("left_inner_finger_joint", -1),
                        ("right_inner_finger_joint", -1)):
        ET.SubElement(converter.equality, "joint", joint1=name, joint2="finger_joint",
                      polycoef=f"0 {ratio} 0 0 0", solref="0.005 1")
    actuator = ET.Element("actuator")
    for name in ARM_JOINTS + ["finger_joint"]:
        joint = body.find(f'.//joint[@name="{name}"]')
        limits = joint.get("range") if name != "finger_joint" else "0 0.7"
        ET.SubElement(actuator, "position", name=name, joint=name, kp="200", kv="20",
                      ctrlrange=limits, forcerange="-40 40")
    return body, converter.meshes, materials(), {"actuator": actuator, "equality": converter.equality}
