#!/usr/bin/env python3
import struct
import sys
from pathlib import Path
import fast_simplification  
import numpy as np

REPO = Path(__file__).resolve().parent.parent
SRC = REPO / "public/models"
OUT = REPO / "public/sim/parts"


KEEP_RATIO = 0.35
DECIMATE_ABOVE = 1200


PARTS = [
    "torque_based/High_Torque_Valve_Base.stl",
    "torque_based/High_Torque_Valve_Screw.stl",
    "torque_based/Small_Valve_Base.stl",
    "torque_based/Small_Valve_Screw.stl",
    "torque_based/M30_Screw_Base.stl",
    "torque_based/M30_Nut.stl",
    "torque_based/M8_Screw_Base.stl",
    "torque_based/M8_Nut.stl",
    "composed_assembly/Shock_Absorber_Base.stl",
    "composed_assembly/Shock_Absorber_Pin.stl",
    "composed_assembly/Shock_Absorber_Screw.stl",
    "precision_based/Button_Base.stl",
    "precision_based/Button.stl",
    "composed_assembly/Button_Cover.stl",
    "composed_assembly/Key_Base.stl",
    "composed_assembly/Key.stl",
    "composed_assembly/Drawer_Base.stl",
    "composed_assembly/Drawer.stl",
]


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


def weld(verts, faces, tol=1e-6):

    keys = np.round(verts / tol).astype(np.int64)
    _, first, inverse = np.unique(keys, axis=0, return_index=True, return_inverse=True)
    order = np.argsort(first)
    remap = np.empty(len(first), np.int64)
    remap[order] = np.arange(len(first))
    faces = remap[inverse.ravel()][faces]
    verts = verts[first[order]]
    keep = ((faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2])
            & (faces[:, 0] != faces[:, 2]))
    return verts, faces[keep]


def stl_write(path, verts, faces):

    tris = verts[faces]
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    lens = np.linalg.norm(n, axis=1, keepdims=True)
    n = np.divide(n, lens, out=np.zeros_like(n), where=lens > 0)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "wb") as fh:
        fh.write(b"\0" * 80)
        fh.write(struct.pack("<I", len(faces)))
        for i in range(len(faces)):
            fh.write(struct.pack("<3f", *n[i]))
            for v in tris[i]:
                fh.write(struct.pack("<3f", *v))
            fh.write(b"\0\0")


def main():
    total_before = total_after = 0
    for rel in PARTS:
        src = SRC / rel
        if not src.exists():
            print(f"  MISSING {rel}")
            continue
        verts, faces = weld(*stl_read(src))
        before = len(faces)
        if before > DECIMATE_ABOVE:
            v, f = fast_simplification.simplify(
                verts.astype(np.float32), faces.astype(np.int32), 1.0 - KEEP_RATIO)
            verts, faces = np.asarray(v, np.float64), np.asarray(f, np.int64)
        after = len(faces)
        stl_write(OUT / rel, verts, faces)
        total_before += before
        total_after += after
        print(f"  {rel:48s} {before:7d} -> {after:6d} tris")
    print(f"\n  total {total_before} -> {total_after} tris "
          f"({100 * total_after / max(total_before, 1):.0f}%)  into {OUT.relative_to(REPO)}")


if __name__ == "__main__":
    sys.exit(main())
