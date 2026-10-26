#!/usr/bin/env python3
"""
Ubah robot_body.stl (hasil decimasi, frame CAD: Y-atas, depan = -X) menjadi frame ROS/Gazebo:
  X = depan, Y = kiri, Z = atas, origin = di lantai tepat di bawah titik tengah poros roda.
Roda dipisah dari body (roda dibuat silinder di SDF agar terlihat berputar).

Hasil ukur dari CAD:
  pusat roda   : x=0.107, y=0.091 (=> radius 0.035), z kiri/kanan = 0.0475 / 0.234
  lantai       : y = 0.056
"""
import struct
import numpy as np
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
src = ROOT / "models/robot/meshes/robot_body.stl"
cad = ROOT / "models/robot/meshes/robot_body_cad.stl"
if not cad.exists():
    src.rename(cad)

AX_X, GROUND_Y, MID_Z = 0.107, 0.056, 0.141

d = cad.read_bytes()
n = struct.unpack("<I", d[80:84])[0]
tri = np.frombuffer(d[84:], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]), count=n)["v"].reshape(-1, 3, 3).astype(np.float64)

# buang segitiga roda
x, y, z = tri[..., 0], tri[..., 1], tri[..., 2]
wheel = ((x > 0.068) & (y < 0.130) & ((z < 0.066) | (z > 0.220))).all(axis=1)
tri = tri[~wheel]

# transform: X=-(x-AX), Y=z-MID, Z=y-GROUND
out = np.stack([-(tri[..., 0] - AX_X), tri[..., 2] - MID_Z, tri[..., 1] - GROUND_Y], axis=-1)

dst = ROOT / "models/robot/meshes/robot_body.stl"
with open(dst, "wb") as f:
    f.write(b"robot body (gazebo frame)".ljust(80, b" "))
    f.write(struct.pack("<I", len(out)))
    for p0, p1, p2 in out:
        nrm = np.cross(p1 - p0, p2 - p0); l = np.linalg.norm(nrm); nrm = nrm / l if l else nrm
        f.write(struct.pack("<12fH", *nrm, *p0, *p1, *p2, 0))
pts = out.reshape(-1, 3)
print("body tris", len(out), "min", pts.min(0).round(3), "max", pts.max(0).round(3))
