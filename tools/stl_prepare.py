#!/usr/bin/env python3
"""Analisa + konversi robot.stl (ASCII besar) -> binary STL ringan (vertex clustering) + preview 3 view."""
import sys, re, struct
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

src, dst, png = sys.argv[1], sys.argv[2], sys.argv[3]
grid = float(sys.argv[4]) if len(sys.argv) > 4 else 0.002

data = open(src, "rb").read()
nums = re.findall(rb"vertex\s+(\S+)\s+(\S+)\s+(\S+)", data)
v = np.array(nums, dtype=np.float64).reshape(-1, 3, 3)
print("triangles:", len(v))
mn, mx = v.reshape(-1, 3).min(0), v.reshape(-1, 3).max(0)
print("min", mn, "max", mx, "size", mx - mn)

# vertex clustering decimation
q = np.round(v.reshape(-1, 3) / grid).astype(np.int64)
uq, inv = np.unique(q, axis=0, return_inverse=True)
inv = inv.reshape(-1, 3)
ok = (inv[:, 0] != inv[:, 1]) & (inv[:, 1] != inv[:, 2]) & (inv[:, 0] != inv[:, 2])
tris = np.unique(np.sort(inv[ok], axis=1), axis=0, return_index=True)[1]
faces = inv[ok][np.sort(tris)]
verts = uq * grid
print("decimated triangles:", len(faces))

with open(dst, "wb") as f:
    f.write(b"decimated".ljust(80, b" "))
    f.write(struct.pack("<I", len(faces)))
    for a, b, c in faces:
        p0, p1, p2 = verts[a], verts[b], verts[c]
        n = np.cross(p1 - p0, p2 - p0); l = np.linalg.norm(n); n = n / l if l else n
        f.write(struct.pack("<12fH", *n, *p0, *p1, *p2, 0))

pts = v.reshape(-1, 3)[:: max(1, len(v) * 3 // 60000)]
fig, ax = plt.subplots(1, 3, figsize=(15, 5))
for i, (a, b, t) in enumerate([(0, 1, "TOP X-Y"), (0, 2, "FRONT X-Z"), (1, 2, "SIDE Y-Z")]):
    ax[i].scatter(pts[:, a], pts[:, b], s=0.2); ax[i].set_title(t); ax[i].set_aspect("equal"); ax[i].grid(True)
plt.savefig(png, dpi=80)
