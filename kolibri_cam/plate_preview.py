"""Картинка стола из 3mf/Kolibri_cam_tilt_MG90S.3mf: вид сверху + объёмный вид."""
import zipfile, xml.etree.ElementTree as ET
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

ns = {"m": "http://schemas.microsoft.com/3dmanufacturing/core/2015/02"}
f = "3mf/Kolibri_cam_tilt_MG90S.3mf"
root = ET.fromstring(zipfile.ZipFile(f).read("3D/3dmodel.model"))
objs = {}
for o in root.findall(".//m:object", ns):
    v = np.array([[float(e.get(k)) for k in "xyz"] for e in o.findall(".//m:vertex", ns)])
    t = np.array([[int(e.get(k)) for k in ("v1", "v2", "v3")] for e in o.findall(".//m:triangle", ns)])
    objs[o.get("id")] = (o.get("name"), v, t)
placed = []
for it in root.findall(".//m:item", ns):
    tr = [float(x) for x in it.get("transform").split()]
    n, v, t = objs[it.get("objectid")]
    placed.append((n, v + np.array(tr[9:12]), t))
cols = {"frame": (0.25, 0.42, 0.75), "cradle": (0.93, 0.55, 0.18)}
fig = plt.figure(figsize=(16, 7.5))
ax = fig.add_subplot(1, 2, 1)
for n, v, t in placed:
    c = cols["frame" if "frame" in n else "cradle"]
    ax.add_collection(PolyCollection(v[t][:, :, :2], facecolor=c, edgecolor="none"))
    cx = (v.min(0)[0] + v.max(0)[0]) / 2
    ax.text(cx, v.max(0)[1] + 4, n.replace("PETG_", ""), ha="center", fontsize=9)
ax.add_patch(plt.Rectangle((0, 0), 220, 220, fill=False, lw=1.5, ec="#444"))
ax.set_xlim(-5, 225); ax.set_ylim(-5, 225); ax.set_aspect("equal"); ax.set_xticks([]); ax.set_yticks([])
ax.set_title("Стол 220×220, вид сверху (стойки 33 мм между центрами)", fontsize=12)
ax3 = fig.add_subplot(1, 2, 2, projection="3d")
light = np.array([0.4, -0.5, 0.8]); light /= np.linalg.norm(light)
T, C = [], []
for n, v, t in placed:
    tri = v[t]
    nrm = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    nrm /= np.linalg.norm(nrm, axis=1, keepdims=True) + 1e-12
    sh = 0.35 + 0.65 * np.clip(nrm @ light, 0, 1)
    T.append(tri); C.append(np.c_[np.outer(sh, cols["frame" if "frame" in n else "cradle"]), np.ones_like(sh)])
T = np.concatenate(T); C = np.concatenate(C)
ax3.add_collection3d(Poly3DCollection(T, facecolors=C, edgecolor="none"))
V = T.reshape(-1, 3); lo, hi = V.min(0), V.max(0); c = (lo + hi) / 2; r = (hi - lo).max() / 2 * 0.8
ax3.set_xlim(c[0] - r, c[0] + r); ax3.set_ylim(c[1] - r, c[1] + r); ax3.set_zlim(0, 2 * r)
ax3.set_box_aspect((1, 1, 1)); ax3.view_init(30, -55); ax3.set_axis_off()
ax3.set_title("Как лежат при печати (крупно)", fontsize=12)
plt.tight_layout(); plt.savefig("3mf/cam_tilt_plate_preview.png", dpi=85)
print("3mf/cam_tilt_plate_preview.png")
