"""Рисует превью деталей в preview.png (нужен matplotlib)."""
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
ITEMS = [
    ("preview_arm_folded.stl", "Луч: сложено", (15, -60)),
    ("preview_arm_deployed.stl", "Луч: раскрыто, упор 95°", (10, -60)),
    ("kolibri_block_x4.stl", "Колодка (луч 11.22 мм) ×4", (30, -50)),
    ("kolibri_leg_x4.stl", "Ножка ×4", (60, -80)),
    ("kolibri_spool_x1.stl", "Катушка ×1", (35, -40)),
    ("kolibri_servo_mount_x1.stl", "Крепление серво ×1", (35, -40)),
]
fig = plt.figure(figsize=(15, 9.5))
for i, (fn, title, (el, az)) in enumerate(ITEMS):
    tm = trimesh.load(os.path.join(HERE, "stl", fn))
    ax = fig.add_subplot(2, 3, i + 1, projection="3d")
    tris = tm.vertices[tm.faces]
    light = np.array([0.4, -0.5, 0.8]); light /= np.linalg.norm(light)
    shade = 0.35 + 0.65 * np.clip(tm.face_normals @ light, 0, 1)
    col = np.c_[0.22 * shade + 0.1, 0.45 * shade + 0.1, 0.55 * shade + 0.1, np.ones_like(shade)]
    ax.add_collection3d(Poly3DCollection(tris, facecolors=col, edgecolor="none"))
    lo, hi = tm.bounds; c = (lo + hi) / 2; r = (hi - lo).max() / 2
    ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(c[2] - r, c[2] + r)
    ax.set_box_aspect((1, 1, 1)); ax.view_init(el, az); ax.set_axis_off()
    ax.set_title(title, fontsize=13)
plt.tight_layout()
plt.savefig(os.path.join(HERE, "preview.png"), dpi=90)
