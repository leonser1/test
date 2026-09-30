"""Рисует превью в preview.png (нужен matplotlib)."""
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
ITEMS = [
    ("preview_with_drone.stl", "Модуль под Mark4 10\" (винты — диски)", (22, -60)),
    ("preview_assembly.stl", "Сборка: колёса прямо", (25, -130)),
    ("preview_steering_detail.stl", "Руль: серво → паз → поперечная тяга → кулаки", (40, -145)),
    ("preview_assembly_left.stl", "Серво +30°: поворот влево (сверху)", (89, -90)),
    ("preview_assembly_right.stl", "Серво −30°: поворот вправо (сверху)", (89, -90)),
    ("car_deck_x1.stl", "Палуба: стек, серво, стойки, моторы", (35, -130)),
]
fig = plt.figure(figsize=(15, 9.5))
for i, (fn, title, (el, az)) in enumerate(ITEMS):
    tm = trimesh.load(os.path.join(HERE, "stl", fn))
    if len(tm.faces) > 60000:
        tm = tm.simplify_quadric_decimation(face_count=60000) if hasattr(tm, "simplify_quadric_decimation") else tm
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
print("preview.png")
