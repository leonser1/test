"""Схема качания: 1) объёмный вид с осью и подписями, 2) вид сбоку с положениями 0/30/60/90°."""
import math
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, FancyArrowPatch
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import photo_mount as p
from generate_car import to_trimesh, cyl_y, union

fig = plt.figure(figsize=(19, 8.5))

# ---------- 1. объёмный вид с подписями
ax = fig.add_subplot(1, 2, 1, projection="3d")
light = np.array([0.4, 0.6, 0.75]); light /= np.linalg.norm(light)
cr, sv, cam = p.rotating(35)
screw = union([cyl_y(1.5, p.Y_EAR_R - p.EAR_T - 0.5, p.Y_CR_BOT + 3.5, p.X_P, p.Z_P, 24),
               cyl_y(2.8, p.Y_EAR_R - p.EAR_T - 2.5, p.Y_EAR_R - p.EAR_T + 1.0, p.X_P, p.Z_P, 24)])
items = [(p.make_frame(), p.COL["frame"], 0.55), (p.horn().translate([p.X_P, 0, p.Z_P]), (0.85, 0.85, 0.88), 1),
         (screw, (0.75, 0.75, 0.78), 1), (cr, p.COL["print"], 1), (sv, p.COL["servo"], 1), (cam, p.COL["cam"], 1)]
for man, col, alpha in items:
    tm = to_trimesh(man)
    sh = 0.35 + 0.65 * np.clip(tm.face_normals @ light, 0, 1)
    fc = np.c_[np.outer(sh, col), np.full_like(sh, alpha)]
    ax.add_collection3d(Poly3DCollection(tm.vertices[tm.faces], facecolors=fc, edgecolor="none"))
yA, yB = p.Y_HORN_FACE + p.EAR_T + 14, p.Y_EAR_R - p.EAR_T - 14
ax.plot([p.X_P, p.X_P], [yB, yA], [p.Z_P, p.Z_P], "r--", lw=2.5)
ax.text(p.X_P, yA + 2, p.Z_P + 3, "ось вращения", color="r", fontsize=12, weight="bold")
lab = [((p.X_P + 2, p.Y_HORN_FACE + 16, p.Z_P + 12), "① качалка прикручена к левому уху\n    — НЕ крутится (левая опора)"),
       ((p.X_P + 2, p.Y_EAR_R - 34, p.Z_P + 12), "② винт M3×6 в правом ухе —\n    правая опора, крутится с люлькой"),
       ((p.X_P + 30, 0, -42), "③ корпус серво + люлька + камера\n    поворачиваются вместе вокруг оси"),
       ((-8, -5, 30), "трубки на 2 стойки рамы")]
for (x, y, z), t in lab:
    ax.text(x, y, z, t, fontsize=10, bbox=dict(facecolor="white", alpha=0.8, edgecolor="none"))
ax.set_xlim(-10, 65); ax.set_ylim(-40, 40); ax.set_zlim(-50, 30); ax.set_box_aspect((1, 1, 0.9))
ax.view_init(24, 32); ax.set_axis_off()
ax.set_title("На чём держится и вокруг чего вращается", fontsize=14)

# ---------- 2. вид сбоку: силуэты при 0/30/60/90°
ax2 = fig.add_subplot(1, 2, 2)


def side(man):
    """Силуэт в плоскости XZ (вид слева): переводим Y в Z и проецируем."""
    cs = man.rotate([90, 0, 0]).project()          # (x, y, z) -> (x, -z, y): в проекции вторая ось = -Z
    return [np.array(poly) * [1, -1] for poly in cs.to_polygons()]


for poly in side(p.frame_proxy()):
    ax2.add_patch(Polygon(poly, closed=True, fc=(0.15, 0.15, 0.17), ec="none", alpha=0.6))
for poly in side(p.make_frame()):
    ax2.add_patch(Polygon(poly, closed=True, fc=p.COL["frame"], ec="none", alpha=0.35))
angles = [0, 30, 60, 90]
alphas = [0.25, 0.35, 0.5, 0.95]
for t, al in zip(angles, alphas):
    cr, sv, cam = p.rotating(t)
    for man, col in ((cr, p.COL["print"]), (sv, p.COL["servo"]), (cam, p.COL["cam"])):
        for poly in side(man):
            ax2.add_patch(Polygon(poly, closed=True, fc=col, ec="k", lw=0.3, alpha=al))
    a = math.radians(t)
    lx, lz = p.X_P + (p.CAM_X + 16) * math.cos(a), p.Z_P - (p.CAM_X + 16) * math.sin(a)
    ax2.annotate(f"{t}°", (lx, lz), fontsize=13, weight="bold", ha="center", va="center")
R = p.CAM_X + 8
arc = FancyArrowPatch((p.X_P + R, p.Z_P + 2), (p.X_P - 2, p.Z_P - R), connectionstyle="arc3,rad=-0.42",
                      arrowstyle="-|>", mutation_scale=22, lw=2, color="r")
ax2.add_patch(arc)
ax2.plot(p.X_P, p.Z_P, "o", color="r", ms=9)
ax2.annotate("ось (вал серво / винт M3)", (p.X_P, p.Z_P), (p.X_P - 28, p.Z_P + 18), fontsize=11, color="r",
             arrowprops=dict(arrowstyle="->", color="r"))
ax2.annotate("плиты рамы", (-10, -p.G - 1), (-28, -p.G - 14), fontsize=10, arrowprops=dict(arrowstyle="->"))
ax2.set_xlim(-30, 85); ax2.set_ylim(-62, 30); ax2.set_aspect("equal")
ax2.set_xlabel("вперёд →, мм"); ax2.set_ylabel("вверх, мм"); ax2.grid(alpha=0.2)
ax2.set_title("Вид сбоку: камера качается вперёд-вниз от 0° до 90°", fontsize=14)
plt.tight_layout()
plt.savefig("swing_diagram.png", dpi=85)
print("swing_diagram.png")
