"""
Варианты неподвижных ножек из TPU для «Колибри» (Mark4 10", луч 11.22 × 8 мм).

У всех вариантов одинаковое седло: обнимает луч, крепится двумя хомутами
4.8 мм через туннель под лучом. Отличается только сама ножка.

Запуск:  python3 leg_designs.py  ->  stl/designs/*.stl
"""
import math
import os

from generate import M, box, union, cyl_y, to_trimesh
from fixed_legs import (HALF, HALF_IN, SADDLE_L, WALL_H, ZIP_X, ZIP_W, ZIP_T,
                        LEG_H, ARM_W, ARM_T, make_fixed_leg)

SEG = 48
TOP = box(-SADDLE_L / 2, SADDLE_L / 2, -HALF, HALF, -4.0, 0.0)


def sph(x, z, r, y=0.0):
    return M.sphere(r, SEG).translate([x, y, z])


def chain(points, hull_extra=None):
    """Гладкая «трубка»: оболочки соседних шаров (x, z, r)."""
    parts = []
    for (x1, z1, r1), (x2, z2, r2) in zip(points, points[1:]):
        parts.append(M.batch_hull([sph(x1, z1, r1), sph(x2, z2, r2)]))
    return union(parts)


def bezier(p0, p1, p2, n):
    out = []
    for i in range(n + 1):
        t = i / n
        x = (1 - t) ** 2 * p0[0] + 2 * (1 - t) * t * p1[0] + t * t * p2[0]
        z = (1 - t) ** 2 * p0[1] + 2 * (1 - t) * t * p1[1] + t * t * p2[1]
        r = p0[2] + (p2[2] - p0[2]) * t
        out.append((x, z, r))
    return out


def finish(leg_body):
    """Седло + ножка, плоская пятка, туннели и канавки под хомуты."""
    walls = union([
        box(-SADDLE_L / 2, SADDLE_L / 2, -HALF, -HALF_IN, -0.01, WALL_H),
        box(-SADDLE_L / 2, SADDLE_L / 2, HALF_IN, HALF, -0.01, WALL_H),
    ])
    leg_body = leg_body ^ box(-80, 80, -50, 50, -LEG_H - 30, -1.0)   # ножка не заходит в луч
    part = union([TOP, walls, leg_body])
    cuts = [box(-80, 80, -50, 50, -LEG_H - 30, -LEG_H)]
    for x in ZIP_X:
        cuts.append(box(x - ZIP_W / 2, x + ZIP_W / 2, -HALF - 1, HALF + 1, -1.0 - ZIP_T, -1.0))
        for s in (-1, 1):
            y0, y1 = sorted((s * (HALF - 1.0), s * (HALF + 1)))
            cuts.append(box(x - ZIP_W / 2, x + ZIP_W / 2, y0, y1, -1.0 - ZIP_T, WALL_H + 1))
    return part - union(cuts)


# ------------------------------------------------------------------ варианты
def cone():
    """1. Конус — базовый."""
    return make_fixed_leg()


def skid():
    """2. Лыжа — полоз вдоль луча на двух стойках."""
    zb = -LEG_H + 3.5
    bar = union([
        M.batch_hull([sph(-26, zb, 4.2), sph(26, zb, 4.2)]),
        M.batch_hull([sph(-26, zb, 4.2), sph(-33, zb + 5, 3.2)]),
        M.batch_hull([sph(26, zb, 4.2), sph(33, zb + 5, 3.2)]),
    ])
    struts = union([
        M.batch_hull([box(-19, -9, -5, 5, -4.5, -3.5), sph(-20, zb + 1, 3.6)]),
        M.batch_hull([box(9, 19, -5, 5, -4.5, -3.5), sph(20, zb + 1, 3.6)]),
    ])
    return finish(union([bar, struts]))


def talon():
    """3. Коготь — изогнутая лапка колибри, палец вперёд, шпора назад."""
    main = chain(bezier((0, -8, 6.5), (-2, -34, 5.5), (18, -LEG_H + 4.2, 4.2), 14))
    base = M.batch_hull([box(-12, 12, -6, 6, -4.5, -3.5), sph(0, -8, 6.8)])
    spur = chain(bezier((3, -LEG_H + 9, 4.2), (-4, -LEG_H + 5, 3.8), (-12, -LEG_H + 3.4, 3.4), 6))
    toe = sph(22, -LEG_H + 3.6, 3.6)
    return finish(union([base, main, spur, toe]))


def tripod():
    """4. Тренога — три тонкие стойки и шар-пятка."""
    foot = sph(0, -LEG_H + 6.5, 6.5)
    tops = [(-14, 0), (11, -5.5), (11, 5.5)]
    legs = [M.batch_hull([sph(x, -5, 3.2, y), sph(0, -LEG_H + 8, 3.0)]) for x, y in tops]
    ring = union([M.batch_hull([sph(x, -5, 3.2, y) for x, y in tops])])
    return finish(union([foot, ring] + legs))


def spring():
    """5. Рессора — С-образная пружина, пятка под центром седла."""
    w = 2 * HALF
    t = 3.2      # полутолщина ленты
    pts = [(10, -4), (17, -13), (17, -26), (10, -36), (0, -LEG_H + t), (-10, -LEG_H + t)]
    parts = [M.batch_hull([box(4, 16, -HALF, HALF, -4.5, -3.5), cyl_y(t, -w / 2, w / 2, 10, -5)])]
    for (x1, z1), (x2, z2) in zip(pts, pts[1:]):
        parts.append(M.batch_hull([cyl_y(t, -w / 2, w / 2, x1, z1), cyl_y(t, -w / 2, w / 2, x2, z2)]))
    return finish(union(parts))


DESIGNS = [
    ("cone", "Конус", cone, "saddle"),
    ("skid", "Лыжа", skid, "skid"),
    ("talon", "Коготь", talon, "saddle"),
    ("tripod", "Тренога", tripod, "saddle"),
    ("spring", "Рессора", spring, "side"),
]


def print_pose(part, how):
    if how == "saddle":        # седлом на стол
        p = part.rotate([180, 0, 0])
    elif how == "side":        # на боку: плоская грань y = -HALF на стол
        p = part.rotate([-90, 0, 0])
    else:                      # лыжей на стол
        p = part
    x0, y0, z0, x1, y1, z1 = p.bounding_box()
    return p.translate([-(x0 + x1) / 2, -(y0 + y1) / 2, -z0])


def main():
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "stl", "designs")
    os.makedirs(out, exist_ok=True)
    arm = box(-70, 70, -ARM_W / 2, ARM_W / 2, 0, ARM_T)
    for key, name, fn, how in DESIGNS:
        part = fn()
        tm = to_trimesh(print_pose(part, how))
        tm.export(os.path.join(out, f"leg_{key}_TPU_x4.stl"))
        to_trimesh(part).export(os.path.join(out, f"view_{key}.stl"))
        clash = (part ^ arm).volume()
        x0, y0, z0, x1, y1, z1 = part.bounding_box()
        grams = tm.volume / 1000 * 1.21 * 0.42     # TPU, ~20 % gyroid + периметры
        print(f"{name:8s} замкнута={tm.is_watertight} genus={part.genus():2d} "
              f"габарит {x1 - x0:5.1f}×{y1 - y0:4.1f}×{-z0:4.1f} мм  ~{grams:4.1f} г  "
              f"пересечение с лучом {clash:.2f}  низ {z0:.1f}")


if __name__ == "__main__":
    main()
