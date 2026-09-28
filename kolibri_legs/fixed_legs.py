"""
Простые неподвижные ножки для «Колибри» (Mark4 10", луч 11.22 × 8 мм).

Одна деталь на луч: седло обнимает луч снизу и с боков, крепится двумя
хомутами 3.6 мм. Хомут идёт поверх луча, по канавкам на щеках и через
туннель под лучом. Печать из TPU 95A: ножка гасит удар и гнётся при
посадке со сносом, а не ломается.

Система координат: X вдоль луча, Y поперёк, Z вверх, низ луча Z=0.

Запуск:  python3 fixed_legs.py   ->  stl/fixed/kolibri_fixed_leg_TPU_x4.stl
"""
import os

import manifold3d as m3d

from generate import M, box, union, to_trimesh

# ---------------- ПАРАМЕТРЫ (мм) ----------------
ARM_W = 11.22        # ширина луча (замер)
ARM_T = 8.0          # высота луча (замер)
CLEAR = 0.4          # зазор посадки
WALL = 3.0           # толщина щёк
WALL_H = 6.0         # высота щёк (хомуту сверху остаётся ARM_T - WALL_H)
SADDLE_L = 40.0      # длина седла вдоль луча
LEG_H = 45.0         # высота ножки: от низа луча до земли
FOOT_R = 7.5         # радиус пятки
FOOT_FLAT = 1.2      # срез пятки снизу — плоская площадка, чтобы дрон стоял ровно
ZIP_W, ZIP_T = 5.2, 2.2   # туннель под хомут 3.6 или 4.8 мм
ZIP_X = (-12.0, 12.0)
# ------------------------------------------------

HALF_IN = (ARM_W + CLEAR) / 2
HALF = HALF_IN + WALL


def make_fixed_leg():
    top = box(-SADDLE_L / 2, SADDLE_L / 2, -HALF, HALF, -4.0, 0.0)
    neck = box(-8.0, 8.0, -6.5, 6.5, -LEG_H + FOOT_R, -LEG_H + FOOT_R + 1.0)
    foot = M.sphere(FOOT_R, 64).translate([0, 0, -LEG_H + FOOT_R - FOOT_FLAT])
    body = M.batch_hull([top, neck, foot])
    walls = union([
        box(-SADDLE_L / 2, SADDLE_L / 2, -HALF, -HALF_IN, -0.01, WALL_H),
        box(-SADDLE_L / 2, SADDLE_L / 2, HALF_IN, HALF, -0.01, WALL_H),
    ])
    part = union([body, walls])

    cuts = [box(-50, 50, -50, 50, -LEG_H - 20, -LEG_H)]          # плоская пятка
    for x in ZIP_X:
        # туннель под лучом
        cuts.append(box(x - ZIP_W / 2, x + ZIP_W / 2, -HALF - 1, HALF + 1, -1.0 - ZIP_T, -1.0))
        # канавки на щеках, чтобы хомут не выпирал
        for s in (-1, 1):
            y0 = s * (HALF - 1.0)
            cuts.append(box(x - ZIP_W / 2, x + ZIP_W / 2, min(y0, s * (HALF + 1)), max(y0, s * (HALF + 1)),
                            -1.0 - ZIP_T, WALL_H + 1))
    return part - union(cuts)


def export():
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "stl", "fixed")
    os.makedirs(out, exist_ok=True)
    leg = make_fixed_leg()
    # печать седлом вниз: форма сужается вверх, поддержки не нужны
    printed = leg.rotate([180, 0, 0])
    x0, y0, z0, x1, y1, z1 = printed.bounding_box()
    printed = printed.translate([-(x0 + x1) / 2, -(y0 + y1) / 2, -z0])
    tm = to_trimesh(printed)
    path = os.path.join(out, "kolibri_fixed_leg_TPU_x4.stl")
    tm.export(path)
    # превью на луче
    arm = box(-60, 60, -ARM_W / 2, ARM_W / 2, 0, ARM_T)
    to_trimesh(union([leg, arm])).export(os.path.join(out, "preview_fixed_leg_on_arm.stl"))
    e = tm.extents
    print(f"kolibri_fixed_leg_TPU_x4.stl  {e[0]:.1f} x {e[1]:.1f} x {e[2]:.1f} мм  "
          f"замкнута={tm.is_watertight}  объём={tm.volume / 1000:.1f} см³")
    print("пересечение с лучом:", round((leg ^ arm).volume(), 3), "мм³")


if __name__ == "__main__":
    export()
