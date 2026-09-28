"""
Убирающиеся ножки для дрона «Колибри» на раме Mark4 V3 10" — параметрический генератор STL.

Схема: на середине каждого луча колодка с шарниром. Ножка сложена под лучом
и прижата резинкой. Одна серво в центре наматывает 4 нити на катушку, нити
тянут рычажки ножек, ножки откидываются до упора 95°.

Система координат колодки: X — вдоль луча наружу (к мотору), Y — поперёк луча,
Z — вверх. Нижняя плоскость луча Z=0, ось шарнира в точке X=0, Z=-PIVOT_DEPTH.

Запуск:  python3 generate.py            -> stl/*.stl
         python3 generate.py --check    -> проверка столкновений по всему ходу
Требует: pip install manifold3d numpy trimesh
"""
import math
import os
import sys

import manifold3d as m3d
import numpy as np
import trimesh

M = m3d.Manifold
SEG = 64

# ---------------- ПАРАМЕТРЫ (мм) — рама Mark4 V3 10" ----------------------
# Рама: колёсная база 427 мм (центр–мотор 213.5), луч 7.5 мм, стек 30.5×30.5.
# Колодка ставится на ~120 мм от центра рамы: сложенная ножка с пяткой
# заканчивается на ~197 мм и не доходит до винтов мотора (213.5).
ARM_W = 11.22         # ширина луча в месте установки колодки (замер)
ARM_W_VARIANTS = ()   # доп. колодки под другие ширины, напр. (14, 16, 18)
MIN_HALF = 9.0        # мин. полуширина колодки: штыри резинки не задевают ножку
ARM_T = 8.0           # высота (толщина) луча, замер
ARM_CLEAR = 0.4       # зазор посадки колодки на луч
WALL = 2.4            # толщина боковых щёк колодки
WALL_H = 6.0          # высота щёк, обнимающих луч сбоку
BASE_T = 2.5          # толщина основания колодки под лучом
BLOCK_X0, BLOCK_X1 = -28.0, 40.0   # длина колодки относительно оси шарнира

PIVOT_DEPTH = 7.5     # ось шарнира ниже луча
PIN_D = 3.1           # отверстие в ушах под ось (винт M3)
HUB_R = 4.5           # радиус ступицы ножки
HUB_W = 8.0           # ширина ножки (по Y)
EAR_T = 3.0           # толщина ушей шарнира
EAR_GAP = HUB_W + 0.6
EAR_R = 6.5           # радиус ушей вокруг оси

LEG_LEN = 68.0        # длина стержня ножки от оси до конца (без пятки)
LEG_T = 7.0           # толщина стержня ножки
LEVER_LEN = 12.0      # рычажок под нить
LEVER_ANGLE = -60.0   # направление рычажка относительно ножки (в плоскости XZ)
THREAD_HOLE = 1.8
TOOTH_R = 6.8         # зуб-упор на ступице (упирается в основание на 95°)
TOOTH_ANGLE = -111.5  # подобран проверкой --check, чтобы упор был на ~95°
BAND_AT = 24.0        # проточка под резинку на ножке

EYE_X = -24.0         # ушко-направляющая нити
EYE_Z = -13.5
POST_X = 36.0         # штыри под резинку
M3_X = -12.0          # отверстие M3 сквозь луч (необязательно)
ZIP_X = (-16.0, 24.0) # канавки под хомуты 3.6 мм

SPOOL_R = 8.0         # радиус барабана: ~15 мм нити за ~107° хода серво
SPOOL_FLANGE_R = 12.0

SERVO_L, SERVO_W = 23.2, 12.9     # карман под SG90 / MG90S (с зазором)
SERVO_TAB_H = 20.0                # глубина гнезда: от ушек серво вверх (SG90 ~16, MG90S ~18.5 — с запасом)
SERVO_SCREW_SPACING = 27.8
STACK = 30.5                      # крепёж к раме по шаблону стека (30.5 или 20)
# --------------------------------------------------------------------------

PIV = np.array([0.0, 0.0, -PIVOT_DEPTH])


def box(x0, x1, y0, y1, z0, z1):
    return M.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])


def cyl_y(r, y0, y1, x, z, seg=SEG):
    """Цилиндр вдоль оси Y."""
    return (M.cylinder(y1 - y0, r, r, seg)
            .rotate([-90, 0, 0]).translate([x, y0, z]))


def cyl_x(r, x0, x1, y, z, seg=SEG):
    return (M.cylinder(x1 - x0, r, r, seg)
            .rotate([0, 90, 0]).translate([x0, y, z]))


def cyl_z(r, z0, z1, x=0.0, y=0.0, seg=SEG):
    return M.cylinder(z1 - z0, r, r, seg).translate([x, y, z0])


def union(parts):
    return M.batch_boolean(parts, m3d.OpType.Add)


def radial_bar(angle_deg, r0, r1, width, thick):
    """Брусок в плоскости XZ от оси шарнира в направлении angle (0 = +X, отрицательный — вниз)."""
    b = box(r0, r1, -width / 2, width / 2, -thick / 2, thick / 2)
    # поворот вокруг Y на +a опускает +X вниз, поэтому знак меняем
    return b.rotate([0, -angle_deg, 0])


# ---------------------------------------------------------------- колодка
def make_block(arm_w=None):
    arm_w = ARM_W if arm_w is None else arm_w
    half = max((arm_w + ARM_CLEAR) / 2 + WALL, MIN_HALF)
    wall = half - (arm_w + ARM_CLEAR) / 2
    base = box(BLOCK_X0, BLOCK_X1, -half, half, -BASE_T, 0)
    walls = union([
        box(BLOCK_X0, BLOCK_X1, -half, -half + wall, 0, WALL_H),
        box(BLOCK_X0, BLOCK_X1, half - wall, half, 0, WALL_H),
    ])

    # уши шарнира
    ear_parts = []
    for s in (-1, 1):
        y0 = EAR_GAP / 2 if s > 0 else -EAR_GAP / 2 - EAR_T
        y1 = y0 + EAR_T
        ear = union([
            box(-EAR_R, EAR_R, y0, y1, -PIVOT_DEPTH, -BASE_T + 0.01),
            cyl_y(EAR_R, y0, y1, 0, -PIVOT_DEPTH),
        ])
        ear_parts.append(ear)
    ears = union(ear_parts)
    pin = cyl_y(PIN_D / 2, -half - 1, half + 1, 0, -PIVOT_DEPTH, 32)

    # ушко-направляющая нити
    eye_tab = union([
        box(EYE_X - 2, EYE_X + 2, -4, 4, EYE_Z, -BASE_T + 0.01),
        cyl_x(4, EYE_X - 2, EYE_X + 2, 0, EYE_Z),
    ])
    eye_hole = cyl_x(1.0, EYE_X - 3, EYE_X + 3, 0, EYE_Z, 24)
    # фаски на входе/выходе нити, чтобы не перетиралась
    eye_hole = union([eye_hole,
                      M.cylinder(1.0, 1.8, 1.0, 24).rotate([0, 90, 0]).translate([EYE_X - 2.01, 0, EYE_Z]),
                      M.cylinder(1.0, 1.0, 1.8, 24).rotate([0, 90, 0]).translate([EYE_X + 1.01, 0, EYE_Z])])

    # штыри под резинку (по бокам от сложенной ножки)
    posts = []
    for s in (-1, 1):
        y = s * (half - 2.4)
        posts.append(cyl_z(1.4, -PIVOT_DEPTH, -BASE_T + 0.01, POST_X, y, 32))
        posts.append(cyl_z(2.0, -PIVOT_DEPTH - 1.0, -PIVOT_DEPTH, POST_X, y, 32))

    body = union([base, walls, ears, eye_tab] + posts)

    # канавки под хомуты: снизу основания и снаружи щёк
    cuts = [pin, eye_hole, cyl_z(1.65, -BASE_T - 1, 1, M3_X, 0, 32)]
    for zx in ZIP_X:
        cuts.append(box(zx - 1.9, zx + 1.9, -half - 1, half + 1, -BASE_T - 1, -BASE_T + 0.9))
        cuts.append(box(zx - 1.9, zx + 1.9, -half - 1, -half + 0.8, -BASE_T - 1, WALL_H + 1))
        cuts.append(box(zx - 1.9, zx + 1.9, half - 0.8, half + 1, -BASE_T - 1, WALL_H + 1))
    return body - union(cuts)


# ---------------------------------------------------------------- ножка
def make_leg():
    """Ножка в сложенном положении, ось шарнира в начале координат."""
    hub = cyl_y(HUB_R, -HUB_W / 2, HUB_W / 2, 0, 0)
    bar = box(0, LEG_LEN, -HUB_W / 2, HUB_W / 2, -LEG_T / 2, LEG_T / 2)
    lever = union([
        radial_bar(LEVER_ANGLE, 0, LEVER_LEN, HUB_W, 5.0),
        cyl_y(2.5, -HUB_W / 2, HUB_W / 2,
              LEVER_LEN * math.cos(math.radians(LEVER_ANGLE)), LEVER_LEN * math.sin(math.radians(LEVER_ANGLE))),
    ])
    tooth = radial_bar(TOOTH_ANGLE, 0, TOOTH_R, HUB_W, 4.5)
    body = union([hub, bar, lever, tooth])

    lx = (LEVER_LEN - 1.5) * math.cos(math.radians(LEVER_ANGLE))
    lz = (LEVER_LEN - 1.5) * math.sin(math.radians(LEVER_ANGLE))
    cuts = [
        cyl_y((PIN_D + 0.15) / 2, -HUB_W, HUB_W, 0, 0, 32),       # ось
        cyl_y(THREAD_HOLE / 2, -HUB_W, HUB_W, lx, lz, 24),        # нить
        # проточка под резинку снизу и по бокам ножки
        box(BAND_AT - 1.5, BAND_AT + 1.5, -HUB_W, HUB_W, -LEG_T / 2 - 1, -LEG_T / 2 + 0.8),
        box(BAND_AT - 1.5, BAND_AT + 1.5, -HUB_W, -HUB_W / 2 + 0.8, -LEG_T, LEG_T),
        box(BAND_AT - 1.5, BAND_AT + 1.5, HUB_W / 2 - 0.8, HUB_W, -LEG_T, LEG_T),
    ]
    return body - union(cuts)


def place_leg(leg, theta_deg):
    return leg.rotate([0, theta_deg, 0]).translate(PIV.tolist())


# ---------------------------------------------------------------- пятка TPU
def make_foot():
    px, pz = HUB_W + 0.3, LEG_T + 0.3
    outer = M.batch_hull([
        box(-px / 2 - 2.2, px / 2 + 2.2, -pz / 2 - 2.2, pz / 2 + 2.2, 4, 12),
        M.sphere(pz / 2 + 2.2, SEG).translate([0, 0, pz / 2 + 1.5]),
    ])
    pocket = box(-px / 2, px / 2, -pz / 2, pz / 2, 4, 13)
    return outer - pocket


# ---------------------------------------------------------------- катушка
def make_spool():
    """Двухуровневый барабан.

    Нижний ярус: пара лучей 45° и 225°, верхний: 135° и 315° (0° = метка на
    верхнем фланце). Нити одной пары приходят с противоположных сторон и
    наматываются в одну сторону, каждая занимает ~120° своей половины
    барабана, поэтому они не перехлёстываются.

    Привязка: на каждом ярусе одно сквозное отверстие через ось барабана.
    Обе нити пары заводятся в него с двух сторон, выводятся в центральный
    канал и завязываются там узлом — натяжение каждой нити регулируется узлом.
    Катушка висит на серво верхним фланцем к земле. Намотка идёт при повороте
    против часовой стрелки, если смотреть на брюхо перевёрнутого дрона.
    Если серво крутит не туда — реверс серво.
    """
    base = 5.0            # основание-фланец: в нём карман под ступицу качалки
    fl = 1.5
    g = 3.0
    z_g0 = base                     # нижний ярус
    z_f1 = z_g0 + g                 # средний фланец
    z_g1 = z_f1 + fl                # верхний ярус
    z_f2 = z_g1 + g                 # верхний фланец
    top = z_f2 + fl
    body = union([
        cyl_z(SPOOL_FLANGE_R, 0, base),
        cyl_z(SPOOL_FLANGE_R, z_f1, z_f1 + fl),
        cyl_z(SPOOL_FLANGE_R, z_f2, top),
        cyl_z(SPOOL_R, 0, top),
    ])

    cuts = [
        cyl_z(3.9, -1, 4.0),              # ступица качалки серво
        cyl_z(2.9, -1, top + 1),          # отвёртка к винту серво + место под узлы
    ]
    for a in (0, 90, 180, 270):          # саморезы через качалку (глухие, в основание)
        for r in (6.0, 10.0):
            cuts.append(cyl_z(0.8, -1, base - 0.8, r * math.cos(math.radians(a)), r * math.sin(math.radians(a)), 16))
    # сквозное отверстие через ось на каждом ярусе. Нить к лучу phi сходит с
    # барабана в точке phi+90 и привязана на 15° дальше по ходу намотки
    # (phi+105); нить к противоположному лучу — в phi-75, та же ось отверстия.
    for zc, phi in ((z_g0 + g / 2, 45), (z_g1 + g / 2, 135)):
        anchor = phi - 75
        cuts.append(cyl_x(0.65, -SPOOL_R - 1, SPOOL_R + 1, 0, zc, 16).rotate([0, 0, anchor]))
    # метка 0° на верхнем фланце: при сложенных ножках смотрит вдоль оси 0° рамы
    cuts.append(M.cylinder(fl + 2, 1.2, 1.2, 3).translate([SPOOL_FLANGE_R, 0, z_f2 - 1]))
    return body - union(cuts)


# ---------------------------------------------------------------- крепление серво
def make_servo_mount():
    plate_t = 2.5
    half = STACK / 2 + 4.5
    plate = box(-half, half, -half, half, -plate_t, 0)
    ox, oy = SERVO_L / 2 + 2, SERVO_W / 2 + 2
    sleeve = box(-ox, ox, -oy, oy, -SERVO_TAB_H, 0)
    wx = SERVO_SCREW_SPACING / 2 + 3
    # «уши» под винты серво со скосом 45° к пластине — печатаются без поддержек
    wings = M.batch_hull([
        box(-wx, wx, -oy, oy, -SERVO_TAB_H, -SERVO_TAB_H + 7),
        box(-ox, ox, -oy, oy, -SERVO_TAB_H + 7 + (wx - ox), -SERVO_TAB_H + 7 + (wx - ox) + 0.1),
    ])
    body = union([plate, sleeve, wings])
    cuts = [box(-SERVO_L / 2, SERVO_L / 2, -SERVO_W / 2, SERVO_W / 2, -SERVO_TAB_H - 1, 1)]
    for sx in (-1, 1):
        cuts.append(cyl_z(0.8, -SERVO_TAB_H - 1, -SERVO_TAB_H + 6, sx * SERVO_SCREW_SPACING / 2, 0, 16))
        for sy in (-1, 1):
            cuts.append(cyl_z(1.65, -plate_t - 1, 1, sx * STACK / 2, sy * STACK / 2, 32))
    # вырезы под провод серво с обоих торцов (у разных серво провод выходит по-разному)
    for sx in (-1, 1):
        cuts.append(box(sx * (SERVO_L / 2 - 1) - (0 if sx > 0 else ox - SERVO_L / 2 + 2),
                        sx * (SERVO_L / 2 - 1) + (ox - SERVO_L / 2 + 2 if sx > 0 else 0), -3, 3, -8, 1))
    return body - union(cuts)


# ---------------------------------------------------------------- экспорт
def to_trimesh(man):
    mesh = man.to_mesh()
    return trimesh.Trimesh(vertices=np.asarray(mesh.vert_properties)[:, :3],
                           faces=np.asarray(mesh.tri_verts), process=True)


def on_bed(man):
    """Сдвигает деталь так, чтобы она лежала на столе принтера (min Z = 0)."""
    x0, y0, z0, x1, y1, z1 = man.bounding_box()
    return man.translate([-(x0 + x1) / 2, -(y0 + y1) / 2, -z0])


def export(out_dir):
    os.makedirs(os.path.join(out_dir, "blocks") if ARM_W_VARIANTS else out_dir, exist_ok=True)
    parts = {
        # ориентация под печать
        "kolibri_leg_x4.stl": on_bed(make_leg().rotate([90, 0, 0])),       # плашмя: слои вдоль ножки
        "kolibri_foot_TPU_x4.stl": on_bed(make_foot().rotate([180, 0, 0])),  # открытым карманом на стол
        "kolibri_spool_x1.stl": on_bed(make_spool()),
        "kolibri_servo_mount_x1.stl": on_bed(make_servo_mount().rotate([180, 0, 0])),
    }
    parts["kolibri_block_x4.stl"] = on_bed(make_block().rotate([90, 0, 0]))
    # колодки под другие ширины луча: на боку, ось шарнира вертикально
    for w in ARM_W_VARIANTS:
        parts[f"blocks/kolibri_block_arm{w}mm_x4.stl"] = on_bed(make_block(w).rotate([90, 0, 0]))
    # сборки одного луча для просмотра (не для печати)
    block, leg = make_block(), make_leg()
    foot = make_foot().rotate([0, 0, 90]).rotate([0, -90, 0]).translate([LEG_LEN + 4, 0, 0])  # карманом на конец ножки
    leg_with_foot = union([leg, foot])
    parts["preview_arm_folded.stl"] = union([block, place_leg(leg_with_foot, 0)])
    parts["preview_arm_deployed.stl"] = union([block, place_leg(leg_with_foot, 95)])
    for name, man in parts.items():
        tm = to_trimesh(man)
        tm.export(os.path.join(out_dir, name))
        ext = tm.extents
        print(f"{name:42s} {ext[0]:6.1f} x {ext[1]:6.1f} x {ext[2]:6.1f} мм  "
              f"watertight={tm.is_watertight}")


def check():
    block, leg = make_block(), make_leg()
    arm = box(-60, 80, -ARM_W / 2, ARM_W / 2, 0, ARM_T)
    fixed = union([block, arm])
    first_hit = None
    for t10 in range(-50, 1200, 5):
        th = t10 / 10
        v = (fixed ^ place_leg(leg, th)).volume()
        if v > 0.01 and th >= 0 and first_hit is None:
            first_hit = th
        if th in (-5.0, 0.0, 45.0, 90.0, 95.0, 100.0):
            print(f"theta={th:6.1f}°  пересечение={v:7.3f} мм³")
    print(f"первое касание при раскрытии: {first_hit}°")

    def pt(theta, ang, r):
        a = math.radians(ang - theta)
        return np.array([r * math.cos(a), -PIVOT_DEPTH + r * math.sin(a)])
    eye = np.array([EYE_X, EYE_Z])
    l0 = np.linalg.norm(pt(0, LEVER_ANGLE, LEVER_LEN - 1.5) - eye)
    l1 = np.linalg.norm(pt(95, LEVER_ANGLE, LEVER_LEN - 1.5) - eye)
    tip = pt(95, 0, LEG_LEN + 5)
    print(f"ход нити {l0 - l1:.1f} мм -> поворот катушки {math.degrees((l0 - l1) / SPOOL_R):.0f}°")
    print(f"кончик пятки на 95°: x={tip[0]:.1f}, z={tip[1]:.1f} мм от низа луча")


if __name__ == "__main__":
    if "--check" in sys.argv:
        check()
    else:
        export(os.path.join(os.path.dirname(os.path.abspath(__file__)), "stl"))
