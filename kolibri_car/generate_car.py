"""
Колёсный модуль «Колибри-кар» под раму Mark4 10" — параметрический генератор STL.

Модуль крепится снизу к нижней плите дрона 4 болтами M3 (отверстия 52.6 × 37).
На нём: второй полётник (стек 30.5×30.5 или 20×20), серво руля, два передних
поворотных кулака с тягами, два задних мотора (такие же, как на дроне, 2806–2807,
крепление 16×16 и 19×19). Задние колёса надеваются на вал мотора, как пропеллер.
Колесо сделано стаканом: колокол мотора прячется внутрь колеса.

Система координат: X вперёд, Y влево, Z вверх. Z=0 — верх палубы модуля.
Нижняя плита дрона на Z = TOWER_H. Центр рамы над X=0, Y=0.

Запуск:  python3 generate_car.py           -> stl/*.stl
         python3 generate_car.py --check   -> проверка столкновений и зоны винтов
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

# ---------------- ПАРАМЕТРЫ (мм) --------------------------------------------
# Рама Mark4 10": база 427 мм -> моторы дрона в (±151, ±151), винт 10" = R 127.
DRONE_MOTOR = 151.0
PROP_R = 127.0

MOUNT_L, MOUNT_S = 52.6, 37.0   # болты нижней плиты (замер). Палуба: 52.6 вдоль X (вперёд)
ADAPTER_T = 5.0                 # переходник, если у рамы 52.6 идёт поперёк (вариант B)
TOWER_H = 42.0                  # от верха палубы до низа нижней плиты дрона
TOWER_R = 5.0
DECK_T = 4.0

AXLE_Z = 20.0                   # высота осей над палубой
REAR_X, FRONT_X = -50.0, 50.0   # колёсная база 100: всё вне дисков винтов
# колесо (стакан): наружный торец ступицы на Y = WHEEL_OUT_Y
WHEEL_OUT_Y = 60.0
WHEEL_W = 20.0                  # ширина обода
TIRE_R, TIRE_IN_R, TIRE_W = 35.0, 24.8, 16.6
RIM_R, RIM_IN_R, FLANGE_R = 25.0, 21.5, 27.0
HUB_T = 6.0                     # толщина диска ступицы
BELL_BOSS_R, BELL_BOSS_T = 6.0, 1.5   # выемка под бортик вала на колоколе

# задний мотор 2806/2807
MOTOR_WALL_Y = 18.0             # внутренняя сторона стенки мотора
MOTOR_WALL_T = 4.0
MOTOR_LEN = 32.0                # от лапы мотора до верха колокола (замерить свой!)
MOTOR_BELL_R = 17.5
MOTOR_CENTER_D = 12.0           # отверстие под стопорное кольцо вала
MOTOR_HOLE_D = 3.3

# передний поворотный кулак
KP_Y = 30.0                     # шкворень (ось поворота)
KP_POST_R, KP_BORE_R, KP_SLEEVE_R = 4.0, 4.2, 7.5
KP_TOP = 27.8                   # верх стойки шкворня
SLEEVE_Z0, SLEEVE_Z1 = 0.5, 27.5
ARM_L = 15.0                    # рычаг рулевой тяги
ARM_Z0, ARM_Z1 = 23.5, 27.5
ARM_HOLE_D = 2.2                # M2 болт + гайка (или шаровой наконечник M2)
AXLE_BOSS_HALF = 7.0            # полудиагональ ромба оси
BEARING_D, BEARING_W = 16.1, 5.0   # 625ZZ (5×16×5), 2 шт на колесо
STEER_MAX = 30.0                # требуемый угол поворота колёс

# серво руля MG90S / SG90
SERVO_L, SERVO_W = 23.2, 12.9
SERVO_SHAFT_OFF = 5.5
SERVO_SCREW_SPACING = 27.8
SERVO_TAB_Z = 15.5              # верх опор под ушки (сервопривод стоит на палубе)
HORN_L = 13.0                   # рабочее отверстие качалки от вала

# стек второго полётника
STACK_30 = 30.5
STACK_20 = 20.0
# -----------------------------------------------------------------------------

# геометрия рулевой трапеции (Аккерман: рычаг смотрит на центр задней оси)
_dx, _dy = REAR_X - FRONT_X, -KP_Y
_n = math.hypot(_dx, _dy)
ARM_DIR = (_dx / _n, _dy / _n)
ARM_END = (FRONT_X + ARM_L * ARM_DIR[0], KP_Y + ARM_L * ARM_DIR[1])
SERVO_SHAFT_X = ARM_END[0] + HORN_L
TIE_ROD_L = round(ARM_END[1], 1)          # тяга вдоль Y от качалки к рычагу
WHEEL_IN_Y = WHEEL_OUT_Y - WHEEL_W


def box(x0, x1, y0, y1, z0, z1):
    return M.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])


def cyl_z(r, z0, z1, x=0.0, y=0.0, seg=SEG):
    return M.cylinder(z1 - z0, r, r, seg).translate([x, y, z0])


def cyl_y(r, y0, y1, x, z, seg=SEG):
    return M.cylinder(y1 - y0, r, r, seg).rotate([-90, 0, 0]).translate([x, y0, z])


def union(parts):
    parts = [p for p in parts if not p.is_empty()]
    return M.batch_boolean(parts, m3d.OpType.Add)


def hex_prism(flats, z0, z1, x=0.0, y=0.0):
    r = flats / math.sqrt(3)
    return M.cylinder(z1 - z0, r, r, 6).translate([x, y, z0])


def mirror_y(man):
    return man.mirror([0, 1, 0])


def tower_positions():
    return [(sx * MOUNT_L / 2, sy * MOUNT_S / 2) for sx in (-1, 1) for sy in (-1, 1)]


# ---------------------------------------------------------------- палуба
def make_deck():
    z0 = -DECK_T
    wall_x0, wall_x1 = REAR_X - 21, REAR_X + 21
    spine = box(wall_x0, FRONT_X - 6, -22, 22, z0, 0)
    body = [spine]

    # передняя поперечина со стойками шкворней
    pads = [cyl_z(8.5, z0, 0, FRONT_X, s * KP_Y) for s in (-1, 1)]
    body.append(M.batch_hull([box(FRONT_X - 8.5, FRONT_X + 8.5, -20, 20, z0, 0)] + pads))
    for s in (-1, 1):
        body.append(cyl_z(KP_POST_R, 0, KP_TOP, FRONT_X, s * KP_Y))

    # площадка под серво
    s_cx = SERVO_SHAFT_X + SERVO_SHAFT_OFF
    s_x0, s_x1 = s_cx - SERVO_L / 2, s_cx + SERVO_L / 2
    body.append(box(s_x0 - 5, s_x1 + 5, -9, 9, z0, 0))
    servo_posts = []
    for sx, (a, b) in ((s_x0, (s_x0 - 4.6, s_x0 - 0.4)), (s_x1, (s_x1 + 0.4, s_x1 + 4.6))):
        servo_posts.append(box(a, b, -5, 5, 0, SERVO_TAB_Z))
    body += servo_posts

    # стенки задних моторов
    for s in (-1, 1):
        y0, y1 = (MOTOR_WALL_Y, MOTOR_WALL_Y + MOTOR_WALL_T)
        wall = box(wall_x0, wall_x1, y0, y1, 0, AXLE_Z + 18)
        if s < 0:
            wall = mirror_y(wall)
        body.append(wall)
        for gx in (wall_x0, wall_x1 - 3):
            g = (M.hull_points([[gx, MOTOR_WALL_Y + 0.01, 0], [gx + 3, MOTOR_WALL_Y + 0.01, 0],
                                [gx, 8, 0], [gx + 3, 8, 0],
                                [gx, MOTOR_WALL_Y + 0.01, 30], [gx + 3, MOTOR_WALL_Y + 0.01, 30]]))
            body.append(g if s > 0 else mirror_y(g))

    # стойки крепления к раме + перемычки
    tps = tower_positions()
    for (x, y) in tps:
        body.append(cyl_z(TOWER_R + 1.5, z0, 0, x, y))            # пятно под стойкой
        body.append(cyl_z(TOWER_R, 0, TOWER_H, x, y))
        body.append(M.cylinder(4.0, TOWER_R + 3, TOWER_R, SEG).translate([x, y, 0]))
    for sgn in (-1, 1):
        x = sgn * MOUNT_L / 2
        body.append(box(x - 2, x + 2, -MOUNT_S / 2, MOUNT_S / 2, 0, TOWER_H))
        for y in (-MOUNT_S / 2, MOUNT_S / 2):   # косынки наружу
            xs = x + sgn * 2
            body.append(M.hull_points([[xs, y - 1.5, 0], [xs, y + 1.5, 0],
                                       [xs + sgn * 10, y - 1.5, 0], [xs + sgn * 10, y + 1.5, 0],
                                       [xs, y - 1.5, 25], [xs, y + 1.5, 25]]))

    # бобышки стека
    for p, r in ((STACK_30, 3.6), (STACK_20, 2.8)):
        for sx in (-1, 1):
            for sy in (-1, 1):
                body.append(cyl_z(r, 0, 4, sx * p / 2, sy * p / 2))
    deck = union(body)

    cuts = []
    # отверстия моторов (16×16 квадрат и 19×19 ромбом)
    for s in (-1, 1):
        ya, yb = s * (MOTOR_WALL_Y - 8), s * (MOTOR_WALL_Y + MOTOR_WALL_T + 1)
        y0, y1 = min(ya, yb), max(ya, yb)
        cuts.append(cyl_y(MOTOR_CENTER_D / 2, y0, y1, REAR_X, AXLE_Z))
        for dx, dz in [(8, 8), (8, -8), (-8, 8), (-8, -8)] + [(13.435, 0), (-13.435, 0), (0, 13.435), (0, -13.435)]:
            cuts.append(cyl_y(MOTOR_HOLE_D / 2, y0, y1, REAR_X + dx, AXLE_Z + dz, 24))
    # шкворни M3 насквозь, гайка сверху; снизу утопленная головка
    for s in (-1, 1):
        cuts.append(cyl_z(1.65, z0 - 1, KP_TOP + 1, FRONT_X, s * KP_Y, 24))
        cuts.append(cyl_z(3.0, z0 - 1, z0 + 2.0, FRONT_X, s * KP_Y, 32))
    # серво: пилоты M2
    for sx in (s_x0 - 2.4, s_x1 + 2.4):
        cuts.append(cyl_z(0.9, 4, SERVO_TAB_Z + 1, sx, 0, 16))
    # стойки: M3 сверху, гайка в боковом пазу
    for (x, y) in tps:
        cuts.append(cyl_z(1.65, TOWER_H - 14, TOWER_H + 1, x, y, 24))
        hz0, hz1 = TOWER_H - 7.0, TOWER_H - 4.2
        nut = hex_prism(5.8, hz0, hz1, x, y).rotate([0, 0, 0])
        ox = math.copysign(1, x)   # паз открыт вперёд/назад, наружу
        slot = box(min(x, x + ox * 10), max(x, x + ox * 10), y - 2.9, y + 2.9, hz0, hz1)
        cuts += [nut, slot]
    # стек: M3 и M2 насквозь, гайки снизу
    for sx in (-1, 1):
        for sy in (-1, 1):
            x, y = sx * STACK_30 / 2, sy * STACK_30 / 2
            cuts.append(cyl_z(1.65, z0 - 1, 5, x, y, 24))
            cuts.append(hex_prism(5.8, z0 - 1, z0 + 2.5, x, y))
            x, y = sx * STACK_20 / 2, sy * STACK_20 / 2
            cuts.append(cyl_z(1.1, z0 - 1, 5, x, y, 16))
            cuts.append(hex_prism(4.1, z0 - 1, z0 + 2.0, x, y))
    # окна под провода: между стенками моторов и под стеком
    cuts.append(box(REAR_X - 13, REAR_X + 13, -11, 11, z0 - 1, 1))
    cuts.append(box(-6, 6, -6, 6, z0 - 1, 1))
    # прорези под хомуты (провода моторов, XT30)
    for x in (-20, 34):
        for s in (-1, 1):
            cuts.append(box(x - 1.5, x + 1.5, s * 11 - 2, s * 11 + 2, z0 - 1, 1))
    return deck - union(cuts)


def make_adapter_b():
    """Переходник для рамы, у которой 52.6 идёт поперёк. Низ z=0 ложится на стойки палубы.
    Снизу: 4 болта M3 в стойки палубы (головки утоплены сверху).
    Сверху: 4 болта рамы, гайки M3 в шестигранниках снизу переходника."""
    t = ADAPTER_T
    deck_pts = tower_positions()
    frame_pts = [(sx * MOUNT_S / 2, sy * MOUNT_L / 2) for sx in (-1, 1) for sy in (-1, 1)]
    bosses = [cyl_z(TOWER_R + 0.5, 0, t, x, y) for x, y in deck_pts + frame_pts]
    plate = M.batch_hull(bosses)
    cuts = [box(-12, 12, -12, 12, -1, t + 1)]           # окно под провода
    for x, y in deck_pts:
        cuts.append(cyl_z(1.65, -1, t + 1, x, y, 24))
        cuts.append(cyl_z(3.0, t - 3.2, t + 1, x, y, 32))  # головка M3 заподлицо
    for x, y in frame_pts:
        cuts.append(cyl_z(1.65, -1, t + 1, x, y, 24))
        cuts.append(hex_prism(5.8, -1, 2.6, x, y))          # гайка снизу
    return plate - union(cuts)


# ---------------------------------------------------------------- кулак
def make_knuckle():
    """Левый кулак (Y>0) в координатах машины при прямых колёсах."""
    x, y = FRONT_X, KP_Y
    sleeve = cyl_z(KP_SLEEVE_R, SLEEVE_Z0, SLEEVE_Z1, x, y)
    ex, ey = ARM_END
    arm = M.batch_hull([cyl_z(6.0, ARM_Z0, ARM_Z1, x, y), cyl_z(3.8, ARM_Z0, ARM_Z1, ex, ey)])
    # торец-шайба упирается во внутреннее кольцо внутреннего подшипника
    boss_y1 = WHEEL_OUT_Y - (2 * BEARING_W + 0.8) - 0.2 - 0.8
    d = AXLE_BOSS_HALF
    diamond = (M.cube([2 * d / math.sqrt(2)] * 2 + [boss_y1 - y], True)
               .rotate([0, 0, 45]).rotate([-90, 0, 0])
               .translate([x, (y + boss_y1) / 2, AXLE_Z]))
    diamond = diamond ^ box(x - d, x + d, y, boss_y1, SLEEVE_Z0, AXLE_Z + d)
    # диамант оси срастается с гильзой через ребро
    web = box(x - 3, x + 3, y, y + KP_SLEEVE_R + 3, SLEEVE_Z0, SLEEVE_Z1)
    tip = cyl_y(4.0, boss_y1 - 0.01, boss_y1 + 0.8, x, AXLE_Z, 32)   # шайба-упор на внутреннее кольцо
    part = union([sleeve, arm, diamond, web, tip])

    cuts = [cyl_z(KP_BORE_R, SLEEVE_Z0 - 1, SLEEVE_Z1 + 1, x, y)]
    # M5 ось, отверстие-капля (острием вниз: печать вверх ногами)
    ax = M.batch_hull([cyl_y(2.65, y + KP_BORE_R + 1.2, boss_y1 + 2, x, AXLE_Z, 32),
                       box(x - 0.3, x + 0.3, y + KP_BORE_R + 1.2, boss_y1 + 2, AXLE_Z - 3.7, AXLE_Z - 3.1)])
    cuts.append(ax)
    ny0 = boss_y1 - 9.0
    cuts.append(box(x - 4.15, x + 4.15, ny0, ny0 + 4.3, AXLE_Z - 4.8, AXLE_Z + d + 1))
    cuts.append(cyl_z(ARM_HOLE_D / 2, ARM_Z0 - 1, ARM_Z1 + 1, ex, ey, 16))
    return part - union(cuts)


def place_knuckle(man, angle, side=1):
    """Поворот узла вокруг шкворня на angle° (+ = влево)."""
    m = man.translate([-FRONT_X, -KP_Y, 0]).rotate([0, 0, angle]).translate([FRONT_X, KP_Y, 0])
    return m if side > 0 else mirror_y(m)


# ---------------------------------------------------------------- колёса
def make_rim(front):
    """Обод-стакан. Локально: ось Z, z=0 — наружный торец (на стол), z растёт внутрь машины."""
    rim = union([
        cyl_z(RIM_R, 0, WHEEL_W),
        cyl_z(FLANGE_R, 0, 1.5),
        M.cylinder(2.0, RIM_R, FLANGE_R, SEG).translate([0, 0, WHEEL_W - 3.5]),
        cyl_z(FLANGE_R, WHEEL_W - 1.5, WHEEL_W),
    ])
    rim = rim - cyl_z(RIM_IN_R, HUB_T, WHEEL_W + 1)
    cuts = []
    if front:
        rim = union([rim, cyl_z(BEARING_D / 2 + 3, 0, 2 * BEARING_W + 0.8)])
        cuts += [cyl_z(BEARING_D / 2, -1, BEARING_W),
                 cyl_z(BEARING_D / 2, BEARING_W + 0.8, 2 * BEARING_W + 1.8),
                 cyl_z(6.5, -1, 20)]
    else:
        cuts += [cyl_z(2.65, -1, HUB_T + 1, seg=32),
                 cyl_z(BELL_BOSS_R, HUB_T - BELL_BOSS_T, HUB_T + 1)]
    for i in range(6):
        a = math.radians(i * 60 + 30)
        cuts.append(cyl_z(3.5, -1, HUB_T + 1, 15 * math.cos(a), 15 * math.sin(a), 32))
    return rim - union(cuts)


def make_tire():
    t = cyl_z(TIRE_R, 0, TIRE_W) - cyl_z(TIRE_IN_R, -1, TIRE_W + 1)
    # скругление кромок
    t = t ^ M.batch_hull([cyl_z(TIRE_R - 1.5, 0, TIRE_W), cyl_z(TIRE_R, 1.5, TIRE_W - 1.5)])
    grooves = []
    n = 28
    for i in range(n):
        a = 360.0 * i / n
        for k, (z0, z1) in enumerate(((1.8, TIRE_W / 2 - 0.8), (TIRE_W / 2 + 0.8, TIRE_W - 1.8))):
            g = box(TIRE_R - 2.0, TIRE_R + 1, -1.3, 1.3, z0, z1)
            grooves.append(g.rotate([0, 0, a + (360.0 / n / 2 if k else 0)]))
    return t - union(grooves)


def place_wheel(man, x, side=1):
    """Колесо в координаты машины: z-локальная -> -Y (для левой стороны)."""
    m = man.rotate([90, 0, 0]).translate([x, WHEEL_OUT_Y, AXLE_Z])
    return m if side > 0 else mirror_y(m)


def make_tie_rod(length):
    rod = M.batch_hull([cyl_z(3.5, 0, 3), cyl_z(3.5, 0, 3, length, 0)])
    rod = rod - union([cyl_z(ARM_HOLE_D / 2, -1, 4, 0, 0, 16), cyl_z(ARM_HOLE_D / 2, -1, 4, length, 0, 16)])
    return rod


# ---------------------------------------------------------------- макеты для просмотра
def motor_proxy(side=1):
    y0 = MOTOR_WALL_Y + MOTOR_WALL_T
    m = union([cyl_y(MOTOR_BELL_R, y0, y0 + MOTOR_LEN, REAR_X, AXLE_Z),
               cyl_y(2.5, y0 + MOTOR_LEN, y0 + MOTOR_LEN + 12, REAR_X, AXLE_Z, 24)])
    return m if side > 0 else mirror_y(m)


def servo_proxy():
    cx = SERVO_SHAFT_X + SERVO_SHAFT_OFF
    body = box(cx - 11.4, cx + 11.4, -6.1, 6.1, 0, 22.5)
    tabs = box(cx - 16.2, cx + 16.2, -6.1, 6.1, SERVO_TAB_Z + 0.3, SERVO_TAB_Z + 2.8)
    horn = M.batch_hull([cyl_z(3.5, 26, 28, SERVO_SHAFT_X, 0), cyl_z(2.2, 26, 28, ARM_END[0], 0)])
    return union([body, tabs, horn, cyl_z(2.4, 22.5, 26, SERVO_SHAFT_X, 0)])


def stack_proxy():
    return union([box(-18, 18, -18, 18, 8, 9.6), box(-20, 20, -20, 20, 17, 18.6)])


def drone_proxy():
    z = TOWER_H
    plate = box(-60, 60, -30, 30, z, z + 2)
    arms = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            L = DRONE_MOTOR * math.sqrt(2)
            a = box(0, L, -5.6, 5.6, z + 3, z + 11).rotate([0, 0, math.degrees(math.atan2(sy, sx))])
            arms.append(a)
            arms.append(cyl_z(17, z + 11, z + 40, sx * DRONE_MOTOR, sy * DRONE_MOTOR))
            arms.append(cyl_z(PROP_R, z + 42, z + 43, sx * DRONE_MOTOR, sy * DRONE_MOTOR, 96))
    return union([plate] + arms)


def assembly(steer=0.0, with_drone=False):
    deck = make_deck()
    kn = make_knuckle()
    rim_f, rim_r, tire = make_rim(True), make_rim(False), make_tire()
    tire_placed = tire.translate([0, 0, 1.7])
    fw = union([rim_f, tire_placed])
    rw = union([rim_r, tire_placed])
    parts = [deck, servo_proxy(), stack_proxy()]
    for s in (-1, 1):
        a = steer
        front = union([kn, place_wheel(fw, FRONT_X)])
        parts.append(place_knuckle(front, a, s))
        parts.append(place_wheel(rw, REAR_X, s))
        parts.append(motor_proxy(s))
    if with_drone:
        parts.append(drone_proxy())
    return union(parts)


# ---------------------------------------------------------------- проверки
def check():
    ok = True
    deck = make_deck()
    kn = make_knuckle()
    fw = union([make_rim(True), make_tire().translate([0, 0, 1.7])])
    front = union([kn, place_wheel(fw, FRONT_X)])
    fixed_env = union([servo_proxy(), stack_proxy()])
    print(f"Рулевая трапеция: рычаг конец X={ARM_END[0]:.1f} Y={ARM_END[1]:.1f}, "
          f"вал серво X={SERVO_SHAFT_X:.1f}, тяга ≈ {TIE_ROD_L} мм между отверстиями")
    for name, d in (("", deck),):
        max_free = None
        for ang in np.arange(0, 45.1, 1.0):
            hit = False
            for s in (-1, 1):
                for a in (ang, -ang):
                    k = place_knuckle(front, a, s)
                    v = (k ^ d).volume() + (k ^ fixed_env).volume()
                    if v > 0.5:
                        hit = True
            if hit:
                break
            max_free = ang
        flag = "OK" if max_free is not None and max_free >= STEER_MAX else "МАЛО"
        ok &= flag == "OK"
        print(f"Палуба: колёса поворачиваются без касаний до ±{max_free:.0f}° [{flag}]")
    # задние колёса и стенки
    rw = union([make_rim(False), make_tire().translate([0, 0, 1.7])])
    for s in (-1, 1):
        v = (place_wheel(rw, REAR_X, s) ^ deck).volume()
        print(f"Заднее колесо {'Л' if s > 0 else 'П'} vs палуба: {v:.2f} мм³")
        ok &= v < 0.5
        v = (motor_proxy(s) ^ deck).volume()
        print(f"Мотор {'Л' if s > 0 else 'П'} vs палуба: {v:.2f} мм³")
        ok &= v < 0.5
    v = (servo_proxy() ^ deck).volume() + (stack_proxy() ^ deck).volume()
    print(f"Серво и стек vs палуба: {v:.2f} мм³")
    ok &= v < 0.5
    # зазор до земли
    ground = AXLE_Z - TIRE_R
    print(f"Земля на Z={ground:.1f}; низ палубы Z={-DECK_T:.1f} -> клиренс {-DECK_T - ground:.1f} мм; "
          f"нижняя плита дрона над землёй {TOWER_H - ground:.1f} мм")
    print(f"Верх колокола мотора Z={AXLE_Z + MOTOR_BELL_R:.1f}, до нижней плиты {TOWER_H - AXLE_Z - MOTOR_BELL_R:.1f} мм")
    # зона винтов: доля проекции модуля внутри дисков винтов
    asm = assembly(0)
    proj = asm.project()
    area = proj.area()
    disks = None
    for sx in (-1, 1):
        for sy in (-1, 1):
            c = m3d.CrossSection.circle(PROP_R, 128).translate([sx * DRONE_MOTOR, sy * DRONE_MOTOR])
            disks = c if disks is None else disks + c
    inside = (proj ^ disks).area()
    disk_area = 4 * math.pi * PROP_R ** 2
    print(f"Проекция модуля: {area / 100:.0f} см², под винтами {inside / 100:.1f} см² "
          f"= {100 * inside / disk_area:.2f} % площади дисков")
    return ok


# ---------------------------------------------------------------- экспорт
def to_trimesh(man):
    mesh = man.to_mesh()
    return trimesh.Trimesh(vertices=np.asarray(mesh.vert_properties)[:, :3],
                           faces=np.asarray(mesh.tri_verts), process=True)


def on_bed(man):
    x0, y0, z0, x1, y1, z1 = man.bounding_box()
    return man.translate([-(x0 + x1) / 2, -(y0 + y1) / 2, -z0])


def export(out_dir):
    os.makedirs(out_dir, exist_ok=True)
    kn = make_knuckle()
    parts = {
        "car_deck_x1.stl": on_bed(make_deck()),
        "car_adapter_B_optional_x1.stl": on_bed(make_adapter_b()),
        # кулак вверх ногами: рычаг на столе, ромб оси печатается без поддержек
        "car_knuckle_L_x1.stl": on_bed(kn.rotate([180, 0, 0])),
        "car_knuckle_R_x1.stl": on_bed(mirror_y(kn).rotate([180, 0, 0])),
        "car_rim_front_625_x2.stl": on_bed(make_rim(True)),
        "car_rim_rear_motor_x2.stl": on_bed(make_rim(False)),
        "car_tire_TPU_x4.stl": on_bed(make_tire()),
    }
    for L in (TIE_ROD_L - 2, TIE_ROD_L, TIE_ROD_L + 2):
        parts[f"tie_rods/car_tie_rod_{L:.1f}mm_x2.stl"] = on_bed(make_tie_rod(L))
    parts["preview_assembly.stl"] = assembly(0)
    parts["preview_assembly_steer30.stl"] = assembly(30)
    parts["preview_with_drone.stl"] = assembly(0, with_drone=True)
    os.makedirs(os.path.join(out_dir, "tie_rods"), exist_ok=True)
    for name, man in parts.items():
        tm = to_trimesh(man)
        path = os.path.join(out_dir, name)
        tm.export(path)
        lo, hi = tm.bounds
        print(f"{name:42s} {hi[0]-lo[0]:6.1f} × {hi[1]-lo[1]:6.1f} × {hi[2]-lo[2]:5.1f} мм  "
              f"{tm.volume / 1000:6.1f} см³  watertight={tm.is_watertight}")


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    if "--check" in sys.argv:
        sys.exit(0 if check() else 1)
    export(os.path.join(here, "stl"))
