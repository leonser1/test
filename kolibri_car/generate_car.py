"""
Колёсный модуль «Колибри-кар» под раму Mark4 10" — параметрический генератор STL.

Модуль крепится снизу к нижней плите дрона 4 болтами M3 (отверстия 52.6 × 37).
На нём: второй полётник (стек 30.5×30.5 или 20×20), серво руля, два передних
поворотных кулака с поперечной тягой, два задних мотора (такие же, как на дроне,
2806–2807, крепление 16×16 и 19×19). Каждый мотор крутит своё заднее колесо через
редуктор 1:4: шестерня 12 зубьев на валу мотора (зажата гайкой пропеллера), венец
48 зубьев на колесе. Колесо вращается на двух 625ZZ на неподвижной оси M5.

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
TIRE_R, TIRE_IN_R, TIRE_W = 35.0, 24.8, 14.6   # ровная полка обода 15 мм (1.5..16.5)
RIM_R, RIM_IN_R, FLANGE_R = 25.0, 21.5, 27.0
HUB_T = 6.0                     # толщина диска ступицы

# задний мотор 2806/2807
MOTOR_LEN = 32.0                # от лапы мотора до верха колокола (замерить свой!)
MOTOR_BELL_R = 17.5
MOTOR_CENTER_D = 12.0           # отверстие под стопорное кольцо вала
MOTOR_HOLE_D = 3.3

# редуктор задних колёс 1:4, модуль 1, угол зацепления 20°
GEAR_M = 1.0
PINION_Z, WHEEL_GEAR_Z = 12, 48
PROFILE_SHIFT = 0.3             # +0.3 у шестерни (12 зубьев без подреза), −0.3 у венца
BACKLASH = 0.15                 # утонение каждого зуба по делительной окружности
CENTER_EXTRA = 0.1              # + к межосевому под печатные зубья
PINION_W = 7.0                  # ширина шестерни (зажата между колоколом и гайкой)
GEAR_W = 7.2                    # ширина венца
PLATE_Y0, PLATE_Y1 = 2.0, 7.0   # съёмная плита мотора: внутренняя / наружная сторона
PLATE_CB_D, PLATE_CB_H = 6.2, 1.5   # цековка под головки M3 изнутри
STUB_R = 6.0                    # неподвижная ось колеса (бобышка вокруг M5)
REAR_TUBE_R = 24.0              # ступица между венцом и ободом (гайка вала мотора снаружи)

# передний поворотный кулак
KP_Y = 30.0                     # шкворень (ось поворота)
KP_POST_R, KP_BORE_R, KP_SLEEVE_R = 5.0, 5.1, 8.5   # стойка Ø10: Ø8 ломалась по слоям при посадке (аудит)
FRONT_PAD_EXTRA = 2.0           # передняя поперечина толще палубы вниз (6 мм под кулаками)
KP_TOP = 27.8                   # верх стойки шкворня
SLEEVE_Z0, SLEEVE_Z1 = 0.5, 27.5
ARM_L = 15.0                    # рычаг рулевой тяги
ARM_Z0, ARM_Z1 = 23.5, 27.5
ARM_HOLE_D = 2.0                # M2 болт + гайка; рассверлить Ø2.0 по месту — меньше люфт
AXLE_BOSS_HALF = 7.5           # полудиагональ ромба оси: вершина = верх гильзы (стол при печати)
BEARING_D, BEARING_W = 16.1, 5.0   # 625ZZ (5×16×5), 2 шт на колесо
BRG_GAP = 2.0                   # между подшипниками: проставка 5.3/8 × 2 мм на внутренние кольца
STEER_MAX = 30.0                # требуемый угол поворота колёс

# серво руля MG90S / SG90
SERVO_L, SERVO_W = 23.2, 12.9
SERVO_SHAFT_OFF = 5.5
SERVO_SCREW_SPACING = 27.8
SERVO_TAB_Z = 15.5              # верх опор под ушки (сервопривод стоит на палубе)
HORN_L = 11.0                   # рабочее отверстие качалки от вала (паз тяги берёт 10–13): меньше момент и люфт
HORN_Z = 28.0                   # верх качалки над палубой
SERVO_TRAVEL = 36.0             # ход качалки ± (конечные точки серво, ≈1100–1900 мкс): колёса ≈ 30°/25°
BAR_Z0 = 36.0                   # низ поперечной тяги: качалка с гайкой пальца до 34 мм (MG90S корпус 28.5)
BAR_T = 4.0                     # толще: тяга работает на сжатие

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
BAR_L = 2 * ARM_END[1]                    # поперечная тяга: между отверстиями рычагов
PIN_X0 = SERVO_SHAFT_X - HORN_L            # палец качалки при прямых колёсах
SLOT_X = (SERVO_SHAFT_X - (HORN_L + 2) - 0.5, SERVO_SHAFT_X - (HORN_L - 1) + (HORN_L + 2) * (1 - math.cos(math.radians(45))) + 0.5)
SLOT_R = 1.05                   # паз под палец M2 (Ø2.1, довести надфилем)
WHEEL_IN_Y = WHEEL_OUT_Y - WHEEL_W

GEAR_RATIO = WHEEL_GEAR_Z / PINION_Z
GEAR_A = GEAR_M * (PINION_Z + WHEEL_GEAR_Z) / 2 + CENTER_EXTRA
MOTOR_X = REAR_X - GEAR_A                 # ось мотора позади оси колеса
BELL_TOP_Y = PLATE_Y1 + MOTOR_LEN         # верх колокола = низ шестерни
GEAR_Y0 = BELL_TOP_Y + 0.3
GEAR_Y1 = GEAR_Y0 + GEAR_W
NUT_Y1 = BELL_TOP_Y + PINION_W + 5.0      # конец гайки вала мотора
REAR_WHEEL_IN_Y = NUT_Y1 + 1.5
REAR_WHEEL_OUT_Y = REAR_WHEEL_IN_Y + WHEEL_W
STUB_Y1 = GEAR_Y0 - 0.9                   # торец оси, шайба-упор до GEAR_Y0 - 0.1
PLATE_X0, PLATE_X1 = MOTOR_X - 23, MOTOR_X + 23   # плита мотора с лапками


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


def gear_profile(z, m=GEAR_M, shift=0.0, backlash=BACKLASH, pts=12):
    """Эвольвентное зубчатое колесо (2D), зуб №0 смотрит по +X."""
    pa = math.radians(20)
    rp = m * z / 2
    rb = rp * math.cos(pa)
    ra = rp + m * (1 + shift)
    rf = rp - m * (1.25 - shift)
    s = m * (math.pi / 2 + 2 * shift * math.tan(pa)) - backlash

    def inv(a):
        return math.tan(a) - a
    half = s / (2 * rp) + inv(pa)

    def psi(r):
        return half if r <= rb else half - inv(math.acos(rb / r))
    r0 = max(rb, rf)
    rs = [r0 + (ra - r0) * i / pts for i in range(pts + 1)]
    poly = []
    for i in range(z):
        c = 2 * math.pi * i / z
        prof = [(rf, c - psi(r0))] + [(r, c - psi(r)) for r in rs] + \
               [(r, c + psi(r)) for r in reversed(rs)] + [(rf, c + psi(r0))]
        poly += [(r * math.cos(a), r * math.sin(a)) for r, a in prof]
    return m3d.CrossSection([poly])


def make_pinion():
    """Шестерня 12 зубьев на вал M5 мотора. Локально: z=0 — к колоколу."""
    g = M.extrude(gear_profile(PINION_Z, shift=PROFILE_SHIFT), PINION_W)
    return g - cyl_z(2.6, -1, PINION_W + 1, seg=32)


def mirror_y(man):
    return man.mirror([0, 1, 0])


def tower_positions():
    return [(sx * MOUNT_L / 2, sy * MOUNT_S / 2) for sx in (-1, 1) for sy in (-1, 1)]


# ---------------------------------------------------------------- палуба
def make_deck():
    z0 = -DECK_T
    spine = box(PLATE_X0 - 1, FRONT_X - 6, -22, 22, z0, 0)
    body = [spine]

    # передняя поперечина со стойками шкворней
    zp = z0 - FRONT_PAD_EXTRA
    pads = [cyl_z(KP_SLEEVE_R + 0.5, zp, 0, FRONT_X, s * KP_Y) for s in (-1, 1)]
    body.append(M.batch_hull([box(FRONT_X - 8.5, FRONT_X + 8.5, -20, 20, zp, 0)] + pads))
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

    # задний мост: поперечная стенка с неподвижными осями колёс (M5 внутри бобышек)
    body.append(box(REAR_X - 3, REAR_X + 3, -STUB_Y1, STUB_Y1, z0, AXLE_Z))
    body.append(cyl_y(STUB_R, -STUB_Y1, STUB_Y1, REAR_X, AXLE_Z))
    for s in (-1, 1):
        tip = cyl_y(4.0, STUB_Y1 - 0.01, GEAR_Y0 - 0.1, REAR_X, AXLE_Z, 32)
        g = M.hull_points([[REAR_X + 2.9, STUB_Y1 - 4, z0], [REAR_X + 2.9, STUB_Y1 - 1, z0],
                           [REAR_X + 12, STUB_Y1 - 4, z0], [REAR_X + 12, STUB_Y1 - 1, z0],
                           [REAR_X + 2.9, STUB_Y1 - 4, 15], [REAR_X + 2.9, STUB_Y1 - 1, 15]])
        body += [tip, g] if s > 0 else [mirror_y(tip), mirror_y(g)]

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
    # оси задних колёс: M5 (капля вверх — палуба печатается как лежит), гайка в пазу сверху
    for s in (-1, 1):
        ax = M.batch_hull([cyl_y(2.65, 12, GEAR_Y0 + 1, REAR_X, AXLE_Z, 32),
                           box(REAR_X - 0.3, REAR_X + 0.3, 12, GEAR_Y0 + 1, AXLE_Z + 3.1, AXLE_Z + 3.7)])
        slot = box(REAR_X - 4.15, REAR_X + 4.15, 25.0, 29.3, AXLE_Z - 4.8, AXLE_Z + STUB_R + 1)
        cuts += [ax, slot] if s > 0 else [mirror_y(ax), mirror_y(slot)]
    # лапки плит моторов: M3 вниз, гайки снизу палубы
    for x, y in plate_foot_holes():
        cuts.append(cyl_z(1.65, z0 - 1, 1, x, y, 24))
        cuts.append(hex_prism(5.8, z0 - 1, z0 + 2.5, x, y))
    # шкворни M3 насквозь, гайка сверху; снизу утопленная головка
    for s in (-1, 1):
        cuts.append(cyl_z(1.65, z0 - FRONT_PAD_EXTRA - 1, KP_TOP + 1, FRONT_X, s * KP_Y, 24))
        cuts.append(cyl_z(3.0, z0 - FRONT_PAD_EXTRA - 1, z0 - FRONT_PAD_EXTRA + 2.0, FRONT_X, s * KP_Y, 32))
    # серво: пилоты M2
    for sx in (s_cx - SERVO_SCREW_SPACING / 2, s_cx + SERVO_SCREW_SPACING / 2):
        cuts.append(cyl_z(0.9, 4, SERVO_TAB_Z + 1, sx, 0, 16))
        cuts.append(box(sx - 3, sx + 3, -2.5, 2.5, -0.01, 4.5))     # проход кабеля серво под ушком
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
    # окна под провода: перед задним мостом, между плитами моторов и под стеком
    cuts.append(box(REAR_X + 6, REAR_X + 18, -11, 11, z0 - 1, 1))
    cuts.append(box(MOTOR_X - 12, MOTOR_X + 12, -1.5, 1.5, z0 - 1, 1))
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


def plate_foot_holes():
    return [(x, s * 11.0) for x in (PLATE_X0 + 2, PLATE_X1 - 2) for s in (-1, 1)]


def make_motor_plate():
    """Съёмная плита левого мотора (Y>0). Мотор прикручивается на столе, потом плита
    ставится на палубу 2 винтами M3 сквозь лапки. Головки винтов мотора утоплены
    изнутри, поэтому две плиты встают спинами почти вплотную."""
    x0, x1 = PLATE_X0, PLATE_X1
    plate = box(x0, x1, PLATE_Y0, PLATE_Y1, 0, AXLE_Z + 19)
    parts = [plate]
    for fx0 in (x0, x1 - 4):
        parts.append(box(fx0, fx0 + 4, PLATE_Y0, 15, 0, 4))
        parts.append(M.hull_points([[fx0, PLATE_Y1 - 0.01, 3.9], [fx0 + 4, PLATE_Y1 - 0.01, 3.9],
                                    [fx0, 15, 3.9], [fx0 + 4, 15, 3.9],
                                    [fx0, PLATE_Y1 - 0.01, 32], [fx0 + 4, PLATE_Y1 - 0.01, 32]]))
    part = union(parts)
    cuts = [cyl_y(MOTOR_CENTER_D / 2, PLATE_Y0 - 1, PLATE_Y1 + 1, MOTOR_X, AXLE_Z)]
    for dx, dz in [(8, 8), (8, -8), (-8, 8), (-8, -8), (13.435, 0), (-13.435, 0), (0, 13.435), (0, -13.435)]:
        cuts.append(cyl_y(MOTOR_HOLE_D / 2, PLATE_Y0 - 1, PLATE_Y1 + 1, MOTOR_X + dx, AXLE_Z + dz, 24))
        cuts.append(cyl_y(PLATE_CB_D / 2, PLATE_Y0 - 1, PLATE_Y0 + PLATE_CB_H, MOTOR_X + dx, AXLE_Z + dz, 32))
    for x, y in plate_foot_holes():
        if y > 0:
            cuts.append(cyl_z(1.65, -1, 5, x, y, 24))
    return part - union(cuts)


def make_rear_wheel():
    """Заднее колесо с венцом 48 зубьев. Локально: z=0 — наружный торец (на стол),
    z растёт внутрь машины: обод 0..20, ступица-диск 20..TUBE, венец сверху."""
    tz1 = REAR_WHEEL_OUT_Y - GEAR_Y1           # верх ступицы-трубы = низ венца
    gz1 = REAR_WHEEL_OUT_Y - GEAR_Y0           # верх венца
    rim = union([
        cyl_z(RIM_R, 0, WHEEL_W),
        cyl_z(FLANGE_R, 0, 1.5),
        M.cylinder(2.0, RIM_R, FLANGE_R, SEG).translate([0, 0, WHEEL_W - 3.5]),
        cyl_z(FLANGE_R, WHEEL_W - 1.5, WHEEL_W),
    ]) - cyl_z(RIM_IN_R, -1, WHEEL_W - 5)
    hub = cyl_z(11, 0, WHEEL_W)
    spokes = [box(10, RIM_IN_R + 0.5, -1.5, 1.5, 0, WHEEL_W - 4.99).rotate([0, 0, 30 + 60 * i]) for i in range(6)]
    tube = cyl_z(REAR_TUBE_R, WHEEL_W - 0.01, tz1 + 0.01)
    gear = M.extrude(gear_profile(WHEEL_GEAR_Z, shift=-PROFILE_SHIFT), gz1 - tz1).translate([0, 0, tz1])
    w = union([rim, hub, tube, gear] + spokes)
    cuts = [cyl_z(BEARING_D / 2, -1, BEARING_W),                 # наружный подшипник
            cyl_z(BEARING_D / 2, gz1 - BEARING_W, gz1 + 1),      # внутренний, в венце
            cyl_z(3.2, -1, gz1 + 1, seg=32)]
    for i in range(6):
        a = math.radians(60 * i)
        cuts.append(cyl_z(3.5, WHEEL_W - 6, gz1 + 1, 16 * math.cos(a), 16 * math.sin(a), 32))
    return w - union(cuts)


def place_pinion(pin, side=1):
    m = pin.rotate([-90, 0, 0]).translate([MOTOR_X, BELL_TOP_Y, AXLE_Z])
    return m if side > 0 else mirror_y(m)


def place_rear_wheel(w, side=1, spin=0.0):
    # венец повёрнут на полшага, чтобы зуб шестерни попал во впадину
    m = (w.rotate([0, 0, 180.0 / WHEEL_GEAR_Z + spin]).rotate([90, 0, 0])
         .translate([REAR_X, REAR_WHEEL_OUT_Y, AXLE_Z]))
    return m if side > 0 else mirror_y(m)


# ---------------------------------------------------------------- кулак
def make_knuckle():
    """Левый кулак (Y>0) в координатах машины при прямых колёсах."""
    x, y = FRONT_X, KP_Y
    sleeve = cyl_z(KP_SLEEVE_R, SLEEVE_Z0, SLEEVE_Z1, x, y)
    ex, ey = ARM_END
    arm = M.batch_hull([cyl_z(6.0, ARM_Z0, ARM_Z1, x, y), cyl_z(3.8, ARM_Z0, ARM_Z1, ex, ey)])
    # торец-шайба упирается во внутреннее кольцо внутреннего подшипника
    boss_y1 = WHEEL_OUT_Y - (2 * BEARING_W + BRG_GAP) - 0.2 - 0.8
    d = AXLE_BOSS_HALF
    diamond = (M.cube([2 * d / math.sqrt(2)] * 2 + [boss_y1 - y], True)
               .rotate([0, 0, 45]).rotate([-90, 0, 0])
               .translate([x, (y + boss_y1) / 2, AXLE_Z]))
    diamond = diamond ^ box(x - d, x + d, y, boss_y1, SLEEVE_Z0, AXLE_Z + d)
    # диамант оси срастается с гильзой через ребро
    web = box(x - 3, x + 3, y, y + KP_SLEEVE_R + 3, SLEEVE_Z0, SLEEVE_Z1)
    tip = cyl_y(3.5, boss_y1 - 0.01, boss_y1 + 0.8, x, AXLE_Z, 32)   # шайба-упор Ø7 только на внутреннее кольцо
    part = union([sleeve, arm, diamond, web, tip])

    cuts = [cyl_z(KP_BORE_R, SLEEVE_Z0 - 1, SLEEVE_Z1 + 1, x, y)]
    # M5 ось, отверстие-капля (острием вниз: печать вверх ногами)
    # острие капли только внутри ромба: на шайбе-упоре не остаётся перемычки 0.3 мм
    ax = M.batch_hull([cyl_y(2.65, y + KP_BORE_R + 1.2, boss_y1 - 0.5, x, AXLE_Z, 32),
                       box(x - 0.3, x + 0.3, y + KP_BORE_R + 1.2, boss_y1 - 0.5, AXLE_Z - 3.7, AXLE_Z - 3.1)])
    cuts += [ax, cyl_y(2.65, y + KP_BORE_R + 1.2, boss_y1 + 2, x, AXLE_Z, 32)]
    ny0 = boss_y1 - 9.0
    cuts.append(box(x - 4.15, x + 4.15, ny0, ny0 + 4.3, AXLE_Z - 4.8, AXLE_Z + d + 1))
    cuts.append(cyl_z(ARM_HOLE_D / 2, ARM_Z0 - 1, ARM_Z1 + 1, ex, ey, 16))
    return part - union(cuts)


def place_knuckle(man, angle, side=1):
    """Поворот узла вокруг шкворня на angle° (+ = влево)."""
    m = man.translate([-FRONT_X, -KP_Y, 0]).rotate([0, 0, angle]).translate([FRONT_X, KP_Y, 0])
    return m if side > 0 else mirror_y(m)


# ---------------------------------------------------------------- колёса
def make_rim(front=True):
    """Передний обод-стакан на 2 × 625ZZ. Локально: ось Z, z=0 — наружный торец (на стол), z растёт внутрь машины."""
    rim = union([
        cyl_z(RIM_R, 0, WHEEL_W),
        cyl_z(FLANGE_R, 0, 1.5),
        M.cylinder(2.0, RIM_R, FLANGE_R, SEG).translate([0, 0, WHEEL_W - 3.5]),
        cyl_z(FLANGE_R, WHEEL_W - 1.5, WHEEL_W),
    ])
    rim = rim - cyl_z(RIM_IN_R, HUB_T, WHEEL_W + 1)
    rim = union([rim, cyl_z(BEARING_D / 2 + 3, 0, 2 * BEARING_W + BRG_GAP)])
    cuts = [cyl_z(BEARING_D / 2, -1, BEARING_W),
            cyl_z(BEARING_D / 2, BEARING_W + BRG_GAP, 2 * BEARING_W + BRG_GAP + 1),
            cyl_z(6.5, -1, 20)]
    for i in range(6):
        a = math.radians(i * 60 + 30)
        cuts.append(cyl_z(3.5, -1, HUB_T + 1, 15 * math.cos(a), 15 * math.sin(a), 32))
    return rim - union(cuts)


def make_bearing_spacer():
    """Проставка между внутренними кольцами двух 625ZZ: затяжка оси идёт по ней, а не через шарики."""
    return cyl_z(4.0, 0, BRG_GAP, seg=48) - cyl_z(2.65, -1, BRG_GAP + 1, seg=32)


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


def make_tie_bar():
    """Поперечная рулевая тяга (в координатах машины, колёса прямо).
    Концы — на рычаги кулаков (M2 + гайка). В середине паз вдоль X: в него входит
    палец качалки серво (винт M2). Паз сам выбирает смещение пальца по X при
    повороте качалки и разную длину качалки (11–14 мм)."""
    ex, ey = ARM_END
    z0, z1 = BAR_Z0, BAR_Z0 + BAR_T
    beam = M.batch_hull([cyl_z(3.0, z0, z1, ex, ey), cyl_z(3.0, z0, z1, ex, -ey)])   # уже: дальше от стоек рамы
    beam = union([beam, cyl_z(3.8, z0, z1, ex, ey), cyl_z(3.8, z0, z1, ex, -ey)])
    pad = M.batch_hull([cyl_z(3.6, z0, z1, SLOT_X[0], 0), cyl_z(3.6, z0, z1, SLOT_X[1], 0)])
    # бобышки-проставки вниз до рычагов
    bosses = [cyl_z(3.8, ARM_Z1 + 0.3, z0 + 0.01, ex, sy * ey) for sy in (-1, 1)]
    bar = union([beam, pad] + bosses)
    cuts = [cyl_z(ARM_HOLE_D / 2, ARM_Z1 - 1, z1 + 1, ex, sy * ey, 16) for sy in (-1, 1)]
    cuts.append(M.batch_hull([cyl_z(SLOT_R, z0 - 1, z1 + 1, SLOT_X[0], 0, 16),
                              cyl_z(SLOT_R, z0 - 1, z1 + 1, SLOT_X[1], 0, 16)]))
    return bar - union(cuts)


# ---------------------------------------------------------------- кинематика руля
def _rot(v, deg):
    a = math.radians(deg)
    return (v[0] * math.cos(a) - v[1] * math.sin(a), v[0] * math.sin(a) + v[1] * math.cos(a))


def _arm_end(side, delta):
    kx, ky = FRONT_X, side * KP_Y
    a = (ARM_END[0] - FRONT_X, side * (ARM_END[1] - KP_Y))
    r = _rot(a, delta)
    return (kx + r[0], ky + r[1])


def _bisect(f, lo, hi, n=60):
    flo = f(lo)
    for _ in range(n):
        mid = (lo + hi) / 2
        fm = f(mid)
        if (fm > 0) == (flo > 0):
            lo, flo = mid, fm
        else:
            hi = mid
    return (lo + hi) / 2


def steer_state(delta_l):
    """По углу левого колеса (+ влево) -> угол правого, угол качалки, положение пальца в пазу."""
    el = _arm_end(1, delta_l)
    dr = _bisect(lambda d: math.dist(el, _arm_end(-1, d)) - BAR_L, delta_l - 25, delta_l + 25)
    er = _arm_end(-1, dr)
    mid = ((el[0] + er[0]) / 2, (el[1] + er[1]) / 2)
    u = ((el[0] - er[0]) / BAR_L, (el[1] - er[1]) / BAR_L)
    n = (u[1], -u[0])             # поперёк тяги, вперёд
    bar_ang = math.degrees(math.atan2(u[1], u[0])) - 90.0

    def pin(phi):
        v = _rot((-HORN_L, 0.0), phi)
        return (SERVO_SHAFT_X + v[0], v[1])

    def f(phi):
        t = pin(phi)
        return (t[0] - mid[0]) * u[0] + (t[1] - mid[1]) * u[1]
    phi = _bisect(f, -80, 80)
    t = pin(phi)
    slot_pos = PIN_X0 + (t[0] - mid[0]) * n[0] + (t[1] - mid[1]) * n[1]
    return dict(dl=delta_l, dr=dr, phi=phi, slot=slot_pos, mid=mid, bar_ang=bar_ang, el=el, er=er)


def delta_for_servo(phi_target):
    return _bisect(lambda d: steer_state(d)["phi"] - phi_target, -45, 45)


def place_bar(bar, st):
    m0 = (ARM_END[0], 0.0)
    return (bar.translate([-m0[0], 0, 0]).rotate([0, 0, st["bar_ang"]])
            .translate([st["mid"][0], st["mid"][1], 0]))


def place_knuckle_state(man, delta, side):
    """Кулак стороны side, повернутый на delta (+ = колесо влево)."""
    return place_knuckle(man, delta if side > 0 else -delta, side)


# ---------------------------------------------------------------- макеты для просмотра
def motor_proxy(side=1):
    """Мотор с валом и гайкой (без шестерни)."""
    m = union([cyl_y(MOTOR_BELL_R, PLATE_Y1, BELL_TOP_Y, MOTOR_X, AXLE_Z),
               cyl_y(2.5, BELL_TOP_Y, NUT_Y1 - 1, MOTOR_X, AXLE_Z, 24),
               cyl_y(4.6, BELL_TOP_Y + PINION_W, NUT_Y1, MOTOR_X, AXLE_Z, 6)])
    return m if side > 0 else mirror_y(m)


def servo_body_proxy():
    cx = SERVO_SHAFT_X + SERVO_SHAFT_OFF
    body = box(cx - 11.4, cx + 11.4, -6.1, 6.1, 0, 22.5)
    tabs = box(cx - 16.2, cx + 16.2, -6.1, 6.1, SERVO_TAB_Z + 0.3, SERVO_TAB_Z + 2.8)
    return union([body, tabs, cyl_z(2.4, 22.5, HORN_Z - 2, SERVO_SHAFT_X, 0)])


def horn_proxy(phi=0.0):
    h = union([M.batch_hull([cyl_z(3.5, HORN_Z - 2, HORN_Z, SERVO_SHAFT_X, 0),
                             cyl_z(2.2, HORN_Z - 2, HORN_Z, PIN_X0, 0)]),
               cyl_z(1.0, HORN_Z, BAR_Z0 + BAR_T + 1, PIN_X0, 0, 16),       # палец M2
               hex_prism(4.0, HORN_Z, HORN_Z + 1.6, PIN_X0, 0)])             # гайка пальца
    return h.translate([-SERVO_SHAFT_X, 0, 0]).rotate([0, 0, phi]).translate([SERVO_SHAFT_X, 0, 0])


def servo_proxy(phi=0.0):
    return union([servo_body_proxy(), horn_proxy(phi)])


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


def assembly(servo=0.0, with_drone=False):
    """servo — угол качалки, колёса и тяга ставятся по кинематике."""
    st = steer_state(delta_for_servo(servo)) if servo else steer_state(0.0)
    deck = make_deck()
    kn = make_knuckle()
    rim_f, tire = make_rim(True), make_tire()
    tire_placed = tire.translate([0, 0, 1.7])
    fw = union([rim_f, tire_placed])
    rw = union([make_rear_wheel(), tire_placed])
    plate, pin = make_motor_plate(), make_pinion()
    parts = [deck, servo_proxy(st["phi"]), stack_proxy(), place_bar(make_tie_bar(), st)]
    for s in (-1, 1):
        front = union([kn, place_wheel(fw, FRONT_X)])
        parts.append(place_knuckle_state(front, st["dl"] if s > 0 else st["dr"], s))
        parts.append(place_rear_wheel(rw, s))
        parts.append(motor_proxy(s))
        parts.append(place_pinion(pin, s))
        parts.append(plate if s > 0 else mirror_y(plate))
    if with_drone:
        parts.append(drone_proxy())
    return union(parts)


# ---------------------------------------------------------------- проверки
def steering_check(deck, kn, fw):
    """Серво -> тяга -> оба кулака: углы, Аккерман, паз, касания на всём ходу."""
    ok = True
    bar = make_tie_bar()
    env = union([deck, servo_body_proxy(), stack_proxy()])
    front = union([kn, place_wheel(fw, FRONT_X)])
    wb, tw = FRONT_X - REAR_X, 2 * KP_Y
    print("  качалка  колесо Л  колесо П  идеал внутр.  палец в пазу X   касания")
    for phi in (-SERVO_TRAVEL - 3, -SERVO_TRAVEL, -20, -10, 0, 10, 20, SERVO_TRAVEL, SERVO_TRAVEL + 3):
        d = delta_for_servo(phi)
        st = steer_state(d)
        outer, inner = sorted((abs(st["dl"]), abs(st["dr"])))
        ideal = math.degrees(math.atan(1 / (1 / math.tan(math.radians(outer)) - tw / wb))) if outer > 0.5 else 0
        b = place_bar(bar, st)
        k = union([place_knuckle_state(front, st["dl"], 1), place_knuckle_state(front, st["dr"], -1)])
        hit = (b ^ env).volume() + (b ^ k).volume() * 0 + (k ^ env).volume() + (horn_proxy(st["phi"]) ^ b).volume()
        in_slot = SLOT_X[0] + 1.2 <= st["slot"] <= SLOT_X[1] - 1.2
        # тяга не должна касаться кулаков кроме своих бобышек: проверяем без рычагов
        wheels_hit = (b ^ union([place_knuckle_state(place_wheel(fw, FRONT_X), st["dl"], 1),
                                 place_knuckle_state(place_wheel(fw, FRONT_X), st["dr"], -1)])).volume()
        bad = hit > 0.5 or wheels_hit > 0.5 or not in_slot
        ok &= not bad
        print(f"  {phi:+6.0f}°  {st['dl']:+7.1f}°  {st['dr']:+7.1f}°    {ideal:5.1f}°      "
              f"{st['slot']:5.1f} [{SLOT_X[0] + 1.2:.1f}..{SLOT_X[1] - 1.2:.1f}]  "
              f"{'ЕСТЬ' if bad else 'нет'}")
    # запас от «мёртвой точки»: угол между рычагом и тягой
    st = steer_state(delta_for_servo(SERVO_TRAVEL))
    for side, e, dlt in ((1, st["el"], st["dl"]), (-1, st["er"], st["dr"])):
        a = _rot((ARM_END[0] - FRONT_X, side * (ARM_END[1] - KP_Y)), dlt)
        u = ((st["el"][0] - st["er"][0]) / BAR_L, (st["el"][1] - st["er"][1]) / BAR_L)
        ang = math.degrees(math.acos(abs(a[0] * u[0] + a[1] * u[1]) / math.hypot(*a)))
        print(f"  угол рычаг–тяга {'Л' if side > 0 else 'П'} на краю хода: {ang:.0f}° (мёртвая точка при 0°)")
        ok &= ang > 25
    return ok


def gear_check(deck):
    """Редуктор: зацепление в 2D на полном обороте, касания колеса, мотора, шестерни."""
    ok = True
    pin2 = gear_profile(PINION_Z, shift=PROFILE_SHIFT)
    gear2 = gear_profile(WHEEL_GEAR_Z, shift=-PROFILE_SHIFT)
    worst = 0.0
    step = 360.0 / PINION_Z
    for i in range(41):
        a = step * i / 40
        p = pin2.rotate(a).translate([GEAR_A, 0])
        g = gear2.rotate(180.0 / WHEEL_GEAR_Z - a / GEAR_RATIO)
        worst = max(worst, (p ^ g).area())
    # плотность зацепления: без зазора (CENTER_EXTRA и BACKLASH = 0) зубья должны пересекаться
    tight = 0.0
    p0 = gear_profile(PINION_Z, shift=PROFILE_SHIFT, backlash=-0.3)
    g0 = gear_profile(WHEEL_GEAR_Z, shift=-PROFILE_SHIFT, backlash=-0.3)
    for i in range(21):
        a = step * i / 20
        tight = max(tight, (p0.rotate(a).translate([GEAR_A - CENTER_EXTRA, 0]) ^
                            g0.rotate(180.0 / WHEEL_GEAR_Z - a / GEAR_RATIO)).area())
    pa = math.radians(20)
    ra1 = GEAR_M * (PINION_Z / 2 + 1 + PROFILE_SHIFT)
    ra2 = GEAR_M * (WHEEL_GEAR_Z / 2 + 1 - PROFILE_SHIFT)
    rb1, rb2 = GEAR_M * PINION_Z / 2 * math.cos(pa), GEAR_M * WHEEL_GEAR_Z / 2 * math.cos(pa)
    a0 = GEAR_M * (PINION_Z + WHEEL_GEAR_Z) / 2
    eps = (math.sqrt(ra1**2 - rb1**2) + math.sqrt(ra2**2 - rb2**2) - a0 * math.sin(pa)) / (math.pi * GEAR_M * math.cos(pa))
    good = worst < 0.01 and tight > 0.01 and eps > 1.2
    ok &= good
    print(f"Редуктор {PINION_Z}:{WHEEL_GEAR_Z} (1:{GEAR_RATIO:.0f}), модуль {GEAR_M}, межосевое {GEAR_A:.2f} мм: "
          f"заклинивание {worst:.3f} мм², перекрытие ε={eps:.2f} [{'OK' if good else 'ПРОБЛЕМА'}]")

    tire = make_tire().translate([0, 0, 1.7])
    rw = union([make_rear_wheel(), tire])
    plate, pin = make_motor_plate(), make_pinion()
    for s in (-1, 1):
        n = 'Л' if s > 0 else 'П'
        wheel = place_rear_wheel(rw, s)
        pl = plate if s > 0 else mirror_y(plate)
        mot = motor_proxy(s)
        pn = place_pinion(pin, s)
        pairs = [("колесо–палуба", wheel, deck), ("колесо–плита", wheel, pl), ("колесо–мотор+гайка", wheel, mot),
                 ("мотор–палуба", mot, deck), ("плита–палуба", pl, deck), ("шестерня–палуба", pn, deck),
                 ("шестерня–плита", pn, pl), ("шестерня–колесо (зубья)", pn, wheel)]
        for name, a, b in pairs:
            v = (a ^ b).volume()
            if v > 0.5:
                print(f"  {n}: {name} пересекаются {v:.1f} мм³")
                ok = False
    other = mirror_y(plate)
    v = (plate ^ other).volume()
    ok &= v < 0.01
    print(f"Задний мост: касаний нет [{'OK' if ok else 'ЕСТЬ'}]; зазор между плитами моторов {2 * PLATE_Y0:.1f} мм")
    kv, volts = 1300, 22.2
    rpm = kv * volts / GEAR_RATIO
    v = rpm / 60 * 2 * math.pi * TIRE_R / 1000
    print(f"Скорость без нагрузки (1300KV, 6S): {rpm:.0f} об/мин колеса ≈ {v:.0f} м/с = {v * 3.6:.0f} км/ч; "
          f"20 % газа ≈ {v * 3.6 * 0.2:.0f} км/ч")
    return ok


def check():
    ok = True
    deck = make_deck()
    kn = make_knuckle()
    fw = union([make_rim(True), make_tire().translate([0, 0, 1.7])])
    front = union([kn, place_wheel(fw, FRONT_X)])
    fixed_env = union([servo_body_proxy(), stack_proxy()])
    print(f"Рулевая трапеция: рычаги до X={ARM_END[0]:.1f} Y=±{ARM_END[1]:.1f}, "
          f"вал серво X={SERVO_SHAFT_X:.1f}, поперечная тяга {BAR_L:.1f} мм между отверстиями")
    ok &= steering_check(deck, kn, fw)
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
    ok &= gear_check(deck)
    v = (servo_proxy() ^ deck).volume() + (stack_proxy() ^ deck).volume()
    print(f"Серво и стек vs палуба: {v:.2f} мм³")
    ok &= v < 0.5
    # зазор до земли
    ground = AXLE_Z - TIRE_R
    print(f"Земля на Z={ground:.1f}; низ палубы Z={-DECK_T:.1f} -> клиренс {-DECK_T - ground:.1f} мм; "
          f"нижняя плита дрона над землёй {TOWER_H - ground:.1f} мм")
    print(f"Верх колокола мотора Z={AXLE_Z + MOTOR_BELL_R:.1f}, до нижней плиты {TOWER_H - AXLE_Z - MOTOR_BELL_R:.1f} мм")
    print(f"Колея: перед {2 * (WHEEL_OUT_Y - WHEEL_W / 2):.0f} мм, зад {2 * (REAR_WHEEL_OUT_Y - WHEEL_W / 2):.0f} мм")
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
        "car_rear_wheel_gear48_x2.stl": on_bed(make_rear_wheel()),
        "car_pinion_12T_M5_x2.stl": on_bed(make_pinion()),
        # плита мотора внутренней стороной на стол: отверстия мотора без поддержек
        "car_motor_plate_L_x1.stl": on_bed(make_motor_plate().rotate([90, 0, 0])),
        "car_motor_plate_R_x1.stl": on_bed(mirror_y(make_motor_plate()).rotate([-90, 0, 0])),
        "car_tire_TPU_x4.stl": on_bed(make_tire()),
    }
    # тяга вверх ногами: плоский верх на столе, бобышки растут вверх
    parts["car_tie_bar_x1.stl"] = on_bed(make_tie_bar().rotate([180, 0, 0]))
    parts["preview_assembly.stl"] = assembly(0)
    parts["preview_assembly_left.stl"] = assembly(SERVO_TRAVEL)
    parts["preview_assembly_right.stl"] = assembly(-SERVO_TRAVEL)
    parts["preview_with_drone.stl"] = assembly(0, with_drone=True)
    # крупный план руля: перед палубы, серво, тяга, кулаки (без колёс), поворот влево
    st = steer_state(delta_for_servo(20))
    kn = make_knuckle()
    tire = make_tire().translate([0, 0, 1.7])
    parts["preview_gearbox_detail.stl"] = union(
        [make_deck() ^ box(-110, -40, -45, 45, -10, 50), make_motor_plate(), motor_proxy(1),
         place_pinion(make_pinion(), 1), place_rear_wheel(make_rear_wheel(), 1)])
    parts["preview_steering_detail.stl"] = union(
        [make_deck() ^ box(20, 80, -45, 45, -10, 50), servo_proxy(st["phi"]), place_bar(make_tie_bar(), st),
         place_knuckle_state(kn, st["dl"], 1), place_knuckle_state(kn, st["dr"], -1)])
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
