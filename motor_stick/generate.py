"""
Мотор FPV 2814 крутит палку — 5 вариантов крепления, параметрический генератор STL.

Ось мотора и палки — X. Мотор ставится основанием на стойку (плоскость X=0..5),
колокол и вал смотрят в +X. Палка (по умолчанию Ø10 мм) лежит в подшипниках 6800
(10×19×5) или в печатной втулке.

  V1  Прямой привод: муфта на вал мотора (гайка M5, как у пропеллера)
  V2  Прямой привод: стакан-хомут на колокол (если вал короткий/без резьбы)
  V3  Ремень GT2 16T → 60T, понижение 1:3.75
  V4  Шестерни 12T → 48T (модуль 1.5), понижение 1:4
  V5  Фрикционный ролик Ø16 → барабан Ø50, понижение ~1:3

Запуск:  python3 generate.py      -> stl/*.stl + preview.png
Требует: pip install manifold3d numpy trimesh matplotlib
"""
import math
import os

import manifold3d as m3d
import numpy as np
import trimesh

M = m3d.Manifold
CS = m3d.CrossSection
SEG = 72

# ---------------- ПАРАМЕТРЫ (мм) ------------------------------------------
STICK_D = 10.0        # диаметр палки
FIT = 0.3             # зазор посадки для печати
SHAFT_D = 5.0         # вал 2814
BELL_D = 35.0         # диаметр колокола 2814 (замерить штангелем!)
MOTOR_H = 28.0        # высота мотора от основания до торца колокола
SHAFT_OUT = 13.0      # вылет вала над колоколом
MOUNT_A, MOUNT_B = 16.0, 19.0   # крепёж мотора 16×19 (M3)
BRG_D, BRG_W = 19.0, 5.0        # подшипник 6800 под палку Ø10
H = 32.0              # высота оси над столом
PLATE_T = 5.0         # толщина стоек
BASE_T = 5.0          # толщина основания
M3 = 3.4              # отверстие под M3
M3_NUT_S, M3_NUT_T = 5.7, 2.6   # гайка M3 в ловушке

BELL_X = PLATE_T + MOTOR_H            # торец колокола
SHAFT_X1 = BELL_X + SHAFT_OUT         # конец вала


# ---------------- примитивы ------------------------------------------------
def box(x0, x1, y0, y1, z0, z1):
    return M.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])


def cyl_x(r, x0, x1, y=0.0, z=H, seg=SEG):
    return M.cylinder(x1 - x0, r, r, seg).rotate([0, 90, 0]).translate([x0, y, z])


def cyl_z(r, z0, z1, x=0.0, y=0.0, seg=SEG):
    return M.cylinder(z1 - z0, r, r, seg).translate([x, y, z0])


def union(parts):
    return M.batch_boolean(parts, m3d.OpType.Add)


def _uvw_to_x(m, x0):
    """(u,v,w) -> (x=w+x0, y=u, z=v): выдавленный по Z профиль кладём осью вдоль X."""
    return m.transform(np.array([[0, 0, 1, x0], [1, 0, 0, 0], [0, 1, 0, 0]], dtype=np.float32))


def prism_x(poly_yz, x0, x1):
    return _uvw_to_x(M.extrude(CS([poly_yz]), x1 - x0), x0)


def profile_x(poly_yz, x0, x1, y=0.0, z=H):
    """Профиль колеса в плоскости YZ вокруг оси (y,z), толщина по X."""
    return prism_x([(p[0] + y, p[1] + z) for p in poly_yz], x0, x1)


def gusset(x0, x1, y0, y1, h, sign=1):
    """Треугольное ребро жёсткости: от стойки (x0) по основанию."""
    pts = [(0, 0), (sign * (x1 - x0), 0), (0, h)]
    tri = M.extrude(CS([[(p[0], p[1]) for p in pts]]), y1 - y0)   # в XZ, толщина по Z
    return tri.rotate([90, 0, 0]).translate([x0, y1, BASE_T])


# ---------------- зацепления ----------------------------------------------
def gear_profile(z, m, backlash=0.15):
    """Эвольвентная шестерня (20°), список точек (y,z) против часовой."""
    r = z * m / 2
    rb = r * math.cos(math.radians(20))
    ra = r + m
    rf = r - 1.25 * m
    inv = lambda a: math.tan(a) - a
    alpha_p = math.radians(20)
    half = math.pi / (2 * z) - backlash / (2 * r)
    pts = []
    n = 8
    for i in range(z):
        c = 2 * math.pi * i / z
        flank = []
        r0 = max(rb, rf)
        for k in range(n + 1):
            rho = r0 + (ra - r0) * k / n
            a = math.acos(min(1.0, rb / rho))
            th = half + inv(alpha_p) - inv(a)
            flank.append((rho, th))
        left = [(rf, flank[0][1] + 0.0)] if rf < rb else []
        one = [(rr, c - t) for rr, t in reversed(left + flank)][::-1]
        right = [(rr, c - t) for rr, t in (left + flank)]
        lft = [(rr, c + t) for rr, t in reversed(left + flank)]
        root_a = c - math.pi / z
        tooth = [(rf, root_a)] + right + lft
        for rr, t in tooth:
            pts.append((rr * math.cos(t), rr * math.sin(t)))
    return pts


def gt2_profile(n):
    """Шкив GT2: круг с полукруглыми впадинами (печатный, проверить на ремне)."""
    pd = n * 2.0 / math.pi
    ro = pd / 2 - 0.254
    base = CS.circle(ro, 4 * n)
    grooves = [CS.circle(0.6, 24).translate([(ro + 0.15) * math.cos(a), (ro + 0.15) * math.sin(a)])
               for a in (2 * math.pi * i / n for i in range(n))]
    for g in grooves:
        base = base - g
    return base, ro


def cs_x(cs, x0, x1, y=0.0, z=H):
    return _uvw_to_x(M.extrude(cs.translate([y, z]), x1 - x0), x0)


# ---------------- узлы крепежа --------------------------------------------
def setscrew_cut(x, r_in, r_out, y=0.0, z=H):
    """Радиальный винт M3 + ловушка для гайки, вход сверху (по +Z)."""
    hole = cyl_z(M3 / 2, z, z + r_out + 1, x, y, 24)
    mid = (r_in + r_out) / 2
    nut = box(x - M3_NUT_S / 2, x + M3_NUT_S / 2, y - M3_NUT_S / 2 - 0.2, y + M3_NUT_S / 2 + 0.2,
              z + mid - M3_NUT_T / 2, z + r_out + 1)
    return hole + nut


def motor_holes(slot_y=0.0, universal=False):
    """Отверстия крепления мотора в стойке X=0..PLATE_T, центр (0,H).
    universal: 4 радиальные прорези r=7.5..12 (16×16, 16×19, 19×19).
    slot_y: удлинение по Y для натяжения ремня/прижима ролика."""
    cuts = []
    if universal:
        for a in (45, 135, 225, 315):
            ca, sa = math.cos(math.radians(a)), math.sin(math.radians(a))
            for t in np.linspace(7.5, 12.0, 10):
                cuts.append(cyl_x(M3 / 2, -1, PLATE_T + 1, t * ca, H + t * sa, 20))
        cuts.append(cyl_x(6.5, -1, PLATE_T + 1))
        return union(cuts)
    pts = [(MOUNT_A / 2, 0), (-MOUNT_A / 2, 0), (0, MOUNT_B / 2), (0, -MOUNT_B / 2)]
    for (py, pz) in pts:
        for dy in np.linspace(-slot_y, slot_y, max(2, int(slot_y * 2) + 1)):
            cuts.append(cyl_x(M3 / 2, -1, PLATE_T + 1, py + dy, H + pz, 20))
    for dy in np.linspace(-slot_y, slot_y, max(2, int(slot_y * 2) + 1)):
        cuts.append(cyl_x(6.5, -1, PLATE_T + 1, dy, H))
    return union(cuts)


def bearing_cut(x_face, y, outward=-1):
    """Гнездо подшипника 6800 + сквозное отверстие под палку."""
    thru = cyl_x(STICK_D / 2 + 1.0, x_face - 50, x_face + 50, y)
    if outward < 0:
        pocket = cyl_x(BRG_D / 2 + 0.1, x_face - 1, x_face + BRG_W, y)
    else:
        pocket = cyl_x(BRG_D / 2 + 0.1, x_face + PLATE_T - BRG_W, x_face + PLATE_T + 1, y)
    return thru + pocket


def base_holes(pts):
    return union([cyl_z(2.2, -1, BASE_T + 1, x, y, 24) for x, y in pts])


# ---------------- стойки (V1, V2) -----------------------------------------
def motor_stand():
    half = 25.0
    top = H + 24
    body = union([
        box(0, PLATE_T, -half, half, 0, top),
        box(-28, PLATE_T, -half, half, 0, BASE_T),
        gusset(0, -24, -half, -half + 5, top - 22, -1),
        gusset(0, -24, half - 5, half, top - 22, -1),
    ])
    return body - motor_holes(universal=True) - base_holes([(-18, -16), (-18, 16)])


def bearing_stand():
    half = 18.0
    top = H + 16
    body = union([
        box(0, PLATE_T, -half, half, 0, top),
        box(-20, 25, -half, half, 0, BASE_T),
        gusset(PLATE_T, 25, -half, -half + 4, top - 26, 1),
        gusset(PLATE_T, 25, half - 4, half, top - 26, 1),
    ])
    return body - bearing_cut(0, 0, outward=-1) - base_holes([(-13, 0), (18, -10), (18, 10)])


# ---------------- V1: муфта на вал ----------------------------------------
HUB_L = 8.0
SOCK_L = 25.0
CPL_R = 9.5


def coupler_shaft():
    x0 = BELL_X
    body = cyl_x(CPL_R, x0, x0 + HUB_L + SOCK_L)
    cuts = [
        cyl_x(SHAFT_D / 2 + FIT / 2, x0 - 1, x0 + HUB_L + 1),               # вал
        cyl_x(STICK_D / 2 + FIT / 2, x0 + HUB_L, x0 + HUB_L + SOCK_L + 1),  # гнездо палки
        setscrew_cut(x0 + HUB_L / 2, SHAFT_D / 2, CPL_R),                   # винт на лыску вала
        setscrew_cut(x0 + HUB_L + 7, STICK_D / 2, CPL_R),
        setscrew_cut(x0 + HUB_L + SOCK_L - 6, STICK_D / 2, CPL_R),
    ]
    return body - union(cuts)


# ---------------- V2: стакан на колокол -----------------------------------
CUP_DEPTH = 12.0


def coupler_bell():
    wall = 2.6
    r_in = BELL_D / 2 + FIT
    r_out = r_in + wall
    x_cup0 = BELL_X - CUP_DEPTH
    x_bot1 = BELL_X + 4.0
    body = union([
        cyl_x(r_out, x_cup0, x_bot1),
        cyl_x(CPL_R, x_bot1 - 1, SHAFT_X1 + 1 + SOCK_L),
        box(x_cup0, x_cup0 + 10, -5, 5, H + r_in, H + r_out + 7),           # ухо хомута
    ])
    cuts = [
        cyl_x(r_in, x_cup0 - 1, BELL_X),                                    # колокол
        cyl_x(4.0, BELL_X - 0.5, SHAFT_X1 + 1),                             # вал проходит насквозь
        cyl_x(STICK_D / 2 + FIT / 2, SHAFT_X1 + 1, SHAFT_X1 + 2 + SOCK_L),
        box(x_cup0 - 1, x_cup0 + CUP_DEPTH - 3, -0.8, 0.8, H, H + r_out + 8),  # прорезь хомута
        setscrew_cut(SHAFT_X1 + 8, STICK_D / 2, CPL_R),
        setscrew_cut(SHAFT_X1 + SOCK_L - 5, STICK_D / 2, CPL_R),
    ]
    # винт хомута — по Y через ухо
    screw = (M.cylinder(24, M3 / 2, M3 / 2, 20).rotate([90, 0, 0])
             .translate([x_cup0 + 5, 12, H + r_out + 3]))
    return body - union(cuts) - screw


# ---------------- рама с передачей (V3, V4, V5) ---------------------------
WHEEL_X0 = BELL_X               # колесо мотора лежит на торце колокола
WHEEL_W = 7.0
FRONT_X = WHEEL_X0 + WHEEL_W + 10.0


def drive_frame(C, slot):
    """Основание + задняя стойка (мотор + подшипник палки) + передняя стойка.
    Мотор на Y=0, палка на Y=C."""
    y0, y1 = -26.0, C + 26.0
    top_m = H + 24
    body = union([
        box(0, PLATE_T, y0, y1, 0, top_m),                                  # задняя стойка
        box(-22, FRONT_X + PLATE_T + 18, y0, y1, 0, BASE_T),                # основание
        box(FRONT_X, FRONT_X + PLATE_T, C - 18, C + 18, 0, H + 16),         # передняя стойка
        gusset(0, -20, y0, y0 + 5, top_m - 22, -1),
        gusset(0, -20, y1 - 5, y1, top_m - 22, -1),
        gusset(FRONT_X + PLATE_T, FRONT_X + PLATE_T + 16, C - 18, C - 14, H - 10, 1),
        gusset(FRONT_X + PLATE_T, FRONT_X + PLATE_T + 16, C + 14, C + 18, H - 10, 1),
    ])
    cuts = [
        motor_holes(slot_y=slot),
        bearing_cut(0, C, outward=-1),
        bearing_cut(FRONT_X, C, outward=+1),
        base_holes([(-15, y0 + 8), (-15, y1 - 8), (FRONT_X + 16, C - 10),
                    (FRONT_X + 16, C + 10), (FRONT_X - 6, y0 + 8)]),
    ]
    return body - union(cuts)


def hub_on_motor(wheel, x0, x1):
    """Колесо на вал мотора: садится на вал и прижимается к колоколу гайкой M5,
    как пропеллер (вал 2814 выступает на ~13 мм, колесо 7–8 мм + гайка)."""
    return wheel - cyl_x(SHAFT_D / 2 + FIT / 2, x0 - 1, x1 + 1, 0)


def hub_on_stick(wheel, x0, x1, C, hub_r=10.0):
    hub_x1 = x1 + 8
    body = wheel + cyl_x(hub_r, x1 - 0.5, hub_x1, C)
    cuts = [cyl_x(STICK_D / 2 + FIT / 2, x0 - 1, hub_x1 + 1, C),
            setscrew_cut((x1 + hub_x1) / 2, STICK_D / 2, hub_r, C)]
    return body - union(cuts)


def flanged(cs_wheel, r_out, x0, x1, y):
    w = cs_x(cs_wheel, x0, x1, y)
    return w + cyl_x(r_out + 1.8, x0 - 1.2, x0, y) + cyl_x(r_out + 1.8, x1, x1 + 1.2, y)


# V3 GT2
GT2_S, GT2_L = 16, 60
GT2_BELT = 200.0


def gt2_center():
    d = GT2_S * 2 / math.pi
    D = GT2_L * 2 / math.pi
    a = 2.0
    b = math.pi * (D + d) / 2 - GT2_BELT
    c = (D - d) ** 2 / 4
    return (-b + math.sqrt(b * b - 4 * a * c)) / (2 * a)


def v3_parts():
    C = gt2_center()
    x0, x1 = WHEEL_X0 + 1.2, WHEEL_X0 + 1.2 + 6.0
    s_cs, s_r = gt2_profile(GT2_S)
    l_cs, l_r = gt2_profile(GT2_L)
    small = hub_on_motor(flanged(s_cs, s_r, x0, x1, 0), x0 - 1.2, x1 + 1.2)
    large = hub_on_stick(flanged(l_cs, l_r, x0, x1, C), x0 - 1.2, x1 + 1.2, C)
    return C, drive_frame(C, 3.0), small, large


# V4 шестерни
G_M, G_Z1, G_Z2 = 1.5, 12, 48


def v4_parts():
    C = G_M * (G_Z1 + G_Z2) / 2
    x0, x1 = WHEEL_X0, WHEEL_X0 + WHEEL_W
    pin = profile_x(gear_profile(G_Z1, G_M), x0, x1, 0)
    gear = profile_x(gear_profile(G_Z2, G_M), x0, x1, C)
    # облегчение большой шестерни
    holes = union([cyl_x(6.5, x0 - 1, x1 + 1, C + 22 * math.cos(a), H + 22 * math.sin(a))
                   for a in np.linspace(0, 2 * math.pi, 6, endpoint=False)])
    pin = hub_on_motor(pin, x0, x1)
    gear = hub_on_stick(gear - holes, x0, x1, C)
    return C, drive_frame(C, 1.0), pin, gear


# V5 фрикцион
ROLL_D, DRUM_D, ORING = 16.0, 50.0, 2.0


def v5_parts():
    C = (ROLL_D + DRUM_D) / 2 - 0.5          # натяг 0.5 мм, дальше — прорезями
    x0, x1 = WHEEL_X0, WHEEL_X0 + WHEEL_W
    roll = cyl_x(ROLL_D / 2, x0, x1, 0)
    groove = (M.cylinder(ORING, ROLL_D / 2 + 1, ROLL_D / 2 + 1, SEG) -
              M.cylinder(ORING, ROLL_D / 2 - 1.0, ROLL_D / 2 - 1.0, SEG)) \
        .rotate([0, 90, 0]).translate([(x0 + x1) / 2 - ORING / 2, 0, H])
    roll = hub_on_motor(roll - groove, x0, x1)
    drum = cyl_x(DRUM_D / 2, x0, x1, C)
    rim = (M.cylinder(x1 - x0 - 3, DRUM_D / 2 + 1, DRUM_D / 2 + 1, SEG) -
           M.cylinder(x1 - x0 - 3, DRUM_D / 2 - 0.6, DRUM_D / 2 - 0.6, SEG)) \
        .rotate([0, 90, 0]).translate([x0 + 1.5, C, H])               # канавка под резинку
    holes = union([cyl_x(5.5, x0 - 1, x1 + 1, C + 15 * math.cos(a), H + 15 * math.sin(a))
                   for a in np.linspace(0, 2 * math.pi, 6, endpoint=False)])
    drum = hub_on_stick(drum - rim - holes, x0, x1, C)
    return C, drive_frame(C, 3.0), roll, drum


# ---------------- макеты для картинки -------------------------------------
def motor_dummy():
    return union([cyl_x(14.0, PLATE_T, PLATE_T + 5), cyl_x(BELL_D / 2, PLATE_T + 5, BELL_X),
                  cyl_x(SHAFT_D / 2, BELL_X, SHAFT_X1, seg=24)])


def stick_dummy(x0, x1, y=0.0):
    return cyl_x(STICK_D / 2, x0, x1, y, seg=32)


# ---------------- экспорт -------------------------------------------------
def to_trimesh(man):
    mesh = man.to_mesh()
    return trimesh.Trimesh(vertices=np.asarray(mesh.vert_properties)[:, :3],
                           faces=np.asarray(mesh.tri_verts), process=False)


def on_bed(man, flip_to_x=False):
    if flip_to_x:   # колёса/муфты печатать торцом на столе
        man = man.rotate([0, -90, 0])
    (x0, y0, z0), (x1, y1, z1) = man.bounding_box()[:3], man.bounding_box()[3:]
    return man.translate([-(x0 + x1) / 2, -(y0 + y1) / 2, -z0])


def build():
    ms, bs = motor_stand(), bearing_stand()
    c1, c2 = coupler_shaft(), coupler_bell()
    BS_X = 150.0
    C3, f3, s3, l3 = v3_parts()
    C4, f4, s4, l4 = v4_parts()
    C5, f5, s5, l5 = v5_parts()

    stl = {
        "V1_V2_motor_stand": (ms, False),
        "V1_V2_bearing_stand_x2": (bs, False),
        "V1_coupler_shaft": (c1, True),
        "V2_coupler_bell": (c2, True),
        "V3_frame_gt2": (f3, False),
        "V3_pulley_16T_motor": (s3, True),
        "V3_pulley_60T_stick": (l3, True),
        "V4_frame_gears": (f4, False),
        "V4_gear_12T_motor": (s4, True),
        "V4_gear_48T_stick": (l4, True),
        "V5_frame_friction": (f5, False),
        "V5_roller_motor": (s5, True),
        "V5_drum_stick": (l5, True),
    }
    mot = motor_dummy()
    asm = {
        "V1": [(ms, "p"), (c1, "a"), (bs.translate([BS_X, 0, 0]), "p"), (mot, "m"),
               (stick_dummy(BELL_X + HUB_L + 5, BS_X + 40), "s")],
        "V2": [(ms, "p"), (c2, "a"), (bs.translate([BS_X, 0, 0]), "p"), (mot, "m"),
               (stick_dummy(SHAFT_X1 + 2, BS_X + 40), "s")],
        "V3": [(f3, "p"), (s3, "a"), (l3, "a"), (mot, "m"), (stick_dummy(-15, FRONT_X + 40, C3), "s"),
               (belt_dummy(C3), "b")],
        "V4": [(f4, "p"), (s4, "a"), (l4, "a"), (mot, "m"), (stick_dummy(-15, FRONT_X + 40, C4), "s")],
        "V5": [(f5, "p"), (s5, "a"), (l5, "a"), (mot, "m"), (stick_dummy(-15, FRONT_X + 40, C5), "s")],
    }
    return stl, asm


def belt_dummy(C):
    d = GT2_S * 2 / math.pi / 2 + 0.6
    D = GT2_L * 2 / math.pi / 2 + 0.6
    x0, x1 = WHEEL_X0 + 1.4, WHEEL_X0 + 7.0
    outer = M.hull(cyl_x(D + 0.8, x0, x1, C) + cyl_x(d + 0.8, x0, x1, 0))
    inner = M.hull(cyl_x(D - 0.2, x0 - 1, x1 + 1, C) + cyl_x(d - 0.2, x0 - 1, x1 + 1, 0))
    return outer - inner


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    out = os.path.join(here, "stl")
    os.makedirs(out, exist_ok=True)
    stl, asm = build()
    for name, (man, flip) in stl.items():
        tm = to_trimesh(on_bed(man, flip))
        assert tm.is_watertight, name
        tm.export(os.path.join(out, name + ".stl"))
        e = tm.extents
        print(f"{name:28s} {e[0]:6.1f} x {e[1]:6.1f} x {e[2]:6.1f} мм")
    render(asm, os.path.join(here, "preview.png"))


TITLES = {
    "V1": "V1  Прямой привод: муфта на вал (гайка M5)",
    "V2": "V2  Прямой привод: стакан-хомут на колокол",
    "V3": "V3  Ремень GT2 16T→60T  (1:3.75)",
    "V4": "V4  Шестерни 12T→48T  (1:4)",
    "V5": "V5  Фрикцион: ролик Ø16 → барабан Ø50 (1:3)",
}
COLORS = {"p": (0.25, 0.50, 0.62), "a": (0.93, 0.55, 0.15), "m": (0.30, 0.30, 0.33),
          "s": (0.72, 0.55, 0.35), "b": (0.12, 0.12, 0.12)}


def render(asm, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    fig = plt.figure(figsize=(18, 11))
    light = np.array([0.5, -0.6, 0.7]); light /= np.linalg.norm(light)
    for i, (key, items) in enumerate(asm.items()):
        ax = fig.add_subplot(2, 3, i + 1, projection="3d")
        tris, cols = [], []
        for man, kind in items:
            tm = to_trimesh(man)
            shade = 0.35 + 0.65 * np.clip(tm.face_normals @ light, 0, 1)
            tris.append(tm.vertices[tm.faces])
            cols.append(np.c_[np.outer(shade, COLORS[kind]) + 0.06, np.ones_like(shade)].clip(0, 1))
        # одна коллекция на сцену — иначе matplotlib сортирует детали целиком
        ax.add_collection3d(Poly3DCollection(np.vstack(tris), facecolors=np.vstack(cols),
                                             edgecolor="none"))
        v = np.vstack(tris).reshape(-1, 3)
        lo, hi = v.min(0), v.max(0); c = (lo + hi) / 2; r = (hi - lo).max() / 2 * 0.72
        ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(c[2] - r, c[2] + r)
        ax.set_box_aspect((1, 1, 1)); ax.view_init(22, -40); ax.set_axis_off()
        ax.set_title(TITLES[key], fontsize=13)
    ax = fig.add_subplot(2, 3, 6); ax.set_axis_off()
    leg = [("синий", "печатные стойки / рама"), ("оранжевый", "печатные муфты, шкивы, шестерни"),
           ("тёмный", "мотор 2814 (макет)"), ("коричневый", "палка Ø10 (макет)"),
           ("чёрный", "ремень GT2 200 мм")]
    for j, ((cn, txt), k) in enumerate(zip(leg, "pamsb")):
        ax.add_patch(plt.Rectangle((0.05, 0.82 - j * 0.13), 0.08, 0.08, color=COLORS[k]))
        ax.text(0.17, 0.86 - j * 0.13, txt, fontsize=13, va="center")
    ax.set_xlim(0, 1); ax.set_ylim(0, 1)
    plt.tight_layout()
    plt.savefig(path, dpi=80)


if __name__ == "__main__":
    main()
