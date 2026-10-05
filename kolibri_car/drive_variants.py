"""
10 вариантов приводного модуля: мотор 2814 рядом с палкой Ø10 (ось мотора параллельна
палке, на одной высоте), коробчатая рама: левая стенка держит мотор и подшипник палки,
правая — второй подшипник, передача между колоколом мотора и правой стенкой.

Система координат: Y — вдоль палки, X — поперёк (мотор в -X от палки), Z вверх,
Z=0 — верх основания. Палка на Z = ROD_Z (как ось машинки: под нижней плитой дрона
остаётся место). Палка Ø10 в подшипниках 6800ZZ (10×19×5).

Запуск:  python3 drive_variants.py          -> stl/drive/V*/ + drive_variants.png
         python3 drive_variants.py --check  -> зацепления и касания
"""
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np

from generate_car import M, m3d, box, cyl_y, cyl_z, union, hex_prism, to_trimesh, on_bed

HERE = os.path.dirname(os.path.abspath(__file__))

# ---------------- ПАРАМЕТРЫ (мм) ----------------
ROD_Z = 20.0            # высота палки над основанием
ROD_R = 5.0             # палка Ø10
BRG_D, BRG_W = 19.1, 5.0   # 6800ZZ
WALL_T = 6.0
WALL_TOP = 40.0         # под нижнюю плиту дрона (42 в машинке)
BASE_T = 4.0
# мотор 2814: колокол Ø35, лапа→верх колокола 32, вал M5 выступает 14
MOTOR_R, MOTOR_L, SHAFT_L = 17.5, 32.0, 14.0
MOTOR_HOLES = [(8, 0), (-8, 0), (0, 9.5), (0, -9.5)]   # 16 × 19, M3
P0 = MOTOR_L + 1.0      # начало плоскости передачи (за верхом колокола)
KV, VOLT, WHEEL_D = 900, 22.2, 70.0   # для оценки скорости: 2814 900KV, 6S, колесо Ø70
GM = 1.0                # модуль шестерён
BL = 0.15               # боковой зазор (утонение зуба)
# ------------------------------------------------

COL = {"frame": (0.25, 0.50, 0.62), "print": (0.93, 0.55, 0.18), "motor": (0.30, 0.31, 0.34),
       "rod": (0.72, 0.56, 0.36), "belt": (0.12, 0.12, 0.12), "tpu": (0.85, 0.35, 0.12)}


# ---------------------------------------------------------------- 2D профили
def involute(z, m=GM, shift=0.0, backlash=BL, ha=1.0, hf=1.25, pts=10):
    """Внешнее эвольвентное колесо, зуб №0 по +X."""
    pa = math.radians(20)
    rp = m * z / 2
    rb = rp * math.cos(pa)
    ra = rp + m * (ha + shift)
    rf = rp - m * (hf - shift)
    s = m * (math.pi / 2 + 2 * shift * math.tan(pa)) - backlash
    inv = lambda a: math.tan(a) - a
    half = s / (2 * rp) + inv(pa)
    psi = lambda r: half if r <= rb else half - inv(math.acos(rb / r))
    r0 = max(rb, rf)
    rs = [r0 + (ra - r0) * i / pts for i in range(pts + 1)]
    poly = []
    for i in range(z):
        c = 2 * math.pi * i / z
        prof = [(rf, c - psi(r0))] + [(r, c - psi(r)) for r in rs] + \
               [(r, c + psi(r)) for r in reversed(rs)] + [(rf, c + psi(r0))]
        poly += [(r * math.cos(a), r * math.sin(a)) for r, a in prof]
    return m3d.CrossSection([poly])


def ring_gear(z, outer_r, m=GM, shift=0.0, tip_r=None):
    """Венец с внутренними зубьями: впадины — это профиль внешнего колеса-«долбяка»."""
    rp = m * z / 2
    tip_r = tip_r or rp - m * (1 - shift) + 0.4
    tool = involute(z, m, shift, backlash=-BL, ha=1.25)
    return circle(outer_r) - (tool + circle(tip_r))


def circle(r, n=96):
    return m3d.CrossSection.circle(r, n)


def gt2_pulley(teeth):
    pd = teeth * 2 / math.pi
    od = pd - 0.508
    cs = circle(od / 2, 128)
    grooves = [circle(0.6, 16).translate([od / 2 * math.cos(2 * math.pi * i / teeth),
                                          od / 2 * math.sin(2 * math.pi * i / teeth)]) for i in range(teeth)]
    return cs - m3d.CrossSection.batch_boolean(grooves, m3d.OpType.Add), pd / 2


def band(c1, r1, c2, r2, t):
    """Открытый ремень вокруг двух шкивов (2D кольцо толщиной t по делительным радиусам)."""
    outer = m3d.CrossSection.batch_hull([circle(r1 + t / 2).translate(c1), circle(r2 + t / 2).translate(c2)])
    inner = m3d.CrossSection.batch_hull([circle(r1 - t / 2).translate(c1), circle(r2 - t / 2).translate(c2)])
    return outer - inner


def belt_length(a, r1, r2):
    return 2 * a + math.pi * (r1 + r2) + (r2 - r1) ** 2 / a


def solve_a(L, r1, r2):
    lo, hi = abs(r2 - r1) + 1, 500
    for _ in range(80):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if belt_length(mid, r1, r2) < L else (lo, mid)
    return (lo + hi) / 2


# ---------------------------------------------------------------- 3D размещение
def ext(cs, y0, y1, x=0.0, z=ROD_Z, twist=0.0, spin=0.0):
    """2D профиль (ось вращения = локальная Z) -> тело вдоль Y от y0 до y1 с центром (x, z)."""
    m = M.extrude(cs.rotate(spin), y1 - y0, n_divisions=12 if twist else 0, twist_degrees=twist)
    return m.rotate([-90, 0, 0]).translate([x, y0, z])


def motor(x):
    return union([cyl_y(15.0, 0, 4, x, ROD_Z), cyl_y(MOTOR_R, 4, MOTOR_L, x, ROD_Z),
                  cyl_y(2.5, MOTOR_L, MOTOR_L + SHAFT_L, x, ROD_Z, 24)])


def rod(y0, y1, x=0.0):
    return cyl_y(ROD_R, y0, y1, x, ROD_Z, 48)


def frame(motor_x, right_y0, rod_through_left=True, slots=0.0, extra_left=None, right_t=WALL_T,
          right_xspan=None):
    """Коробчатая рама. Левая стенка y∈[-WALL_T,0], правая y∈[right_y0, right_y0+right_t]."""
    xl0 = min(motor_x - 22, -14)
    xl1 = max(motor_x + 22, 14) if not rod_through_left else max(14, motor_x + 22)
    if extra_left:
        xl0 = min(xl0, extra_left[0]); xl1 = max(xl1, extra_left[1])
    y_end = right_y0 + right_t
    xr0, xr1 = xl0, max(xl1, 15)
    xb0, xb1 = min(xl0, xr0) - 2, max(xl1, xr1) + 2
    parts = [box(xb0, xb1, -WALL_T - 10, y_end + 10, -BASE_T, 0),
             box(xl0, xl1, -WALL_T, 0, 0, WALL_TOP),
             box(xr0, xr1, right_y0, y_end, 0, ROD_Z + 15)]
    parts.append(cyl_y(13, right_y0, y_end, 0, ROD_Z))
    # косынки снаружи стенок
    for gx in (xl0, xl1 - 4):
        parts.append(M.hull_points([[gx, -WALL_T, 0], [gx + 4, -WALL_T, 0], [gx, -WALL_T - 10, 0],
                                    [gx + 4, -WALL_T - 10, 0], [gx, -WALL_T, 32], [gx + 4, -WALL_T, 32]]))
    for gx in (xr0, xr1 - 4):
        parts.append(M.hull_points([[gx, y_end, 0], [gx + 4, y_end, 0], [gx, y_end + 10, 0],
                                    [gx + 4, y_end + 10, 0], [gx, y_end, 26], [gx + 4, y_end, 26]]))
    # задняя стенка-связь (за мотором) с окном: коробка не складывается при ударе
    back = box(xl0, xl0 + 4, -WALL_T, y_end, 0, WALL_TOP)
    back -= box(xl0 - 1, xl0 + 5, 3, y_end - 4, 7, WALL_TOP - 6)
    parts.append(back)
    parts.append(box(xl0, xl1, -WALL_T, -WALL_T + 0.01, 0, 0.01))
    f = union(parts)
    cuts = [cyl_y(6.0, -WALL_T - 1, 1, motor_x, ROD_Z)]
    for dx, dz in MOTOR_HOLES:
        if slots:
            cuts.append(M.batch_hull([cyl_y(1.65, -WALL_T - 1, 1, motor_x + dx - slots, ROD_Z + dz, 20),
                                      cyl_y(1.65, -WALL_T - 1, 1, motor_x + dx + slots, ROD_Z + dz, 20)]))
        else:
            cuts.append(cyl_y(1.65, -WALL_T - 1, 1, motor_x + dx, ROD_Z + dz, 20))
    if slots:
        cuts.append(M.batch_hull([cyl_y(6.0, -WALL_T - 1, 1, motor_x - slots, ROD_Z),
                                  cyl_y(6.0, -WALL_T - 1, 1, motor_x + slots, ROD_Z)]))
    if rod_through_left:
        cuts += [cyl_y(BRG_D / 2, -WALL_T - 1, -WALL_T + BRG_W, 0, ROD_Z), cyl_y(6.5, -WALL_T - 1, 1, 0, ROD_Z)]
    cuts += [cyl_y(BRG_D / 2, y_end - BRG_W, y_end + 1, 0, ROD_Z), cyl_y(6.5, right_y0 - 1, y_end + 1, 0, ROD_Z)]
    if right_t >= 2 * BRG_W + 1:
        cuts.append(cyl_y(BRG_D / 2, right_y0 - 1, right_y0 + BRG_W, 0, ROD_Z))
    return f - union(cuts)


def hub(y0, y1, r=8.0):
    """Ступица на палку Ø10 с поперечным винтом M3 (стопор)."""
    h = cyl_y(r, y0, y1, 0, ROD_Z) - cyl_y(ROD_R + 0.15, y0 - 1, y1 + 1, 0, ROD_Z)
    ym = (y0 + y1) / 2
    return h - M.cylinder(2 * r + 2, 1.4, 1.4, 16).translate([0, 0, -r - 1]).translate([0, ym, ROD_Z])


def on_shaft(cs, y0, y1, x, spin=0.0, twist=0.0, nut=True):
    """Деталь на вал M5 мотора, зажата гайкой пропеллера."""
    return ext(cs, y0, y1, x, twist=twist, spin=spin) - cyl_y(2.6, y0 - 1, y1 + 1, x, ROD_Z, 24)


def nut_proxy(x, y0):
    return hex_prism(8.0, 0, 4.5).rotate([-90, 0, 0]).translate([x, y0, ROD_Z])


# ---------------------------------------------------------------- зацепления
def phase_search(drv_cs, drv_pos, drn_cs, drn_pos, period):
    best, bp = 1e9, 0.0
    d = drv_cs.translate(drv_pos)
    for i in range(int(period / 0.25) + 1):
        a = i * 0.25
        v = (d ^ drn_cs.rotate(a).translate(drn_pos)).area()
        if v < best:
            best, bp = v, a
    return bp, best


def mesh_sweep(drv_cs, drv_pos, drv_ph, drn_cs, drn_pos, drn_ph, ratio, steps=40, internal=False):
    """Крутим ведущее на шаг зуба, ведомое — с передаточным; ищем максимум пересечения."""
    worst, eng = 0.0, 1e9
    z_period = 30.0
    for i in range(steps + 1):
        a = z_period * i / steps
        d = drv_cs.rotate(drv_ph + a).translate(drv_pos)
        n = drn_cs.rotate(drn_ph + (a / ratio if internal else -a / ratio)).translate(drn_pos)
        worst = max(worst, (d ^ n).area())
        # зуб в зацеплении: доворот ведомого на 1/4 шага обязан дать пересечение
        kick = 90.0 / (ratio * 12)
        n2 = drn_cs.rotate(drn_ph + (a / ratio if internal else -a / ratio) + kick).translate(drn_pos)
        n3 = drn_cs.rotate(drn_ph + (a / ratio if internal else -a / ratio) - kick).translate(drn_pos)
        eng = min(eng, max((d ^ n2).area(), (d ^ n3).area()))
    return worst, eng


# ---------------------------------------------------------------- варианты
def v_coupler():
    """Прямой привод: муфта на вал мотора, палка соосно."""
    mx = 0.0
    y1 = MOTOR_L + 22
    c = cyl_y(9, MOTOR_L + 0.5, y1, 0, ROD_Z)
    c -= union([cyl_y(2.6, MOTOR_L - 1, MOTOR_L + 9, 0, ROD_Z, 24),
                hex_prism(8.3, 0, 7).rotate([-90, 0, 0]).translate([0, MOTOR_L + 6, ROD_Z]),   # гайка вала внутри
                cyl_y(ROD_R + 0.15, MOTOR_L + 9, y1 + 1, 0, ROD_Z)])
    c -= M.cylinder(20, 1.4, 1.4, 16).translate([0, 0, -10]).translate([0, y1 - 6, ROD_Z])
    ry = y1 + 6
    fr = frame(mx, ry, rod_through_left=False, right_t=2 * BRG_W + 2)
    return dict(name="Прямой: муфта на вал", ratio=1.0, frame=fr,
                parts=[("coupler", c, "print", 1)], motor=motor(mx),
                extra=[("rod", rod(MOTOR_L + 10, ry + 2 * BRG_W + 2 + 25), "rod")],
                note="самый простой; очень быстрый, рывок при старте")


def v_bell_clamp():
    """Прямой привод: стакан-хомут на колокол."""
    mx = 0.0
    y0, y1 = 12.0, MOTOR_L + 20
    c = cyl_y(MOTOR_R + 3, y0, y1, 0, ROD_Z)
    ear = box(-4, 4, y0, y0 + 10, ROD_Z + MOTOR_R, ROD_Z + MOTOR_R + 8)
    c = union([c, ear])
    c -= union([cyl_y(MOTOR_R + 0.2, y0 - 1, MOTOR_L + 0.01, 0, ROD_Z),
                cyl_y(4.5, MOTOR_L - 1, MOTOR_L + 5, 0, ROD_Z),
                cyl_y(ROD_R + 0.15, MOTOR_L + 5, y1 + 1, 0, ROD_Z),
                box(-0.8, 0.8, y0 - 1, y0 + 11, ROD_Z, ROD_Z + MOTOR_R + 9),              # прорезь хомута
                M.cylinder(12, 1.65, 1.65, 16).rotate([0, 90, 0]).translate([-6, y0 + 5, ROD_Z + MOTOR_R + 4])])
    ry = y1 + 6
    fr = frame(mx, ry, rod_through_left=False, right_t=2 * BRG_W + 2)
    return dict(name="Прямой: стакан-хомут на колокол", ratio=1.0, frame=fr,
                parts=[("bell_clamp", c, "print", 1)], motor=motor(mx),
                extra=[("rod", rod(MOTOR_L + 5, ry + 2 * BRG_W + 2 + 25), "rod")],
                note="момент берётся с колокола, вал не нагружен; очень быстрый")


def v_spur(z1=12, z2=48, helical=False):
    a = GM * (z1 + z2) / 2 + 0.1
    mx = -a
    w = 7.0
    p = involute(z1, shift=0.3)
    g = involute(z2, shift=-0.3)
    ph, _ = phase_search(p, (-a, 0), g, (0, 0), 360 / z2)
    beta = math.radians(20)
    tw1 = math.degrees(w * math.tan(beta) / (GM * z1 / 2)) if helical else 0
    tw2 = -math.degrees(w * math.tan(beta) / (GM * z2 / 2)) if helical else 0
    pin = on_shaft(p, P0, P0 + w, mx, twist=tw1)
    gear = ext(g, P0, P0 + w, 0, spin=ph, twist=tw2)
    gear = union([gear, hub(P0 + w - 0.01, P0 + w + 8)]) - cyl_y(ROD_R + 0.15, P0 - 1, P0 + w + 9, 0, ROD_Z)
    for i in range(4):
        an = math.radians(45 + 90 * i)
        gear -= cyl_y(5.5, P0 - 1, P0 + w + 1, 14.5 * math.cos(an), ROD_Z + 14.5 * math.sin(an), 32)
    ry = P0 + w + 10
    fr = frame(mx, ry)
    name = f"Шестерни {'косозубые' if helical else ''} {z1}T→{z2}T".replace("  ", " ")
    return dict(name=name, ratio=z2 / z1, frame=fr,
                parts=[(f"pinion_{z1}T", pin, "print", 1), (f"gear_{z2}T", gear, "print", 1)],
                motor=union([motor(mx), nut_proxy(mx, P0 + w)]),
                extra=[("rod", rod(-WALL_T - 12, ry + WALL_T + 25), "rod")],
                mesh=[(p, (-a, 0), 0, g, (0, 0), ph, z2 / z1, False)],
                note="тише и плавнее, осевая сила на подшипник" if helical else "надёжно, без натяжения, слышно")


def v_two_stage():
    z1, z2 = 12, 36
    a = GM * (z1 + z2) / 2 + 0.1
    mx, ix = -2 * a, -a
    w = 6.0
    p, g = involute(z1, shift=0.3), involute(z2, shift=-0.3)
    ph1, _ = phase_search(p, (mx, 0), g, (ix, 0), 360 / z2)
    # промежуточный блок: 36T (ступень 1) + 12T (ступень 2) на оси M5
    blk = union([ext(g, P0, P0 + w, ix, spin=ph1), ext(p, P0 + w, P0 + 2 * w + 0.5, ix, spin=ph1)])
    # 2 × 625ZZ по торцам блока, ось — болт M5 из правой стенки
    blk -= union([cyl_y(8.05, P0 - 1, P0 + 5, ix, ROD_Z), cyl_y(8.05, P0 + 2 * w - 4.5, P0 + 2 * w + 2, ix, ROD_Z),
                  cyl_y(3.0, P0 - 1, P0 + 2 * w + 2, ix, ROD_Z)])
    small_rel = p.rotate(ph1).translate([ix, 0])
    ph2, _ = phase_search(small_rel, (0, 0), g, (0, 0), 360 / z2)
    gear = union([ext(g, P0 + w + 0.5, P0 + 2 * w + 0.5, 0, spin=ph2), hub(P0 + 2 * w + 0.49, P0 + 2 * w + 8)])
    gear -= cyl_y(ROD_R + 0.15, P0, P0 + 2 * w + 10, 0, ROD_Z)
    pin = on_shaft(p, P0, P0 + w, mx)
    ry = P0 + 2 * w + 10
    fr = frame(mx, ry)
    # ось промежуточного блока: M5 от правой стенки
    fr = union([fr, cyl_y(6, P0 + 2 * w + 0.8, ry + 0.01, ix, ROD_Z)]) - cyl_y(2.6, P0, ry + WALL_T + 1, ix, ROD_Z, 24)
    return dict(name="Два каскада 12→36→12→36", ratio=9.0, frame=fr,
                parts=[("pinion_12T", pin, "print", 1), ("idler_36T_12T", blk, "print", 1),
                       ("gear_36T", gear, "print", 1)],
                motor=union([motor(mx), nut_proxy(mx, P0 + w)]),
                extra=[("rod", rod(-WALL_T - 12, ry + WALL_T + 25), "rod")],
                mesh=[(p, (mx, 0), 0, g, (ix, 0), ph1, 3.0, False),
                      (p, (ix, 0), ph1, g, (0, 0), ph2, 3.0, False)],
                note="много момента, медленно; больше деталей")


def v_internal():
    z1, z2, m = 12, 50, 0.8
    a = m * (z2 - z1) / 2
    mx = -a
    w = 7.0
    p = involute(z1, m, shift=0.3, backlash=0.12)
    ring_cs = ring_gear(z2, 23.0, m, shift=0.3, tip_r=m * z2 / 2 - 0.4)
    ph, _ = phase_search(p, (-a, 0), ring_cs, (0, 0), 360 / z2)
    ring = ext(ring_cs, P0, P0 + w, 0, spin=ph)
    # за венцом — проставка-кольцо (внутри крутится гайка вала мотора), потом дно и ступица
    spacer = cyl_y(23, P0 + w - 0.01, P0 + w + 6.5, 0, ROD_Z) - cyl_y(20.5, P0 + w - 1, P0 + w + 7, 0, ROD_Z)
    cup = union([ring, spacer, cyl_y(23, P0 + w + 6.49, P0 + w + 9.5, 0, ROD_Z), hub(P0 + w + 9.49, P0 + w + 17)])
    cup -= cyl_y(ROD_R + 0.15, P0 + w + 6, P0 + w + 18, 0, ROD_Z)
    pin = on_shaft(p, P0, P0 + w, mx)
    ry = P0 + w + 19
    fr = frame(mx, ry, rod_through_left=False, right_t=2 * BRG_W + 2)
    return dict(name="Внутреннее зацепление 12T→50T", ratio=z2 / z1, frame=fr,
                parts=[("pinion_12T_m0.8", pin, "print", 1), ("ring_50T_cup", cup, "print", 1)],
                motor=union([motor(mx), nut_proxy(mx, P0 + w)]),
                extra=[("rod", rod(P0 + w + 9.5, ry + 2 * BRG_W + 2 + 25), "rod")],
                mesh=[(p, (-a, 0), 0, ring_cs, (0, 0), ph, z2 / z1, True)],
                note="зубья закрыты венцом, компактно по ширине; палка только с одной стороны")


def v_planetary():
    zs, zp, zr, m = 12, 18, 48, 0.8
    a = m * (zs + zp) / 2
    w = 7.0
    sun = involute(zs, m, shift=0.3, backlash=0.12)
    pl = involute(zp, m, shift=-0.3, backlash=0.12)
    ring_cs = ring_gear(zr, 23.0, m, shift=-0.3, tip_r=m * zr / 2 - 0.75)
    planets, phs = [], []
    for k in range(3):
        ang = 120 * k
        c = (a * math.cos(math.radians(ang)), a * math.sin(math.radians(ang)))
        # фаза сателлита против солнца
        best, bp = 1e9, 0
        for i in range(int(20 / 0.25) + 1):
            ph = i * 0.25
            v = (sun ^ pl.rotate(ph).translate(c)).area()
            if v < best:
                best, bp = v, ph
        phs.append((c, bp))
    # фаза венца против первого сателлита
    rph, _ = phase_search(pl.rotate(phs[0][1]), phs[0][0], ring_cs, (0, 0), 360 / zr)
    sun_m = on_shaft(sun, P0, P0 + w, 0)
    pl_parts = []
    for c, ph in phs:
        pm = ext(pl, P0, P0 + w, c[0], z=ROD_Z - c[1], spin=ph) - cyl_y(1.7, P0 - 1, P0 + w + 1, c[0], ROD_Z - c[1], 20)
        pl_parts.append(pm)
    ring = ext(ring_cs, P0 - 1, P0 + w + 0.5, 0, spin=rph)
    housing = union([box(-23, 23, P0 - 1, P0 + w + 0.5, 0, ROD_Z) - cyl_y(22.9, P0 - 2, P0 + w + 2, 0, ROD_Z), ring])
    carrier = cyl_y(16, P0 + w + 0.8, P0 + w + 5, 0, ROD_Z)
    carrier = union([carrier, hub(P0 + w + 4.99, P0 + w + 13)]) - cyl_y(ROD_R + 0.15, P0 + w, P0 + w + 14, 0, ROD_Z)
    for c, _ in phs:
        carrier -= cyl_y(1.25, P0 + w, P0 + w + 6, c[0], ROD_Z - c[1], 16)   # пины M3 саморезом
    ry = P0 + w + 15
    fr = frame(0.0, ry, rod_through_left=False, right_t=2 * BRG_W + 2)
    fr = union([fr, housing])
    return dict(name="Планетарный 12/18/48 соосный", no_cut=True, ratio=1 + zr / zs, frame=fr,
                parts=[("sun_12T", sun_m, "print", 1), ("planet_18T", pl_parts[0], "print", 3),
                       ("carrier", carrier, "print", 1)] + [(None, pm, "print", 0) for pm in pl_parts[1:]],
                motor=union([motor(0.0)]),
                extra=[("rod", rod(P0 + w + 5, ry + 2 * BRG_W + 2 + 25), "rod")],
                mesh=[(sun, (0, 0), 0, pl, phs[0][0], phs[0][1], zp / zs, False),
                      (pl, phs[0][0], phs[0][1], ring_cs, (0, 0), rph, zr / zp, True)],
                note="мотор соосно с палкой, самый компактный по высоте/длине; сложнее печать")


def v_gt2(t1=16, t2=60, L=160.0):
    p1, r1 = gt2_pulley(t1)
    p2, r2 = gt2_pulley(t2)
    a = solve_a(L, r1, r2)
    mx = -a
    w = 7.0
    fl = 1.2
    drv = union([ext(p1, P0 + fl, P0 + fl + w, mx), ext(circle(r1 + 1.5), P0, P0 + fl, mx),
                 ext(circle(r1 + 1.5), P0 + fl + w, P0 + 2 * fl + w, mx)]) - cyl_y(2.6, P0 - 1, P0 + 12, mx, ROD_Z, 24)
    big = union([ext(p2, P0 + fl, P0 + fl + w, 0), ext(circle(r2 + 1.5), P0, P0 + fl, 0),
                 ext(circle(r2 + 1.5), P0 + fl + w, P0 + 2 * fl + w, 0), hub(P0 + 2 * fl + w - 0.01, P0 + 2 * fl + w + 7)])
    big -= cyl_y(ROD_R + 0.15, P0 - 1, P0 + 30, 0, ROD_Z)
    for i in range(5):
        an = math.radians(90 + 72 * i)
        big -= cyl_y(4.5, P0 - 1, P0 + 2 * fl + w + 1, 11.5 * math.cos(an), ROD_Z + 11.5 * math.sin(an), 32)
    belt = ext(band((-a, 0), r1, (0, 0), r2, 1.4), P0 + fl + 0.3, P0 + fl + 6.3, 0)
    ry = P0 + 2 * fl + w + 9
    fr = frame(mx, ry, slots=2.5)
    return dict(name=f"Ремень GT2 {t1}T→{t2}T, петля {L:.0f}", ratio=t2 / t1, frame=fr,
                parts=[(f"pulley_GT2_{t1}T", drv, "print", 1), (f"pulley_GT2_{t2}T", big, "print", 1)],
                motor=union([motor(mx), nut_proxy(mx, P0 + 2 * fl + w)]),
                extra=[("rod", rod(-WALL_T - 12, ry + WALL_T + 25), "rod"), ("belt", belt, "belt")],
                note=f"тихо, гасит удары; межосевое {a:.1f} мм, натяжение пазами мотора")


def v_oring(d1=16.0, d2=44.0, cord=3.0):
    r1, r2 = d1 / 2, d2 / 2
    a = 40.0
    mx = -a
    w = 7.0

    def pulley(r, y0, x):
        body = ext(circle(r + 2), y0, y0 + w, x)
        groove = M.revolve(m3d.CrossSection.circle(cord / 2 + 0.2, 24).translate([r + cord / 2, 0]), 96)
        return body - groove.rotate([-90, 0, 0]).translate([x, y0 + w / 2, ROD_Z])
    drv = pulley(r1, P0, mx) - cyl_y(2.6, P0 - 1, P0 + w + 1, mx, ROD_Z, 24)
    big = union([pulley(r2, P0, 0), hub(P0 + w - 0.01, P0 + w + 7)]) - cyl_y(ROD_R + 0.15, P0 - 1, P0 + 30, 0, ROD_Z)
    for i in range(5):
        an = math.radians(90 + 72 * i)
        big -= cyl_y(4.5, P0 - 1, P0 + w + 1, 13.5 * math.cos(an), ROD_Z + 13.5 * math.sin(an), 32)
    belt = ext(band((-a, 0), r1 + cord / 2, (0, 0), r2 + cord / 2, cord), P0 + w / 2 - cord / 2, P0 + w / 2 + cord / 2, 0)
    L = belt_length(a, r1 + cord / 2, r2 + cord / 2)
    ry = P0 + w + 9
    fr = frame(mx, ry, slots=3.0)
    return dict(name=f"Круглый ремень Ø{cord:.0f} (O-ring) Ø{d1:.0f}→Ø{d2:.0f}", ratio=d2 / d1, frame=fr,
                parts=[("pulley_round_16", drv, "print", 1), ("pulley_round_44", big, "print", 1)],
                motor=union([motor(mx), nut_proxy(mx, P0 + w)]),
                extra=[("rod", rod(-WALL_T - 12, ry + WALL_T + 25), "rod"), ("belt", belt, "belt")],
                note=f"проскальзывает при ударе — бережёт мотор; кольцо ≈ L{L:.0f} мм с натягом 5–8 %")


def v_friction(d1=16.0, d2=50.0):
    r1, r2 = d1 / 2, d2 / 2
    a = r1 + r2 - 0.6     # преднатяг 0.6 мм в шину TPU
    mx = -a
    w = 8.0
    roll = ext(circle(r1), P0, P0 + w, mx)
    for i in range(18):          # насечка
        an = 2 * math.pi * i / 18
        roll -= cyl_y(0.6, P0 - 1, P0 + w + 1, mx + r1 * math.cos(an), ROD_Z + r1 * math.sin(an), 8)
    roll -= cyl_y(2.6, P0 - 1, P0 + w + 1, mx, ROD_Z, 24)
    drum = union([ext(circle(r2 - 3), P0, P0 + w, 0), hub(P0 + w - 0.01, P0 + w + 7)])
    drum -= cyl_y(ROD_R + 0.15, P0 - 1, P0 + 30, 0, ROD_Z)
    for i in range(5):
        an = math.radians(90 + 72 * i)
        drum -= cyl_y(4.0, P0 - 1, P0 + w + 1, 13 * math.cos(an), ROD_Z + 13 * math.sin(an), 32)
    tire = ext(circle(r2) - circle(r2 - 3.2), P0 + 0.5, P0 + w - 0.5, 0)
    ry = P0 + w + 9
    fr = frame(mx, ry, slots=2.0)
    return dict(name=f"Фрикцион: ролик Ø{d1:.0f} → барабан Ø{d2:.0f}", ratio=d2 / d1, frame=fr,
                parts=[("roller_16", roll, "print", 1), ("drum_50", drum, "print", 1), ("drum_tire_TPU", tire, "tpu", 1)],
                motor=union([motor(mx), nut_proxy(mx, P0 + w)]),
                extra=[("rod", rod(-WALL_T - 12, ry + WALL_T + 25), "rod")],
                note="мягкий старт, без шума; шина TPU изнашивается, теряет сцепление в воде/пыли")


VARIANTS = [
    ("V1", v_coupler), ("V2", v_bell_clamp), ("V3", lambda: v_spur(12, 48)), ("V4", lambda: v_spur(12, 48, True)),
    ("V5", v_two_stage), ("V6", v_internal), ("V7", v_planetary), ("V8", lambda: v_gt2(16, 60, 160)),
    ("V9", v_oring), ("V10", v_friction),
]


# ---------------------------------------------------------------- проверка, экспорт, рендер
def speed_kmh(ratio):
    rpm = KV * VOLT / ratio
    return rpm / 60 * math.pi * WHEEL_D / 1000 * 3.6


def check_variant(tag, v):
    ok = True
    msgs = []
    for (dc, dp, dph, nc, npos, nph, ratio, internal) in v.get("mesh", []):
        worst, eng = mesh_sweep(dc, dp, dph, nc, npos, nph, ratio, internal=internal)
        good = worst < 0.05 and eng > 0.05
        ok &= good
        msgs.append(f"клин {worst:.2f} / в зацеплении {eng:.2f} мм² {'✓' if good else '✗'}")
    moving = [p for _, p, _, _ in v["parts"]]
    for name, other in (("рама", v["frame"]), ("мотор", v["motor"])):
        for i, p in enumerate(moving):
            vol = (p ^ other).volume()
            if vol > 1.0 and not (name == "мотор" and v["parts"][i][0] in ("coupler", "bell_clamp")):
                ok = False
                msgs.append(f"{v['parts'][i][0]}∩{name} {vol:.0f} мм³")
    lo = v["frame"].bounding_box()
    allb = union([v["frame"], v["motor"]] + moving).bounding_box()
    dims = (allb[3] - allb[0], allb[4] - allb[1] + 0, allb[5] - allb[2])
    msgs.append(f"габарит {dims[0]:.0f}×{dims[1]:.0f}×{dims[2]:.0f}")
    return ok, msgs, dims


def render(vs, path):
    fig = plt.figure(figsize=(20, 15.5))
    light = np.array([0.4, -0.6, 0.75]); light /= np.linalg.norm(light)
    for k, (tag, v) in enumerate(vs):
        ax = fig.add_subplot(3, 4, k + 1, projection="3d")
        items = [(v["frame"], "frame"), (v["motor"], "motor")]
        items += [(p, c) for _, p, c, _ in v["parts"]] + [(p, c) for _, p, c in v["extra"]]
        tris_all, cols_all = [], []
        for man, col in items:
            tm = to_trimesh(man)
            shade = 0.35 + 0.65 * np.clip(tm.face_normals @ light, 0, 1)
            tris_all.append(tm.vertices[tm.faces])
            cols_all.append(np.c_[np.outer(shade, np.array(COL[col])), np.ones_like(shade)])
        T = np.concatenate(tris_all)
        # одна коллекция: сортировка по граням, а не по деталям — рама не перекрывает шестерни
        ax.add_collection3d(Poly3DCollection(T, facecolors=np.concatenate(cols_all), edgecolor="none"))
        V = T.reshape(-1, 3)
        lo, hi = V.min(0), V.max(0)
        c = (lo + hi) / 2; r = (hi - lo).max() / 2 * 0.8
        ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(c[2] - r, c[2] + r)
        ax.set_box_aspect((1, 1, 1)); ax.view_init(26, -35); ax.set_axis_off()
        ax.set_title(f"{tag}  {v['name']}  (1:{v['ratio']:.3g})\n≈{speed_kmh(v['ratio']):.0f} км/ч макс.",
                     fontsize=12)
    ax = fig.add_subplot(3, 4, 11); ax.set_axis_off()
    handles = [Patch(color=COL["frame"], label="печатная рама (стенки, основание)"),
               Patch(color=COL["print"], label="печатные муфты, шкивы, шестерни"),
               Patch(color=COL["motor"], label="мотор 2814 (макет)"),
               Patch(color=COL["rod"], label="палка Ø10 (макет)"),
               Patch(color=COL["belt"], label="ремень"),
               Patch(color=COL["tpu"], label="TPU")]
    ax.legend(handles=handles, loc="center", fontsize=13, frameon=False)
    ax = fig.add_subplot(3, 4, 12); ax.set_axis_off()
    ax.text(0.02, 0.5, f"Скорость: 2814 {KV}KV, 6S,\nколесо Ø{WHEEL_D:.0f}, без нагрузки.\n"
            f"Палка на {ROD_Z:.0f} мм над основанием,\nверх рамы {WALL_TOP:.0f} мм —\nвлезает под нижнюю плиту Mark4.",
            fontsize=12, va="center")
    plt.tight_layout()
    plt.savefig(path, dpi=80)


def clear_frame(v):
    """Прорези в основании/стенках под вращающиеся детали и ремень (зазор 1 мм)."""
    if v.get("no_cut"):
        return
    cuts = []
    for _, p, _, _ in v["parts"]:
        x0, y0, z0, x1, y1, z1 = p.bounding_box()
        r = max(x1 - x0, z1 - z0) / 2
        cuts.append(cyl_y(r + 1.0, y0 - 0.6, y1 + 0.6, (x0 + x1) / 2, (z0 + z1) / 2))
    for nm, p, col in v["extra"]:
        if col == "belt":
            x0, y0, z0, x1, y1, z1 = p.bounding_box()
            cuts.append(box(x0 - 1, x1 + 1, y0 - 1, y1 + 1, -1.5, 1.0))   # канавка под ремень
    v["frame"] = v["frame"] - union(cuts)


def main():
    vs = [(tag, f()) for tag, f in VARIANTS]
    for _, v in vs:
        clear_frame(v)
    all_ok = True
    for tag, v in vs:
        ok, msgs, _ = check_variant(tag, v)
        all_ok &= ok
        print(f"{tag:4s} {v['name']:42s} 1:{v['ratio']:<5.3g} ≈{speed_kmh(v['ratio']):4.0f} км/ч  "
              f"[{'OK' if ok else 'ПРОБЛЕМА'}] " + "; ".join(msgs))
    if "--check" in sys.argv:
        return 0 if all_ok else 1
    out = os.path.join(HERE, "stl", "drive")
    for tag, v in vs:
        d = os.path.join(out, f"{tag}")
        os.makedirs(d, exist_ok=True)
        to_trimesh(on_bed(v["frame"])).export(os.path.join(d, f"{tag}_frame_x1.stl"))
        for name, p, col, n in v["parts"]:
            if name and n:
                to_trimesh(on_bed(p.rotate([90, 0, 0]))).export(os.path.join(d, f"{tag}_{name}_x{n}.stl"))
    render(vs, os.path.join(HERE, "drive_variants.png"))
    print("drive_variants.png")
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
