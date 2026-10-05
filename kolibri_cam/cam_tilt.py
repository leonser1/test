"""
Наклоняемая камера для «Колибри» (Mark4 10" V2): 5 вариантов крепления с серво.

Камера micro 19×19 (крепление M2 по бокам, расстояние между боковинами рамы 19 мм).
Узел ставится НА МЕСТО ШТАТНОЙ КАМЕРЫ: переходной блок 18.8 мм шириной садится
между карбоновыми боковинами на те же 2 винта M2, от него вперёд-вниз выходит
поворотная часть. Серво наклоняет люльку с камерой от 0° (вперёд) до 90° (в пол).

Система координат: X вперёд, Y влево, Z вверх; (0,0,0) — ось штатных винтов камеры.
Запуск:  python3 cam_tilt.py           -> stl/V*/ + variants.png
         python3 cam_tilt.py --check   -> касания на всём ходе 0…90°
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "kolibri_car"))
from generate_car import M, m3d, box, cyl_y, cyl_z, union, hex_prism, to_trimesh, on_bed  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))

# ---------------- ПАРАМЕТРЫ (мм) — проверить по своей камере и серво ----------------
FRAME_GAP = 19.0        # между боковинами камеры рамы (замер Mark4 V2)
SIDE_T = 2.5            # толщина карбоновых боковин
# крепление на 2 передние стойки рамы (ЗАМЕРИТЬ!): ось стоек X=0, Y=±SO_S/2
SO_S = 30.0             # между осями передних стоек
SO_D = 5.0              # диаметр стойки (круглая алюминиевая M3)
PLATE_GAP = 35.0        # между плитами (Mark4 10" V2: стойки 35 мм)
PLATE_T = 2.0
PLATE_FRONT_X = 6.0     # нижняя плита выступает перед осью стоек
CLIP_WALL = 2.2         # стенка хомута на стойке (PETG/PA — защёлка, TPU — на натяг)
CAM_W = 19.0            # камера micro 19×19
CAM_L = 20.0            # корпус от передней грани назад
LENS_R, LENS_L = 7.0, 8.0
CAM_PIVOT = 8.0         # винты M2 камеры от передней грани назад
CLR = 0.3               # зазор посадок
CHEEK = 2.2             # щёчки люльки
# серво 9 г (SG90 / MG90S / ES08): корпус 22.8 × 12.2 × 22.7, ушки 32.2, вал смещён на 5.5
SV_L, SV_W, SV_H = 22.8, 12.2, 22.7
SV_TAB_L, SV_TAB_T, SV_TAB_FROM_TOP = 32.2, 2.5, 6.5
SV_SHAFT_OFF, SV_SPLINE_H = 5.5, 4.0
SV_SCREW = 27.8
HORN_T = 2.0
# ----------------------------------------------------------------------------------------

COL = {"frame": (0.25, 0.50, 0.62), "print": (0.93, 0.55, 0.18), "cam": (0.18, 0.18, 0.2),
       "servo": (0.30, 0.31, 0.34), "metal": (0.70, 0.70, 0.72), "carbon": (0.12, 0.12, 0.13),
       "esp": (0.15, 0.55, 0.25)}

HW = CAM_W / 2 + CLR          # полуширина гнезда камеры
CY = HW + CHEEK               # наружная сторона щеки люльки


def rot_y(man, deg, about=(0, 0, 0)):
    """Наклон вокруг оси Y (+deg — носом вниз)."""
    ax, _, az = about
    return man.translate([-ax, 0, -az]).rotate([0, deg, 0]).translate([ax, 0, az])


# ---------------------------------------------------------------- общие детали
def camera():
    """Камера в своих координатах: ось винтов в (0,0,0), объектив по +X."""
    body = box(CAM_PIVOT - CAM_L, CAM_PIVOT, -CAM_W / 2, CAM_W / 2, -CAM_W / 2, CAM_W / 2)
    lens = M.cylinder(LENS_L, LENS_R, LENS_R, 48).rotate([0, 90, 0]).translate([CAM_PIVOT, 0, 0])
    return union([body, lens])


def cradle(drive_left=None, lever_right=None, pin_left=True, pin_right=True, lever_y=None):
    """Люлька: щёки + задняя стенка с окном под кабель; ось поворота = ось винтов камеры."""
    x0, x1 = CAM_PIVOT - CAM_L - 2.0, CAM_PIVOT - 4.0
    parts = [box(x0, x1, HW, CY, -CAM_W / 2, CAM_W / 2), box(x0, x1, -CY, -HW, -CAM_W / 2, CAM_W / 2),
             box(x0, x0 + 2.0, -CY, CY, -CAM_W / 2, CAM_W / 2)]
    for s, pin in ((1, pin_left), (-1, pin_right)):
        if pin:   # полая цапфа Ø8 (внутри головка M2 камеры)
            c = cyl_y(4.0, HW, CY + 4.5, 0, 0)
            parts.append(c if s > 0 else c.mirror([0, 1, 0]))
    if drive_left == "horn":   # фланец под круглую качалку серво
        parts.append(cyl_y(10.5, CY - 0.01, CY + 2.0, 0, 0))
    if lever_right is not None:   # рычаг под тягу (V2): длина lever_right, вниз-назад
        L = lever_right
        y0, y1 = lever_y or (-CY - 2.5, -CY + 0.01)
        arm = M.batch_hull([cyl_y(4.0, y0, y1, 0, 0), cyl_y(3.0, y0, y1, -L, 0)])
        parts += [arm, cyl_y(4.0, y0, -HW, 0, 0)]          # цапфа доходит до рычага за ухом
    c = union(parts)
    cuts = [box(x0 - 1, x0 + 3, -5, 5, -3.5, 3.5)]                  # окно под шлейф
    for s in (1, -1):
        h = cyl_y(1.15, 0, CY + 6, 0, 0, 16)                            # M2 в камеру
        rc = cyl_y(2.1, CY - 0.6, CY + 6, 0, 0, 24)                     # головка M2 утоплена
        cuts += [h, rc] if s > 0 else [h.mirror([0, 1, 0]), rc.mirror([0, 1, 0])]
    if drive_left == "horn":
        for a in (0, 90, 180, 270):
            cuts.append(cyl_y(0.75, CY - 1, CY + 2.5, 7.5 * math.cos(math.radians(a)),
                              7.5 * math.sin(math.radians(a)), 12))
    if lever_right is not None:
        y0, y1 = lever_y or (-CY - 2.5, -CY + 0.01)
        cuts.append(cyl_y(1.0, y0 - 1, y1 + 1, -lever_right, 0, 12))
    return c - union(cuts)


def adapter(front_x=7.0, extra=None):
    """Блок на место штатной камеры: между боковинами, на 2 × M2 через боковины."""
    w = FRAME_GAP / 2 - 0.1
    blk = box(-7, front_x, -w, w, -8, 8)
    parts = [blk] + (extra or [])
    a = union(parts)
    cuts = [cyl_y(0.8, -w - 1, -w + 7, 0, 0, 12), cyl_y(0.8, w - 7, w + 1, 0, 0, 12),   # M2 саморез 6 мм
            cyl_z(1.4, -9, -2, -4, 0, 16)]                                               # M3 упор в нижнюю плиту
    return a - union(cuts)


def frame_proxy():
    """Плиты рамы и 2 передние стойки — для картинки и проверки."""
    g = PLATE_GAP / 2
    plates = [box(-60, PLATE_FRONT_X, -35, 35, g, g + PLATE_T), box(-60, PLATE_FRONT_X, -35, 35, -g - PLATE_T, -g)]
    so = [cyl_z(SO_D / 2, -g, g, 0, s * SO_S / 2, 32) for s in (-1, 1)]
    return union(plates + so)


def standoff_clip(front_x=7.0):
    """Надевается спереди на 2 передние стойки: 2 хомута-защёлки во всю высоту + поперечина к узлу.
    Прорезь хомута смотрит назад: узел надавливается спереди, хомуты защёлкиваются на стойках.
    В каждом хомуте отверстие M3 поперёк — стяжной винт, если защёлка слабая."""
    g = PLATE_GAP / 2 - 0.2
    ro, ri = SO_D / 2 + CLIP_WALL, SO_D / 2 + 0.1
    parts = []
    for sy in (-1, 1):
        y = sy * SO_S / 2
        parts.append(cyl_z(ro, -g, g, 0, y, 48))
        parts.append(box(0, front_x, y - ro, y + ro, -g, g))            # щека хомута вперёд
    parts.append(box(0, front_x, -SO_S / 2, SO_S / 2, 6, 10))            # верхняя поперечина
    parts.append(box(0, front_x, -SO_S / 2, SO_S / 2, -10, -6))          # нижняя поперечина
    c = union(parts)
    cuts = []
    for sy in (-1, 1):
        y = sy * SO_S / 2
        cuts.append(cyl_z(ri, -g - 1, g + 1, 0, y, 48))
        cuts.append(box(-ro - 1, -ri * 0.55, y - ri * 0.82, y + ri * 0.82, -g - 1, g + 1))   # защёлка: зев 0.82·D
        for z in (-g + 6, g - 6):
            cuts.append(M.cylinder(2 * ro + 2, 1.6, 1.6, 16).rotate([90, 0, 0]).translate([1.5, y + ro + 1, z]))
    return c - union(cuts)


def servo(shaft_axis="y+", horn_disc=True):
    """Серво 9 г, вал по +Y из (0,0,0) (верх шлица в y=0), корпус в -Y, длина вдоль X (вал у +X торца)."""
    top = -SV_SPLINE_H
    body = box(-SV_L / 2 - SV_SHAFT_OFF, SV_L / 2 - SV_SHAFT_OFF, top - SV_H, top, -SV_W / 2, SV_W / 2)
    tabs = box(-SV_TAB_L / 2 - SV_SHAFT_OFF, SV_TAB_L / 2 - SV_SHAFT_OFF,
               top - SV_TAB_FROM_TOP - SV_TAB_T, top - SV_TAB_FROM_TOP, -SV_W / 2, SV_W / 2)
    spl = cyl_y(2.4, top, 0, 0, 0, 24)
    parts = [body, tabs, spl]
    if horn_disc:
        parts.append(cyl_y(10.0, 0, HORN_T, 0, 0))
    return union(parts)


def servo_mount_plate(y_face, x_c, z_c, thick=2.5, along="x"):
    """Пластина с окном под корпус серво, ушки ложатся на неё (плоскость XZ на y = y_face … y_face-thick)."""
    pl = box(x_c - SV_TAB_L / 2 - 2.5, x_c + SV_TAB_L / 2 + 2.5, y_face - thick, y_face, z_c - SV_W / 2 - 3, z_c + SV_W / 2 + 3)
    win = box(x_c - SV_L / 2 - 0.3, x_c + SV_L / 2 + 0.3, y_face - thick - 1, y_face + 1, z_c - SV_W / 2 - 0.3, z_c + SV_W / 2 + 0.3)
    holes = [cyl_y(0.85, y_face - thick - 1, y_face + 1, x_c + s * SV_SCREW / 2, z_c, 12) for s in (-1, 1)]
    return pl - union([win] + holes)


# ---------------------------------------------------------------- варианты
R_SWEEP = 17.5          # радиус, который заметает камера с люлькой вокруг оси
P1 = np.array([28.0, 0.0, -14.0])    # ось наклона V1, V3: впереди рамы, ниже верхней балки (z 6)


def place_servo_side(px, pz, y_horn_face, side=1):
    """Серво сбоку, вал соосно оси наклона, качалка на фланце люльки."""
    sv = servo().rotate([0, 0, 180]) if side > 0 else servo()
    # servo(): вал +Y; повернуть на 180 вокруг Z -> вал смотрит в -Y (к камере), корпус в +Y
    sv = sv.translate([px, y_horn_face + HORN_T if side > 0 else -(y_horn_face + HORN_T), pz])
    return sv


def v1():
    """Прямой привод: серво сбоку, вал = ось наклона."""
    px, _, pz = P1
    yh = CY + 2.0                                   # фланец качалки
    sv = servo().rotate([0, 0, 180]).translate([px, yh + HORN_T, pz])
    sv = sv.rotate([0, 0, 0])
    tab_face = yh + HORN_T + SV_SPLINE_H + SV_TAB_FROM_TOP
    mount = servo_mount_plate(tab_face + 5.0, px + SV_SHAFT_OFF, pz)
    ear_r = box(px - 6, px + 6, -CY - 5.0, -CY - 0.5, pz, 10)        # правое ухо с отверстием Ø8.3
    ear_r = union([ear_r, cyl_y(6, -CY - 5.0, -CY - 0.5, px, pz)]) - cyl_y(4.15, -CY - 6, -CY, px, pz)
    beam = box(5, px + 6, -CY - 5.0, tab_face + 5.0, 6, 10)            # балка сверху
    post = box(px + SV_SHAFT_OFF - SV_TAB_L / 2 - 2.5, px + SV_SHAFT_OFF + SV_TAB_L / 2 + 2.5,
               tab_face + 2.5, tab_face + 5.0, pz + SV_W / 2 + 3 - 0.01, 10)
    body = union([standoff_clip(7.0), beam, ear_r, mount, post])
    cr = cradle(drive_left="horn", pin_left=False)
    return dict(name="Прямой привод: серво сбоку на оси", pivot=(px, pz), body=body, cradle=cr, servo=sv,
                extra_move=[], note="2 печатные детали, минимум люфта; серво торчит сбоку на ~30 мм",
                parts={"mount": body, "cradle": cr})


def _side_servo_mount(sv_x, sv_z, y_spline_top, top_z=10.0):
    """Пластина под ушки серво (вал в -Y, корпус наружу в +Y) и стойка до верхней балки."""
    T = y_spline_top + SV_SPLINE_H + SV_TAB_FROM_TOP       # грань ушек к камере
    xc = sv_x + SV_SHAFT_OFF
    plate = box(xc - 18.6, xc + 18.6, T + 2.5, T + 5.0, sv_z - 9.1, sv_z + 9.1) - union([
        box(xc - 11.7, xc + 11.7, T + 1.5, T + 6, sv_z - 6.4, sv_z + 6.4),
        cyl_y(0.85, T + 1.5, T + 6, xc - 13.9, sv_z, 12), cyl_y(0.85, T + 1.5, T + 6, xc + 13.9, sv_z, 12)])
    post = box(xc - 18.6, xc + 18.6, T + 2.5, T + 5.0, sv_z + 9.0, top_z)
    return union([plate, post]), T + 5.0


def _ear(px, pz, y0, y1, top_z=10.0):
    e = union([box(px - 6, px + 6, y0, y1, pz, top_z), cyl_y(6, y0, y1, px, pz)])
    return e - cyl_y(4.15, y0 - 1, y1 + 1, px, pz)


def v2():
    """Тяга-параллелограмм сбоку: серво сбоку ниже оси, качалка и рычаг люльки равны и параллельны."""
    px, _, pz = P1
    L = 10.0
    sx, sz = px, pz - 26.0                     # серво под осью, сбоку от камеры
    ly = (CY + 5.4, CY + 7.7)                  # рычаг люльки снаружи левого уха (ухо CY+0.5 … CY+5)
    y_rod0, y_rod1 = ly[1] + 0.2, ly[1] + 2.2  # тяга
    y_horn = y_rod1 + 0.2                      # качалка y_horn … y_horn+HORN_T
    sv = servo(horn_disc=False).rotate([0, 0, 180]).translate([sx, y_horn + HORN_T, sz])
    mount, y_out = _side_servo_mount(sx, sz, y_horn + HORN_T)
    beam = box(5, px + 6 + SV_SHAFT_OFF + 12.6, -CY - 5.0, y_out, 6, 10)
    body = union([standoff_clip(7.0), beam, mount, _ear(px, pz, CY + 0.5, CY + 5.0), _ear(px, pz, -CY - 5.0, -CY - 0.5)])
    # рычаг слева: зеркало правого варианта cradle(lever_right) — строим напрямую
    cr = cradle()
    arm = M.batch_hull([cyl_y(4.0, ly[0], ly[1], 0, 0), cyl_y(3.0, ly[0], ly[1], -L, 0)])
    cr = union([cr, arm, cyl_y(4.0, HW, ly[1], 0, 0)]) - union([cyl_y(1.0, ly[0] - 1, ly[1] + 1, -L, 0, 12),
                                                            cyl_y(2.1, CY - 0.6, ly[1] + 1, 0, 0, 24)])
    rod_len = abs(pz - sz)
    rod = M.batch_hull([cyl_y(2.6, y_rod0, y_rod1, 0, 0), cyl_y(2.6, y_rod0, y_rod1, rod_len, 0)])
    rod = rod - union([cyl_y(1.0, y_rod0 - 1, y_rod1 + 1, 0, 0, 12), cyl_y(1.0, y_rod0 - 1, y_rod1 + 1, rod_len, 0, 12)])
    return dict(name="Тяга сбоку (параллелограмм 1:1)", pivot=(px, pz), body=body, cradle=cr, servo=sv,
                rod=dict(L=L, sx=sx, sz=sz, y0=y_rod0, y1=y_rod1, yh=y_horn + HORN_T, len=rod_len), extra_move=[],
                note="серво сбоку ниже камеры, сверху узко; ход 1:1; люфт в 2 шарнирах M2",
                parts={"mount": body, "cradle_lever": cr, "pushrod": rod})


def v3():
    """Шестерни 2:1 сбоку: серво сбоку спереди-снизу от оси, шестерня на качалке крутит венец на люльке."""
    import drive_variants as dv
    px, _, pz = P1
    m = 0.8
    z1, z2 = 16, 32
    a = m * (z1 + z2) / 2 + 0.1
    ang = math.radians(-35)                       # серво впереди-снизу (за камерой — плата ESP)
    sx, sz = px + a * math.cos(ang), pz + a * math.sin(ang)
    yg = CY + 0.5
    gw = 4.0
    pin_cs = dv.involute(z1, m, shift=0.3, backlash=0.12)
    gear_cs = dv.involute(z2, m, shift=-0.3, backlash=0.12)
    dx, dy = sx - px, -(sz - pz)                  # центр шестерни в локальной 2D-системе венца
    ph, _ = dv.phase_search(gear_cs, (0, 0), pin_cs, (dx, dy), 360 / z1)
    gear = M.extrude(gear_cs, gw).rotate([-90, 0, 0]).translate([0, yg, 0])
    pinion = M.extrude(pin_cs.rotate(ph), gw).rotate([-90, 0, 0]).translate([sx, yg, sz])
    pinion = pinion - cyl_y(1.0, yg - 1, yg + gw + 1, sx, sz, 12)
    sv = servo().rotate([0, 0, 180]).translate([sx, yg + gw + HORN_T, sz])
    mount, y_out = _side_servo_mount(sx, sz, yg + gw + HORN_T)
    ear_l = _ear(px, pz, yg + gw + 0.5, yg + gw + 4.5)
    beam = box(5, sx + SV_SHAFT_OFF + 18.6, -CY - 5.0, y_out, 6, 10)
    body = union([standoff_clip(7.0), beam, mount, ear_l, _ear(px, pz, -CY - 5.0, -CY - 0.5)])
    body = body - cyl_y(a - 13.5 + 1.0, yg - 0.5, yg + gw + 0.5, px, pz)     # чтобы ухо не мешало венцу
    cr = union([cradle(pin_left=False), gear, cyl_y(4.0, HW, yg + gw + 4.5, 0, 0)])
    cr = cr - cyl_y(2.1, CY - 0.6, yg + gw + 5, 0, 0, 24)
    return dict(name="Шестерни 2:1 сбоку (серво 180° → камера 90°)", pivot=(px, pz), body=body, cradle=cr, servo=sv,
                pinion=dict(m=pinion, sx=sx, sz=sz, ratio=z2 / z1, cs=pin_cs, ph=ph, gcs=gear_cs, d=(dx, dy)),
                extra_move=[], note="вдвое точнее и сильнее; ход серво 180°; модуль 0.8 — печать слоем 0.12",
                parts={"mount": body, "cradle_gear32": cr, "pinion16_on_horn": pinion})


def v4():
    """Выносной рычаг: камера на конце рычага 26 мм, рычаг опускается вперёд-вниз."""
    ax_x, ax_z = 19.0, -14.0           # ось рычага (перед хомутами, ниже балки)
    R = 26.0
    yh = CY + 2.0
    sv = servo().rotate([0, 0, 180]).translate([ax_x, yh + HORN_T + 1.5, ax_z])
    tab_face = yh + HORN_T + 1.5 + SV_SPLINE_H + SV_TAB_FROM_TOP
    mount = box(ax_x + SV_SHAFT_OFF - 18.6, ax_x + SV_SHAFT_OFF + 18.6, tab_face + 2.5, tab_face + 5.0, ax_z - 9.1, ax_z + 9.1) - union([
        box(ax_x + SV_SHAFT_OFF - 11.7, ax_x + SV_SHAFT_OFF + 11.7, tab_face + 1.5, tab_face + 6, ax_z - 6.4, ax_z + 6.4),
        cyl_y(0.85, tab_face + 1.5, tab_face + 6, ax_x + SV_SHAFT_OFF - 13.9, ax_z, 12),
        cyl_y(0.85, tab_face + 1.5, tab_face + 6, ax_x + SV_SHAFT_OFF + 13.9, ax_z, 12)])
    ear_r = union([box(ax_x - 6, ax_x + 6, -CY - 5.0, -CY - 0.5, ax_z, 10), cyl_y(6, -CY - 5.0, -CY - 0.5, ax_x, ax_z)]) \
        - cyl_y(4.15, -CY - 6, -CY, ax_x, ax_z)
    beam = box(5, ax_x + 6, -CY - 5.0, tab_face + 5.0, 6, 10)
    post = box(ax_x + SV_SHAFT_OFF - 18.6, ax_x + SV_SHAFT_OFF + 18.6, tab_face + 2.5, tab_face + 5.0, ax_z + 9.0, 10)
    body = union([standoff_clip(7.0), beam, ear_r, mount, post])
    # рычаг-вилка: от оси вперёд на R, на конце люлька (камера жёстко, смотрит вдоль рычага)
    cam_off = R
    arms = []
    for s in (1, -1):
        a = M.batch_hull([cyl_y(5.0, HW, CY, 0, 0), box(cam_off + CAM_PIVOT - CAM_L - 2.0, cam_off + CAM_PIVOT - 4.0, HW, CY, -CAM_W / 2, CAM_W / 2)])
        arms.append(a if s > 0 else a.mirror([0, 1, 0]))
    back = box(cam_off + CAM_PIVOT - CAM_L - 2.0, cam_off + CAM_PIVOT - CAM_L, -CY, CY, -CAM_W / 2, CAM_W / 2)
    hub_l = cyl_y(10.5, CY - 0.01, CY + 2.0, 0, 0)
    pin_r = cyl_y(4.0, -CY - 4.5, -HW, 0, 0)
    arm = union(arms + [back, hub_l, pin_r])
    cuts = [box(cam_off + CAM_PIVOT - CAM_L - 3, cam_off + CAM_PIVOT - CAM_L + 1, -5, 5, -3.5, 3.5),
            box(-1, cam_off + CAM_PIVOT - CAM_L - 2.0 - 0.01, -HW, HW, -12, 12)]   # проход между щеками
    for s in (1, -1):
        h = cyl_y(1.15, 0, CY + 6, cam_off, 0, 16)
        cuts.append(h if s > 0 else h.mirror([0, 1, 0]))
    for a_ in (0, 90, 180, 270):
        cuts.append(cyl_y(0.75, CY - 1, CY + 2.5, 7.5 * math.cos(math.radians(a_)), 7.5 * math.sin(math.radians(a_)), 12))
    arm = arm - union(cuts)
    return dict(name="Выносной рычаг 26 мм вперёд-вниз", pivot=(ax_x, ax_z), body=body, cradle=arm, servo=sv,
                cam_offset=cam_off, extra_move=[],
                note="в пол смотрит из-под носа, рама и плита не видны; камера уходит на 26 мм — длиннее шлейф",
                parts={"mount": body, "swing_arm": arm})


def v5():
    """Гондола под нижней плитой: вилка свисает под рамой, серво внутри вилки, камера ниже плиты."""
    pb = -PLATE_GAP / 2 - PLATE_T                 # низ нижней плиты
    px, pz = 21.0, pb - 4.0 - R_SWEEP - 1.0
    yh = CY + 2.0
    sv = servo().rotate([0, 0, 180]).translate([px, yh + HORN_T, pz])
    tab_face = yh + HORN_T + SV_SPLINE_H + SV_TAB_FROM_TOP
    mount = box(px + SV_SHAFT_OFF - 18.6, px + SV_SHAFT_OFF + 18.6, tab_face + 2.5, tab_face + 5.0, pz - 9.1, pz + 9.1) - union([
        box(px + SV_SHAFT_OFF - 11.7, px + SV_SHAFT_OFF + 11.7, tab_face + 1.5, tab_face + 6, pz - 6.4, pz + 6.4),
        cyl_y(0.85, tab_face + 1.5, tab_face + 6, px + SV_SHAFT_OFF - 13.9, pz, 12),
        cyl_y(0.85, tab_face + 1.5, tab_face + 6, px + SV_SHAFT_OFF + 13.9, pz, 12)])
    ear_r = union([box(px - 6, px + 6, -CY - 5.0, -CY - 0.5, pz, pb - 4.0 + 0.01), cyl_y(6, -CY - 5.0, -CY - 0.5, px, pz)]) \
        - cyl_y(4.15, -CY - 6, -CY, px, pz)
    post = box(px + SV_SHAFT_OFF - 18.6, px + SV_SHAFT_OFF + 18.6, tab_face + 2.5, tab_face + 5.0, pz + 9.0, pb - 4.0 + 0.01)
    # верхняя полка под нижней плитой рамы: крепится к блоку и ложится под плиту
    shelf = box(PLATE_FRONT_X + 0.5, px + 22, -CY - 5.0, tab_face + 5.0, pb - 4.0, pb)
    neck = box(PLATE_FRONT_X + 0.5, PLATE_FRONT_X + 6.0, -SO_S / 2, SO_S / 2, pb - 4.0, -6)   # перед краем нижней плиты
    body = union([standoff_clip(7.0), shelf, neck, ear_r, mount, post])
    body = body - box(-60, PLATE_FRONT_X + 0.3, -40, 40, pb - 0.01, -PLATE_GAP / 2 + 0.01)   # не залезать в нижнюю плиту
    cr = cradle(drive_left="horn", pin_left=False)
    return dict(name="Гондола под рамой (серво внутри вилки)", pivot=(px, pz), body=body, cradle=cr, servo=sv,
                extra_move=[], note="камера ниже плиты: ничего не мешает и вперёд, и вниз; самый низкий — бережёт посадку",
                parts={"mount": body, "cradle": cr})


VARIANTS = [("V1", v1), ("V2", v2), ("V3", v3), ("V4", v4), ("V5", v5)]


# ---------------------------------------------------------------- сборка в положении tilt
def posed(v, tilt):
    px, pz = v["pivot"]
    out = [(frame_proxy(), "carbon"), (v["body"], "frame"), (v["servo"], "servo")]
    if "cam_offset" in v:                         # V4: камера на рычаге
        mov = union([v["cradle"]]).translate([px, 0, pz])
        cam = camera().translate([px + v["cam_offset"], 0, pz])
        out += [(rot_y(mov, tilt, (px, 0, pz)), "print"), (rot_y(cam, tilt, (px, 0, pz)), "cam")]
    else:
        mov = v["cradle"].translate([px, 0, pz])
        cam = camera().translate([px, 0, pz])
        out += [(rot_y(mov, tilt, (px, 0, pz)), "print"), (rot_y(cam, tilt, (px, 0, pz)), "cam")]
    if "rod" in v:                                # V2: тяга-параллелограмм
        r = v["rod"]
        L = r["L"]
        a = math.radians(tilt)
        # конец рычага люльки и качалки поворачиваются одинаково (параллелограмм)
        p_cr = np.array([px - L * math.cos(a), pz + L * math.sin(a)])
        p_sv = np.array([r["sx"] - L * math.cos(a), r["sz"] + L * math.sin(a)])
        rod = M.batch_hull([cyl_y(2.6, r["y0"], r["y1"], *p_cr), cyl_y(2.6, r["y0"], r["y1"], *p_sv)])
        horn = M.batch_hull([cyl_y(3.5, r["yh"] - HORN_T, r["yh"], r["sx"], r["sz"]),
                             cyl_y(2.5, r["yh"] - HORN_T, r["yh"], *p_sv)])
        out += [(rod, "print"), (horn, "metal")]
    if "pinion" in v:
        p = v["pinion"]
        out.append((rot_y(p["m"], -tilt * p["ratio"], (p["sx"], 0, p["sz"])), "print"))
    return out


def check():
    import drive_variants as dv
    ok = True
    for tag, f in VARIANTS:
        v = f()
        worst, where = 0.0, ""
        back = 1e9
        for t in range(0, 91, 5):
            items = posed(v, t)
            fixed = union([items[0][0], items[1][0], items[2][0]])
            for m, c in items[3:]:
                if c == "metal":          # качалка серво крутится вместе с валом — с серво не сравниваем
                    vol = (m ^ union([items[0][0], items[1][0]])).volume()
                else:
                    vol = (m ^ fixed).volume()
                if vol > worst:
                    worst, where = vol, f"при {t}°"
                back = min(back, m.bounding_box()[0])
        sv_hit = (v["servo"] ^ v["body"]).volume()
        fr_hit = (v["body"] ^ frame_proxy()).volume() + (v["servo"] ^ frame_proxy()).volume()
        if fr_hit > 1.0:
            where += f"; крепление/серво∩рама {fr_hit:.0f} мм³"
            sv_hit = max(sv_hit, fr_hit)
        if sv_hit > 1.0:
            where += f"; серво∩крепление {sv_hit:.0f} мм³"
        back = min(back, v["servo"].bounding_box()[0])
        body_back = 0.0   # хомуты на стойках — штатное место, дальше назад ничего
        behind = min(back, body_back)
        mesh = ""
        if "pinion" in v:
            p = v["pinion"]
            w, e = dv.mesh_sweep(p["gcs"], (0, 0), 0, p["cs"], p["d"], p["ph"], 1 / p["ratio"], steps=30)
            mesh = f"; зацепление клин {w:.2f} / зуб {e:.2f} мм²"
            ok &= w < 0.05 and e > 0.05
        good = worst < 1.0 and sv_hit <= 1.0 and behind >= -SO_D / 2 - CLIP_WALL - 0.01
        ok &= good
        print(f"{tag} {v['name']:44s} касаний 0…90°: {'нет' if worst < 1 else f'{worst:.1f} мм³ {where}'}; "
              f"самая задняя точка X={behind:.1f} (за стойками {'пусто' if behind >= -SO_D / 2 - CLIP_WALL - 0.01 else 'ЗАНЯТО'}){mesh}")
    return ok


def render(path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    light = np.array([0.4, -0.6, 0.75]); light /= np.linalg.norm(light)
    fig = plt.figure(figsize=(27, 12))
    for k, (tag, f) in enumerate(VARIANTS):
        v = f()
        for row, tilt in enumerate((0, 90)):
            T, C = [], []
            esp = box(-13.9, -5, -12, 12, -6, -4)        # край платы ESP за стойками (место занято)
            for man, col in posed(v, tilt) + [(esp, "esp")]:
                tm = to_trimesh(man)
                if len(tm.faces) == 0:
                    continue
                sh = 0.35 + 0.65 * np.clip(tm.face_normals @ light, 0, 1)
                T.append(tm.vertices[tm.faces]); C.append(np.c_[np.outer(sh, COL[col]), np.ones_like(sh)])
            T = np.concatenate(T); C = np.concatenate(C)
            keep = T[:, :, 0].mean(1) > -14     # только узел камеры и край рамы
            T, C = T[keep], C[keep]
            ax = fig.add_subplot(2, 5, row * 5 + k + 1, projection="3d")
            ax.add_collection3d(Poly3DCollection(T, facecolors=C, edgecolor="none"))
            V = T.reshape(-1, 3); lo, hi = V.min(0), V.max(0); c = (lo + hi) / 2; r = (hi - lo).max() / 2 * 0.72
            ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(c[2] - r, c[2] + r)
            ax.set_box_aspect((1, 1, 1)); ax.view_init(16, 38); ax.set_axis_off()
            ttl = f"{tag}  {v['name']}" if row == 0 else ""
            ax.set_title((ttl + "\n" if ttl else "") + ("камера вперёд (0°)" if tilt == 0 else "камера в пол (90°)"),
                         fontsize=11)
    plt.tight_layout()
    plt.savefig(path, dpi=75)


def export():
    for tag, f in VARIANTS:
        v = f()
        d = os.path.join(HERE, "stl", tag)
        os.makedirs(d, exist_ok=True)
        for name, man in v["parts"].items():
            to_trimesh(on_bed(man)).export(os.path.join(d, f"{tag}_{name}.stl"))
    render(os.path.join(HERE, "variants.png"))


if __name__ == "__main__":
    if "--check" in sys.argv:
        sys.exit(0 if check() else 1)
    export()
