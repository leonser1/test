"""
Поворот камеры v2 (по образцу с фото, переделано после неудачной печати).

Что изменено против photo_mount.py:
* Уши — ОТДЕЛЬНЫЕ детали, печатаются плашмя (слои вдоль уха — не ломается),
  крепятся к толстым стойкам рамы 2 × M3 (гайки утоплены в ухе).
* Качалка MG90S — СНАРУЖИ левого уха, в утопленном кармане по форме качалки;
  шлиц проходит сквозь отверстие уха. Момент держит карман, саморезы — только прижим.
  Центральный винт качалки доступен снаружи.
* Правая опора — M3×12 сквозь ухо в самоконтрящуюся гайку в бобышке люльки
  (не зажимает ухо, без люфта). Гайка вставляется до установки серво.
* Гнездо серво 23.6 × 13.0 (FDM-запас), полки под ушки на 15.8 мм от дна,
  вырез под кабель в переднем торце внизу.
* Ушки камеры длинные, паз M2 на 12 мм — камеры длиной 20…30 мм.

Координаты: X вперёд, Y влево, Z вверх; стойки рамы X=0, Y=±SO_S/2; Z=0 — середина
между плитами. Запуск: python3 tilt_v2.py [--check]
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "kolibri_car"))
from generate_car import M, m3d, box, cyl_y, cyl_z, union, hex_prism, to_trimesh, on_bed  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))

# ---------------- рама дрона (замеры) ----------------
SO_S = 33.0             # между центрами передних стоек (замер)
SO_D = 5.0              # диаметр стойки
PLATE_GAP = 35.0        # между плитами Mark4 10" V2
PLATE_T = 2.0
PLATE_FRONT_X = 6.0     # нижняя плита выступает перед стойками
# ---------------- MG90S ----------------
SV_L, SV_W, SV_H = 22.8, 12.2, 22.5     # корпус: длина, ширина, высота до верха корпуса
SV_BOSS_H = 4.0                          # выступ под шлицем над корпусом
SV_SPL_H = 3.5                           # шлиц над выступом
SV_OFF = 5.5                             # вал смещён от центра корпуса к заднему торцу
SV_TAB_L, SV_TAB_T, SV_TAB_Z = 32.5, 2.5, 16.0   # ушки: длина, толщина, низ ушка от дна (замер 16 мм)
SV_SCREW = 27.8
FIT = 0.4               # зазор гнезда на сторону (FDM)
# ---------------- камера ----------------
CAM_W = 19.0
CAM_PIVOT = 8.0         # винты M2 камеры от передней грани
CAM_LENGTHS = (20.0, 28.0)   # проверяем самую короткую и длинную
# ---------------- механизм ----------------
X_P = 23.0              # ось поворота: заметаемый радиус люльки 14.7 + рама-основание до X 6.5
WALL = 2.4              # стенки люльки
FLOOR = 3.0             # дно люльки (сторона оси M3)
BOSS_L, BOSS_R = 6.0, 5.5    # бобышка под гайку M3 нейлок
GAP = 0.5               # люлька — ухо
EAR_T_L, EAR_T_R = 6.0, 5.0
HORN_POCKET = 3.5       # глубина кармана качалки (дно кармана ≈ уровень верха шлица)
BASE_X = (2.5, 6.5)     # вертикальная рама-основание перед стойками
FLANGE_T = 4.0          # фланец съёмного левого уха
FLANGE_Y1 = None
TAB_CAM_T = 2.5
FRONT_BLK = 3.0         # передний блок за полкой серво: от него идут ушки камеры
SLOT = (16.0, 26.0)     # паз M2 камеры от передней полки ушка серво: камеры 20…30 мм
# --------------------------------------------------------

G = PLATE_GAP / 2
Z_FR0 = -G + 0.2
Z_FR1 = Z_FR0 + 4.0
# Y-раскладка (Y+ — сторона шлица, левое ухо)
STACK = GAP + BOSS_L + FLOOR + SV_H + SV_BOSS_H + GAP
Y_ER_IN = -STACK / 2                    # внутренняя сторона правого уха
Y_BOSS = Y_ER_IN + GAP                  # торец бобышки
Y_FLOOR_OUT = Y_BOSS + BOSS_L
Y_SV_BOT = Y_FLOOR_OUT + FLOOR
Y_CASE_TOP = Y_SV_BOT + SV_H
Y_BOSS_TOP = Y_CASE_TOP + SV_BOSS_H
Y_SPL_TOP = Y_BOSS_TOP + SV_SPL_H
Y_EL_IN = Y_BOSS_TOP + GAP              # внутренняя сторона левого уха
Y_EL_OUT = Y_EL_IN + EAR_T_L
Y_ER_OUT = Y_ER_IN - EAR_T_R
# X относительно вала (при 0°)
XR = -(SV_L / 2 - SV_OFF)               # задний торец корпуса
XF = SV_L / 2 + SV_OFF                  # передний торец
XC = SV_OFF                             # центр корпуса
X_WALL_F = XF + FIT                     # внутренняя грань передней стенки
X_FRONT = X_WALL_F + WALL               # наружная грань — от неё ушки камеры
ZW = SV_W / 2 + FIT                     # полуширина гнезда
Y_TAB_LOW = Y_SV_BOT + SV_TAB_Z         # низ ушек серво = верх полок
CAM_TAB_IN = CAM_W / 2 + 0.25
X_LEDGE_F = XC + SV_TAB_L / 2 + 0.6     # передняя грань полки под ушко серво
R_SWEEP = max(math.hypot(SV_TAB_L / 2 - XC + 0.6, ZW + WALL), math.hypot(-XR + FIT + WALL, ZW + WALL))
Z_P = 0.0                                # ось по середине между плитами (как штатная камера)
Y_FL1 = Y_EL_OUT + 8.0                   # фланец левого уха до сюда
BOLTS = [(Y_EL_OUT + 4.0, 9.0), (Y_EL_OUT + 4.0, -9.0)]   # M3 вдоль X: фланец → рама, гайки сзади рамы
assert X_P - R_SWEEP - 1.0 >= BASE_X[1], "люлька задевает раму-основание"

COL = {"frame": (0.25, 0.42, 0.75), "ear": (0.20, 0.55, 0.80), "print": (0.93, 0.55, 0.18),
       "cam": (0.16, 0.16, 0.18), "servo": (0.30, 0.31, 0.34), "metal": (0.75, 0.75, 0.78),
       "carbon": (0.12, 0.12, 0.13), "esp": (0.15, 0.55, 0.25)}


# ---------------------------------------------------------------- рама на стойки (как на фото)
def make_frame():
    """Трубки на стойки + вертикальная рама с окном + правое ухо (литое).
    Печать: лёжа на правом ухе (слои вдоль уха)."""
    ro = SO_D / 2 + 2.5
    gz = G - 0.2
    parts = []
    for s_ in (-1, 1):
        y = s_ * SO_S / 2
        tube = cyl_z(ro, -gz, gz, 0, y, 48)
        keel = box(-0.5, 0.5, y - ro - 1.8, y - ro + 0.5, -gz, gz)      # киль 45° под печать на боку
        parts.append(M.batch_hull([tube, keel]))
    parts.append(box(BASE_X[0], BASE_X[1], Y_ER_OUT, Y_FL1, -gz, gz))   # рама-основание
    # правое ухо: от рамы вперёд до оси, толщина EAR_T_R, косынки к раме
    ear = M.batch_hull([box(BASE_X[0], BASE_X[1] + 2, Y_ER_OUT, Y_ER_IN, -gz, gz),
                        cyl_y(10.5, Y_ER_OUT, Y_ER_IN, X_P, Z_P, 48)])
    parts.append(ear)
    f = union(parts)
    cuts = [cyl_z(SO_D / 2 + 0.15, -G - 1, G + 1, 0, s_ * SO_S / 2, 48) for s_ in (-1, 1)]
    cuts.append(box(BASE_X[0] - 1, BASE_X[1] + 1, -11.5, 11.5, -9.0, 9.0))       # окно (кабель серво)
    cuts.append(cyl_y(1.7, Y_ER_OUT - 1, Y_ER_IN + 1, X_P, Z_P, 24))              # ось M3
    for y, z in BOLTS:    # M3 фланца: гайка в гнезде на задней стороне рамы
        cuts.append(cyl_x(1.65, BASE_X[0] - 1, BASE_X[1] + 1, y, z))
        cuts.append(hex_prism(5.8, 0, 2.6).rotate([0, 90, 0]).translate([BASE_X[0] - 0.01, y, z]))
    return f - union(cuts)


def cyl_x(r, x0, x1, y, z, seg=24):
    return M.cylinder(x1 - x0, r, r, seg).rotate([0, 90, 0]).translate([x0, y, z])


def make_ear(right=False):
    """Съёмное левое ухо: пластина + фланец к раме. Печать: внутренней стороной на стол."""
    gz = G - 0.2
    y0, y1 = Y_EL_IN, Y_EL_OUT
    plate = M.batch_hull([box(BASE_X[1] + 0.1, BASE_X[1] + 0.1 + FLANGE_T, y0, y1, -gz + 2, gz - 2),
                          cyl_y(10.5, y0, y1, X_P, Z_P, 48), cyl_y(6.0, y0, y1, X_P + 19.0, Z_P, 32)])
    flange = box(BASE_X[1] + 0.1, BASE_X[1] + 0.1 + FLANGE_T, y0, Y_FL1, -gz + 2, gz - 2)
    gus = [M.hull_points([[BASE_X[1] + 0.1, y1, z0], [BASE_X[1] + 0.1, Y_FL1, z0], [BASE_X[1] + 0.1 + FLANGE_T, Y_FL1, z0],
                          [BASE_X[1] + 10, y1, z0], [BASE_X[1] + 0.1, y1, z0 + 3], [BASE_X[1] + 10, y1, z0 + 3],
                          [BASE_X[1] + 0.1, Y_FL1, z0 + 3], [BASE_X[1] + 0.1 + FLANGE_T, Y_FL1, z0 + 3]])
           for z0 in (-1.5,)]
    e = union([plate, flange] + gus)
    cuts = [cyl_y(4.1, y0 - 1, y1 + 1, X_P, Z_P, 40)]                         # ступица качалки
    pk = M.batch_hull([cyl_y(4.5, y1 - HORN_POCKET, y1 + 1, X_P, Z_P, 40),
                       cyl_y(2.4, y1 - HORN_POCKET, y1 + 1, X_P + 19.0, Z_P, 24)])
    cuts.append(pk)
    for r in (9.0, 11.0, 13.0, 15.0, 17.0):
        cuts.append(cyl_y(0.7, y0 + 0.6, y1, X_P + r, Z_P, 12))
    for y, z in BOLTS:
        cuts.append(cyl_x(1.65, BASE_X[1] - 1, BASE_X[1] + FLANGE_T + 2, y, z))
    return e - union(cuts)


# ---------------------------------------------------------------- люлька
def make_cradle():
    """Координаты относительно вала (при 0°): X вперёд, Z вверх."""
    x0 = XR - FIT - WALL
    tab_ext = SV_TAB_L / 2 + 0.6
    shell = box(x0, X_FRONT, Y_FLOOR_OUT, Y_TAB_LOW + SV_TAB_T + 2.0, -ZW - WALL, ZW + WALL)
    inner = box(x0 + WALL, X_WALL_F, Y_SV_BOT, Y_TAB_LOW + 10, -ZW, ZW)
    # полки под ушки серво (снаружи торцов)
    ledges = [box(XC - tab_ext, x0 + 0.01, Y_FLOOR_OUT, Y_TAB_LOW, -ZW - WALL, ZW + WALL),
              box(X_FRONT - 0.01, XC + tab_ext, Y_FLOOR_OUT, Y_TAB_LOW, -ZW - WALL, ZW + WALL)]
    # бобышка оси (сторона -Y) + опора до стола печати (низ -Z)
    boss = M.batch_hull([cyl_y(BOSS_R, Y_BOSS, Y_FLOOR_OUT + 0.01, 0, 0, 48),
                         box(-BOSS_R, BOSS_R, Y_BOSS, Y_FLOOR_OUT + 0.01, -ZW - WALL, -BOSS_R + 1)])
    # передний блок за полкой серво (серво при вставке сверху его не задевает) + ушки камеры
    xb0, xb1 = X_LEDGE_F - 0.01, X_LEDGE_F + FRONT_BLK
    yb_lo = min(Y_FLOOR_OUT, -CAM_TAB_IN - TAB_CAM_T)
    front = box(xb0, xb1, yb_lo, CAM_TAB_IN + TAB_CAM_T, -ZW - WALL, ZW + WALL)
    tabs = []
    for s in (1, -1):
        ya, yb = (CAM_TAB_IN, CAM_TAB_IN + TAB_CAM_T) if s > 0 else (-CAM_TAB_IN - TAB_CAM_T, -CAM_TAB_IN)
        t = M.batch_hull([box(xb0, xb1, ya, yb, -ZW - WALL, ZW + WALL),
                          cyl_y(6.0, ya, yb, X_LEDGE_F + SLOT[1], 0, 40)])
        tabs.append(t)
    c = union([shell, front, boss] + ledges + tabs) - inner
    cuts = []
    # паз под ушки серво в стенках (ушки лежат на полках)
    cuts.append(box(XC - tab_ext, XC + tab_ext, Y_TAB_LOW, Y_TAB_LOW + SV_TAB_T + 10, -SV_W / 2 - FIT, SV_W / 2 + FIT))
    # пилоты саморезов ушек (паз ±0.4 под разброс 27.5…28.2)
    for sx in (-1, 1):
        xh = XC + sx * SV_SCREW / 2
        cuts.append(M.batch_hull([cyl_y(0.8, Y_TAB_LOW - 7, Y_TAB_LOW + 1, xh - 0.4, 0, 12),
                                  cyl_y(0.8, Y_TAB_LOW - 7, Y_TAB_LOW + 1, xh + 0.4, 0, 12)]))
    # ось M3: отверстие + паз гайки M3 нейлок (вставить ДО серво), открыт вниз (-Z)
    cuts.append(cyl_y(1.25, Y_BOSS - 1, Y_SV_BOT - 0.6, 0, 0, 24))
    ny0 = Y_BOSS + 0.8
    cuts.append(box(-2.95, 2.95, ny0, ny0 + 4.2, -ZW - WALL - 1, 3.4))
    # кабель серво: передний торец, внизу (у дна)
    cuts.append(box(X_WALL_F - 1, X_FRONT + 1, Y_SV_BOT - 0.5, Y_SV_BOT + 5.0, -3.0, 3.0))
    # вертикальный паз под втулку кабеля во всю высоту стенки — серво заходит сверху
    cuts.append(box(X_WALL_F - 0.5, X_WALL_F + 2.0, Y_SV_BOT - 0.5, Y_TAB_LOW + 20, -2.3, 2.3))
    # пазы M2 камеры
    for s in (1, -1):
        cuts.append(M.batch_hull([cyl_y(1.15, -CAM_TAB_IN - 4, CAM_TAB_IN + 4, X_LEDGE_F + SLOT[0], 0, 16),
                                  cyl_y(1.15, -CAM_TAB_IN - 4, CAM_TAB_IN + 4, X_LEDGE_F + SLOT[1], 0, 16)]))
    # фаски на входе гнезда
    cuts.append(M.batch_hull([box(x0 + WALL, X_WALL_F, Y_TAB_LOW + SV_TAB_T + 1.99, Y_TAB_LOW + SV_TAB_T + 2.01, -ZW, ZW),
                              box(x0 + WALL - 0.8, X_WALL_F + 0.8, Y_TAB_LOW + SV_TAB_T + 2.8, Y_TAB_LOW + SV_TAB_T + 2.9,
                                  -ZW - 0.8, ZW + 0.8)]))
    return c - union(cuts)


# ---------------------------------------------------------------- макеты
def servo_body():
    """MG90S в координатах люльки, с ушками, выступом, шлицем и втулкой кабеля."""
    body = box(XR, XF, Y_SV_BOT, Y_CASE_TOP, -SV_W / 2, SV_W / 2)
    tabs = box(XC - SV_TAB_L / 2, XC + SV_TAB_L / 2, Y_TAB_LOW, Y_TAB_LOW + SV_TAB_T, -SV_W / 2, SV_W / 2)
    bossb = cyl_y(5.8, Y_CASE_TOP, Y_BOSS_TOP, 0, 0, 40)
    spl = cyl_y(2.4, Y_BOSS_TOP, Y_SPL_TOP, 0, 0, 24)
    grom = box(XF, XF + 1.5, Y_SV_BOT + 0.5, Y_SV_BOT + 4.0, -1.8, 1.8)
    return union([body, tabs, bossb, spl, grom])


def horn():
    """Одноплечая качалка (неподвижна): ступица в ухе + плечо вперёд в кармане."""
    hub = cyl_y(3.6, Y_BOSS_TOP + 0.3, Y_EL_OUT - 0.3, 0, 0, 32)
    arm = M.batch_hull([cyl_y(4.1, Y_EL_OUT - HORN_POCKET + 0.2, Y_EL_OUT - HORN_POCKET + 1.8, 0, 0, 32),
                        cyl_y(2.0, Y_EL_OUT - HORN_POCKET + 0.2, Y_EL_OUT - HORN_POCKET + 1.8, 19.0, 0, 24)])
    return union([hub, arm])


def camera(L):
    body = box(-(L - CAM_PIVOT), CAM_PIVOT, -CAM_W / 2, CAM_W / 2, -CAM_W / 2, CAM_W / 2)
    lens = M.cylinder(8.0, 7.0, 7.0, 40).rotate([0, 90, 0]).translate([CAM_PIVOT, 0, 0])
    return union([body, lens])


def cam_pivot_x(L):
    return X_LEDGE_F + max(SLOT[0], FRONT_BLK + (L - CAM_PIVOT) + 1.0)


def frame_proxy():
    plates = [box(-60, PLATE_FRONT_X, -40, 40, G, G + PLATE_T), box(-60, PLATE_FRONT_X, -40, 40, -G - PLATE_T, -G)]
    so = [cyl_z(SO_D / 2, -G, G, 0, s * SO_S / 2, 32) for s in (-1, 1)]
    return union(plates + so)


def rotating(tilt, L=CAM_LENGTHS[0]):
    def pl(m):
        return m.rotate([0, tilt, 0]).translate([X_P, 0, Z_P])
    cam = camera(L).translate([cam_pivot_x(L), 0, 0])
    return pl(make_cradle()), pl(servo_body()), pl(cam)


# ---------------------------------------------------------------- проверка
def check():
    ok = True
    fr, el = make_frame(), make_ear()
    er = M.cube([0.001, 0.001, 0.001]).translate([-100, 0, 0])
    cr, sv = make_cradle(), servo_body()
    fixed = union([fr, el, er, frame_proxy()])

    def rep(name, cond, detail=""):
        nonlocal ok
        ok &= bool(cond)
        print(f"  [{'OK' if cond else 'ПРОБЛЕМА'}] {name} {detail}")

    print("Сборка:")
    v = (sv ^ cr).volume()
    rep("MG90S входит в люльку (корпус, ушки, кабель, зазор 0.4 на сторону)", v < 0.05, f"пересечение {v:.2f} мм³")
    # путь вставки сверху (+Y): каждая выпуклая часть серво отдельно (общая оболочка заполнила бы пустоты)
    pieces = [box(XR, XF, Y_SV_BOT, Y_CASE_TOP, -SV_W / 2, SV_W / 2),
              box(XC - SV_TAB_L / 2, XC + SV_TAB_L / 2, Y_TAB_LOW, Y_TAB_LOW + SV_TAB_T, -SV_W / 2, SV_W / 2),
              box(XF, XF + 1.5, Y_SV_BOT + 0.5, Y_SV_BOT + 4.0, -1.8, 1.8)]
    v = sum((M.batch_hull([pc, pc.translate([0, 30, 0])]) ^ cr).volume() for pc in pieces)
    rep("серво вставляется сверху без препятствий", v < 0.05, f"{v:.2f} мм³")
    for L in CAM_LENGTHS:
        v = (camera(L).translate([cam_pivot_x(L), 0, 0]) ^ cr).volume()
        rep(f"камера {L:.0f} мм встаёт в паз ушек", v < 0.05 and cam_pivot_x(L) <= X_LEDGE_F + SLOT[1],
            f"ось камеры {cam_pivot_x(L) - X_LEDGE_F:.1f} мм от полки (паз {SLOT[0]:.0f}…{SLOT[1]:.0f})")
    hn = horn().translate([X_P, 0, Z_P])
    v = (hn ^ el).volume()
    rep("качалка ложится в карман левого уха", v < 0.05, f"{v:.2f} мм³")
    v = (el ^ fr).volume()
    rep("левое ухо садится на раму без натяга", v < 0.05, f"{v:.2f} мм³")
    v = (fr ^ frame_proxy()).volume()
    rep("рама не врезается в плиты и стойки дрона", v < 0.05, f"{v:.2f} мм³")
    # доступ отвёрткой
    tools = {"винт оси M3 (снаружи правого уха)": cyl_y(3.5, Y_ER_OUT - 40, Y_ER_OUT - 0.01, X_P, Z_P),
             "центральный винт качалки (снаружи левого уха)": cyl_y(3.0, Y_EL_OUT + 0.01, Y_EL_OUT + 40, X_P, Z_P)}
    for y, z in BOLTS:
        tools[f"винт фланца левого уха (Z={z:+.0f}) спереди"] = cyl_x(3.0, BASE_X[1] + FLANGE_T + 0.2, 70, y, z)
    for n, t in tools.items():
        v = (t ^ union([fr, el, er, frame_proxy()])).volume()
        rep(f"доступ отвёрткой: {n}", v < 0.05, f"{v:.2f} мм³")
    print("Ход 0…90° (камеры 20 и 28 мм):")
    worst, at = 0.0, None
    for L in CAM_LENGTHS:
        for t in range(0, 91, 3):
            for m in rotating(t, L):
                v = (m ^ fixed).volume()
                if v > worst:
                    worst, at = v, (L, t)
    rep("ничего не задевает", worst < 0.5, f"{worst:.2f} мм³ {at or ''}")
    back = min(fr.bounding_box()[0], el.bounding_box()[0])
    rep("позади стоек пусто (плата ESP)", back >= -(SO_D / 2 + 2.5) - 0.01, f"X min = {back:.1f}")
    print(f"Ось X={X_P:.1f}, Z={Z_P:.1f}; уши Y {Y_ER_OUT:.1f}…{Y_EL_OUT:.1f}; ширина {Y_FL1 - Y_ER_OUT:.1f} мм")
    return ok


# ---------------------------------------------------------------- экспорт и картинки
PARTS = {
    "tilt_v2_frame_x1": (lambda: make_frame(), [90, 0, 0]),                # лёжа на правом ухе
    "tilt_v2_ear_left_horn_x1": (lambda: make_ear(), [90, 0, 0]),         # внутренней стороной вниз, фланец и карман вверх
    "tilt_v2_cradle_x1": (lambda: make_cradle(), None),
}


def export():
    d = os.path.join(HERE, "stl", "tilt_v2")
    os.makedirs(d, exist_ok=True)
    for name, (f, rot) in PARTS.items():
        m = f()
        if rot:
            m = m.rotate(rot)
        to_trimesh(on_bed(m)).export(os.path.join(d, name + ".stl"))
    render(os.path.join(HERE, "tilt_v2.png"))


def render(path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    light = np.array([0.4, 0.6, 0.75]); light /= np.linalg.norm(light)
    fig = plt.figure(figsize=(21, 7.5))
    views = [(0, "камера вперёд (0°)", (22, 40)), (90, "камера в пол (90°)", (22, 40)),
             (0, "разнесённая сборка", (25, 55))]
    for k, (t, title, (el_, az)) in enumerate(views):
        cr, sv, cam = rotating(t)
        hn = horn().translate([X_P, 0, Z_P])
        items = [(frame_proxy(), "carbon"), (make_frame(), "frame"), (make_ear(), "ear"),
                 (hn, "metal"), (cr, "print"), (sv, "servo"), (cam, "cam")]
        if k == 2:   # разнести: левое ухо с качалкой в сторону, серво из люльки вверх
            items = [(make_frame(), "frame"), (make_ear().translate([0, 22, 0]), "ear"),
                     (hn.translate([0, 34, 0]), "metal"), (cr, "print"), (sv.translate([0, 26, 0]), "servo")]
        T, C = [], []
        for man, col in items:
            tm = to_trimesh(man)
            sh = 0.35 + 0.65 * np.clip(tm.face_normals @ light, 0, 1)
            T.append(tm.vertices[tm.faces]); C.append(np.c_[np.outer(sh, COL[col]), np.ones_like(sh)])
        T = np.concatenate(T); C = np.concatenate(C)
        keep = T[:, :, 0].mean(1) > -14
        T, C = T[keep], C[keep]
        ax = fig.add_subplot(1, 3, k + 1, projection="3d")
        ax.add_collection3d(Poly3DCollection(T, facecolors=C, edgecolor="none"))
        V = T.reshape(-1, 3); lo, hi = V.min(0), V.max(0); c = (lo + hi) / 2; r = (hi - lo).max() / 2 * 0.75
        ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(c[2] - r, c[2] + r)
        ax.set_box_aspect((1, 1, 1)); ax.view_init(el_, az); ax.set_axis_off(); ax.set_title(title, fontsize=14)
    plt.tight_layout(); plt.savefig(path, dpi=80)


if __name__ == "__main__":
    if "--check" in sys.argv:
        sys.exit(0 if check() else 1)
    export()
