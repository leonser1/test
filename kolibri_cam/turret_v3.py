"""
Поворот камеры v3 — «турель» MG90S 180°: корпус серво НЕПОДВИЖЕН, на качалке сидит
U-скоба с камерой, второе ухо скобы вращается на оси M3 в правом ухе рамы.
Крепление к раме дрона спереди: 2 трубки на передние стойки (33 мм между центрами).

Детали:
  рама (трубки + вертикальная рама + правое ухо-опора)  — печать стоя, трубки вертикально
  корпус серво с фланцем на раму (2×M3, пазы ±2 мм)      — печать фланцем шлица вниз
  U-скоба под камеру 19 мм, паз M2 под камеры 20…30 мм   — печать перемычкой вниз

Координаты: X вперёд, Y влево, Z вверх; стойки X=0, Y=±SO_S/2; Z=0 — середина между плитами.
Запуск: python3 turret_v3.py [--check]
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "kolibri_car"))
from generate_car import M, m3d, box, cyl_y, cyl_z, union, hex_prism, to_trimesh, on_bed  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))

# ---------------- рама дрона ----------------
SO_S, SO_D = 33.0, 5.0
TUBE_RO = SO_D / 2 + 3.15         # стенка трубки 3.0 мм
PLATE_GAP, PLATE_T, PLATE_FRONT_X = 35.0, 2.0, 6.0
# ---------------- MG90S (замер: низ → ушки 16 мм) ----------------
SV_L, SV_W, SV_H = 22.8, 12.2, 22.5
SV_BOSS_H, SV_BOSS_R, SV_SPL_H = 4.0, 5.8, 3.5
SV_OFF = 5.5
SV_TAB_L, SV_TAB_T, SV_TAB_Z = 32.5, 2.5, 16.0
SV_SCREW = 27.8
HORN_R = 12.0                    # плечо качалки (обрезать по 14 мм от центра), назад
FIT = 0.4
# ---------------- камера ----------------
CAM_W, CAM_PIVOT = 19.0, 8.0
CAM_LENGTHS = (20.0, 28.0)
# ---------------- механизм ----------------
X_P, Z_P = 23.5, 0.0
BASE_X = (1.5, 6.5)                 # 5 мм: дно гнезда гайки 2.4
ARM_T = 5.0                      # уши скобы (после кармана качалки 3.2 мм)
CAM_IN = CAM_W / 2 + 0.25        # 19.5 между ушами скобы
HORN_POCKET = 1.8                # карман под плечо качалки на наружной стороне левого уха скобы
HUB_L = 3.0                      # ступица качалки (со стороны серво)
BACK = (-7.0, -4.0)              # перемычка скобы позади оси
SLOT = (9.0, 19.0)               # паз M2 камеры от оси: камеры 20…30 мм
YOKE_FRONT = SLOT[1] + 4.0
BOSS_R, BOSS_L = 6.0, 6.0        # бобышка оси на правом ухе скобы
GAP = 0.5
EAR_T = 5.0                      # правое ухо рамы
FL_T = 4.0                       # фланец корпуса серво к раме
WALL = 2.4
# --------------------------------------------------------

G = PLATE_GAP / 2
Y_AL0, Y_AL1 = CAM_IN, CAM_IN + ARM_T                 # левое ухо скобы
Y_AR0, Y_AR1 = -CAM_IN - ARM_T, -CAM_IN               # правое ухо скобы
Y_SPL_TOP = Y_AL1 + 0.5                               # шлиц почти у наружной грани уха (ступица качалки снаружи)
Y_BOSS_TOP = Y_SPL_TOP + SV_SPL_H
Y_CASE_TOP = Y_BOSS_TOP + SV_BOSS_H
Y_SV_BOT = Y_CASE_TOP + SV_H
Y_TAB_IN = Y_SV_BOT - SV_TAB_Z - SV_TAB_T             # грань ушка серво к шлицу
Y_FLANGE0 = Y_CASE_TOP - 2.0                          # плита корпуса со стороны шлица (2 мм)
Y_ER1 = Y_AR0 - BOSS_L - GAP                         # внутренняя грань правого уха рамы
Y_ER0 = Y_ER1 - EAR_T
XR, XF, XC = -(SV_L / 2 - SV_OFF), SV_L / 2 + SV_OFF, SV_OFF
ZW = SV_W / 2 + FIT
HX0, HX1 = XR - FIT - WALL, XF + FIT + WALL            # корпус серво по X (от вала)
Y_FL1 = Y_TAB_IN + 10.5                               # фланец до сюда (Y)
BOLTS = [(Y_TAB_IN + 4.5, 9.0), (Y_TAB_IN + 4.5, -9.0)]

COL = {"frame": (0.25, 0.42, 0.75), "house": (0.20, 0.55, 0.80), "print": (0.93, 0.55, 0.18),
       "cam": (0.16, 0.16, 0.18), "servo": (0.30, 0.31, 0.34), "metal": (0.75, 0.75, 0.78),
       "carbon": (0.12, 0.12, 0.13)}


def cyl_x(r, x0, x1, y, z, seg=24):
    return M.cylinder(x1 - x0, r, r, seg).rotate([0, 90, 0]).translate([x0, y, z])


# ---------------------------------------------------------------- рама
def make_frame():
    ro, gz = TUBE_RO, G - 0.2
    parts = [cyl_z(ro, -gz, gz, 0, s * SO_S / 2, 48) for s in (-1, 1)]
    parts.append(box(*BASE_X, Y_ER0, Y_FL1, -gz, gz))
    parts.append(M.batch_hull([box(BASE_X[0], BASE_X[1] + 2, Y_ER0, Y_ER1, -gz, gz), cyl_y(9.5, Y_ER0, Y_ER1, X_P, Z_P, 48)]))
    f = union(parts)
    cuts = [cyl_z(SO_D / 2 + 0.15, -G - 1, G + 1, 0, s * SO_S / 2, 48) for s in (-1, 1)]
    cuts.append(M.batch_hull([cyl_x(2.0, BASE_X[0] - 1, BASE_X[1] + 1, y, z)          # окно (облегчение), R2
                              for y in (-4.0, 4.0) for z in (-7.0, 7.0)]))
    cuts.append(cyl_y(1.7, Y_ER0 - 1, Y_ER1 + 1, X_P, Z_P, 24))                       # ось M3
    for y, z in BOLTS:
        cuts.append(cyl_x(1.65, BASE_X[0] - 1, BASE_X[1] + 1, y, z))
        cuts.append(hex_prism(5.8, 0, 2.6).rotate([0, 90, 0]).translate([BASE_X[0] - 0.01, y, z]))
    return f - union(cuts)


# ---------------------------------------------------------------- корпус серво
def make_housing():
    """Корпус MG90S: плита со стороны шлица + стенки до ушек; фланец к раме (пазы ±2 по Y)."""
    gz = G - 0.2
    x0, x1 = X_P + HX0, X_P + HX1
    shell = box(x0, x1, Y_FLANGE0, Y_TAB_IN, Z_P - ZW - WALL, Z_P + ZW + WALL)
    ledges = [box(X_P + XC - SV_TAB_L / 2 - 0.6, x0 + 0.01, Y_FLANGE0, Y_TAB_IN, Z_P - ZW - WALL, Z_P + ZW + WALL),
              box(x1 - 0.01, X_P + XC + SV_TAB_L / 2 + 0.6, Y_FLANGE0, Y_TAB_IN, Z_P - ZW - WALL, Z_P + ZW + WALL)]
    fx0 = BASE_X[1] + 0.1
    flange = box(fx0, fx0 + FL_T, Y_FLANGE0, Y_FL1, -gz + 2, gz - 2)
    web = box(fx0, x0 + 0.01, Y_FLANGE0, Y_TAB_IN, Z_P - ZW - WALL, Z_P + ZW + WALL)
    ribs = [M.batch_hull([box(fx0, x0, Y_FLANGE0, Y_TAB_IN, z0, z1), box(fx0, fx0 + FL_T + 0.5, Y_FL1 - 0.1, Y_FL1, z0, z1)])
            for z0, z1 in ((12.3, gz - 2), (-gz + 2, -12.3))]                # косынки фланца
    h = union([shell, flange, web] + ledges + ribs)
    cuts = [box(X_P + XR - FIT, X_P + XF + FIT, Y_CASE_TOP, Y_TAB_IN + 1, Z_P - ZW, Z_P + ZW),     # гнездо
            cyl_y(SV_BOSS_R + 0.7, Y_FLANGE0 - 1, Y_CASE_TOP + 0.5, X_P, Z_P, 48),               # выступ+шлиц
            box(X_P + XF - 0.5, X_P + XF + FIT + 2.6, Y_CASE_TOP + 1.0, Y_TAB_IN + 1, Z_P - 2.3, Z_P + 2.3)]  # кабель
    for sx in (-1, 1):
        xh = X_P + XC + sx * SV_SCREW / 2
        cuts.append(M.batch_hull([cyl_y(0.8, Y_TAB_IN - 7, Y_TAB_IN + 1, xh - 0.4, Z_P, 12),
                                  cyl_y(0.8, Y_TAB_IN - 7, Y_TAB_IN + 1, xh + 0.4, Z_P, 12)]))
    for y, z in BOLTS:     # пазы ±2 мм по Y — подгонка посадки скобы на шлиц
        cuts.append(M.batch_hull([cyl_x(1.65, fx0 - 1, fx0 + FL_T + 1, y - 2, z), cyl_x(1.65, fx0 - 1, fx0 + FL_T + 1, y + 2, z)]))
    return h - union(cuts)


# ---------------------------------------------------------------- U-скоба
def make_yoke():
    """Координаты относительно оси (при 0°)."""
    zh = 7.0
    arms = []
    for y0, y1 in ((Y_AL0, Y_AL1), (Y_AR0, Y_AR1)):
        arms.append(M.batch_hull([box(BACK[0], BACK[1], y0, y1, -zh, zh), cyl_y(7.0, y0, y1, 0, 0, 48),
                                  cyl_y(5.5, y0, y1, YOKE_FRONT - 5.5, 0, 40), cyl_y(5.0, y0, y1, -HORN_R, 0, 40),
                                  box(-HORN_R, YOKE_FRONT - 5.5, y0, y1, -zh, -zh + 1)]))       # плоский низ на стол
    back = box(BACK[0], BACK[1], Y_AR0, Y_AL1, -zh, zh)
    boss = M.batch_hull([cyl_y(BOSS_R, Y_AR0 - BOSS_L, Y_AR0 + 0.01, 0, 0, 40),
                         box(-BOSS_R, BOSS_R, Y_AR0 - BOSS_L, Y_AR0 + 0.01, -zh, -zh + 1)])
    y = union(arms + [back, boss])
    cuts = []
    # карман качалки на наружной стороне левого уха: ступица Ø9 + плечо назад
    cuts.append(M.batch_hull([cyl_y(4.5, Y_AL1 - HORN_POCKET, Y_AL1 + 1, 0, 0, 40),
                              cyl_y(2.4, Y_AL1 - HORN_POCKET, Y_AL1 + 1, -HORN_R, 0, 24)]))
    cuts.append(cyl_y(1.6, Y_AL0 - 1, Y_AL1 + 1, 0, 0, 24))                     # центральный винт качалки
    for r in (7.0, 9.0, 11.0):
        cuts.append(cyl_y(0.7, Y_AL0 + 0.6, Y_AL1, -r, 0, 12))                   # пилоты саморезов качалки
    # ось M3 справа: отверстие + паз гайки M3 с нейлоном
    cuts.append(cyl_y(1.25, Y_AR0 - BOSS_L - 1, Y_AR1 + 1, 0, 0, 24))
    cuts.append(box(-2.95, 2.95, Y_AR0 - BOSS_L + 1.6, Y_AR0 - BOSS_L + 5.8, -zh - 1, 3.4))   # гайка M3 с нейлоном (4 мм), вставляется снизу
    # пазы M2 камеры
    for yy in (Y_AL0, Y_AR0):
        cuts.append(M.batch_hull([cyl_y(1.15, Y_AR0 - BOSS_L - 1, Y_AL1 + 1, SLOT[0], 0, 16),
                                  cyl_y(1.15, Y_AR0 - BOSS_L - 1, Y_AL1 + 1, SLOT[1], 0, 16)]))
    return y - union(cuts)


def horn():
    """Одноплечая качалка в кармане скобы (плечо назад-вниз), ступица к серво."""
    arm = M.batch_hull([cyl_y(4.1, Y_AL1 - HORN_POCKET + 0.1, Y_AL1 - 0.1, 0, 0, 32),
                        cyl_y(2.0, Y_AL1 - HORN_POCKET + 0.1, Y_AL1 - 0.1, -HORN_R, 0, 24)])
    hub = cyl_y(3.6, Y_AL1 - 0.1, Y_AL1 + HUB_L, 0, 0, 32)
    return union([arm, hub])


def servo_body():
    """MG90S в координатах рамы (неподвижен): вал на оси, шлиц к скобе (-Y), низ корпуса наружу."""
    body = box(XR, XF, Y_CASE_TOP, Y_SV_BOT, -SV_W / 2, SV_W / 2)
    tabs = box(XC - SV_TAB_L / 2, XC + SV_TAB_L / 2, Y_TAB_IN, Y_TAB_IN + SV_TAB_T, -SV_W / 2, SV_W / 2)
    bossb = cyl_y(SV_BOSS_R, Y_BOSS_TOP, Y_CASE_TOP, 0, 0, 40)
    spl = cyl_y(2.4, Y_SPL_TOP, Y_BOSS_TOP, 0, 0, 24)
    grom = box(XF, XF + 1.5, Y_SV_BOT - 4.0, Y_SV_BOT - 0.5, -1.8, 1.8)
    return union([body, tabs, bossb, spl, grom]).translate([X_P, 0, Z_P])


def camera(L):
    body = box(-(L - CAM_PIVOT), CAM_PIVOT, -CAM_W / 2, CAM_W / 2, -CAM_W / 2, CAM_W / 2)
    lens = M.cylinder(8.0, 7.0, 7.0, 40).rotate([0, 90, 0]).translate([CAM_PIVOT, 0, 0])
    return union([body, lens])


def cam_pivot(L):
    return max(SLOT[0], BACK[1] + (L - CAM_PIVOT) + 1.0)


def frame_proxy():
    plates = [box(-60, PLATE_FRONT_X, -45, 45, G, G + PLATE_T), box(-60, PLATE_FRONT_X, -45, 45, -G - PLATE_T, -G)]
    return union(plates + [cyl_z(SO_D / 2, -G, G, 0, s * SO_S / 2, 32) for s in (-1, 1)])


def rotating(t, L=CAM_LENGTHS[0]):
    def pl(m):
        return m.rotate([0, t, 0]).translate([X_P, 0, Z_P])
    return pl(make_yoke()), pl(horn()), pl(camera(L).translate([cam_pivot(L), 0, 0]))


# ---------------------------------------------------------------- проверка
def check():
    ok = True
    fr, hs, yk, sv = make_frame(), make_housing(), make_yoke(), servo_body()

    def rep(name, cond, detail=""):
        nonlocal ok
        ok &= bool(cond)
        print(f"  [{'OK' if cond else 'ПРОБЛЕМА'}] {name} {detail}")

    print("Сборка:")
    rep("MG90S входит в корпус", (sv ^ hs).volume() < 0.05, f"{(sv ^ hs).volume():.2f} мм³")
    pieces = [box(XR, XF, Y_CASE_TOP, Y_SV_BOT, -SV_W / 2, SV_W / 2),
              box(XC - SV_TAB_L / 2, XC + SV_TAB_L / 2, Y_TAB_IN, Y_TAB_IN + SV_TAB_T, -SV_W / 2, SV_W / 2),
              box(XF, XF + 1.5, Y_SV_BOT - 4.0, Y_SV_BOT - 0.5, -1.8, 1.8),
              cyl_y(SV_BOSS_R, Y_BOSS_TOP, Y_CASE_TOP, 0, 0, 40)]
    v = sum((M.batch_hull([p_.translate([X_P, 0, Z_P]), p_.translate([X_P, 40, Z_P])]) ^ hs).volume() for p_ in pieces)
    rep("серво вставляется снаружи (+Y) без помех", v < 0.05, f"{v:.2f} мм³")
    rep("корпус садится на раму", (hs ^ fr).volume() < 0.05)
    rep("рама не врезается в плиты/стойки", (fr ^ frame_proxy()).volume() < 0.05)
    hn = horn()
    rep("качалка ложится в карман скобы", (hn ^ yk).volume() < 0.05, f"{(hn ^ yk).volume():.2f} мм³")
    for L in CAM_LENGTHS:
        v = (camera(L).translate([cam_pivot(L), 0, 0]) ^ yk).volume()
        rep(f"камера {L:.0f} мм в пазу скобы", v < 0.05 and cam_pivot(L) <= SLOT[1], f"ось камеры {cam_pivot(L):.1f} от оси")
    tools = {"винт оси M3 снаружи правого уха": cyl_y(3.5, Y_ER0 - 40, Y_ER0 - 0.01, X_P, Z_P)}
    for y, z in BOLTS:
        tools[f"винт фланца корпуса (Z={z:+.0f}) спереди"] = cyl_x(3.0, BASE_X[1] + FL_T + 0.2, 80, y, z)
    tools["саморезы ушек серво снаружи"] = union([cyl_y(2.5, Y_TAB_IN + SV_TAB_T + 0.01, Y_TAB_IN + 40,
                                                          X_P + XC + s_ * SV_SCREW / 2, Z_P) for s_ in (-1, 1)])
    for n, t_ in tools.items():
        v = (t_ ^ union([fr, hs, frame_proxy()])).volume()
        rep(f"доступ отвёрткой: {n}", v < 0.05, f"{v:.2f} мм³")
    print("Ход 0…90° (камеры 20 и 28 мм):")
    fixed = union([fr, hs, sv, frame_proxy()])
    worst, at = 0.0, None
    for L in CAM_LENGTHS:
        for t in range(0, 91, 3):
            yk_, hn_, cm = rotating(t, L)
            v = (yk_ ^ fixed).volume() + (cm ^ fixed).volume() + ((hn_ ^ fixed).volume() - (hn_ ^ sv).volume())
            if v > worst:
                worst, at = v, (L, t)
    rep("ничего не задевает", worst < 0.5, f"{worst:.2f} мм³ {at or ''}")
    back = min(fr.bounding_box()[0], hs.bounding_box()[0])
    rep("позади стоек пусто", back >= -TUBE_RO - 0.01, f"X min = {back:.1f}")
    print(f"Ось X={X_P}, Z={Z_P}; серво Y {Y_CASE_TOP:.1f}…{Y_SV_BOT:.1f}; ширина {Y_SV_BOT - Y_ER0:.1f} мм")
    return ok


# ---------------------------------------------------------------- экспорт и картинка
PARTS = {
    "turret_v3_frame_x1": (make_frame, [0, 0, 0]),             # стоя, трубки вертикально
    "turret_v3_servo_housing_x1": (make_housing, [90, 0, 0]),   # плитой шлица вниз, гнездо открыто вверх
    "turret_v3_yoke_x1": (make_yoke, [0, 0, 0]),              # плоским низом ушей на стол, слои вдоль ушей
}


def export():
    d = os.path.join(HERE, "stl", "turret_v3")
    os.makedirs(d, exist_ok=True)
    for name, (f, rot) in PARTS.items():
        to_trimesh(on_bed(f().rotate(rot))).export(os.path.join(d, name + ".stl"))
    render(os.path.join(HERE, "turret_v3.png"))


def render(path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    light = np.array([0.4, 0.6, 0.75]); light /= np.linalg.norm(light)
    fig = plt.figure(figsize=(21, 7.5))
    for k, (t, title) in enumerate([(0, "камера вперёд (0°)"), (45, "45°"), (90, "камера в пол (90°)")]):
        yk, hn, cm = rotating(t)
        items = [(frame_proxy(), "carbon"), (make_frame(), "frame"), (make_housing(), "house"), (servo_body(), "servo"),
                 (yk, "print"), (hn, "metal"), (cm, "cam")]
        T, C = [], []
        for man, col in items:
            tm = to_trimesh(man)
            sh = 0.35 + 0.65 * np.clip(tm.face_normals @ light, 0, 1)
            T.append(tm.vertices[tm.faces]); C.append(np.c_[np.outer(sh, COL[col]), np.ones_like(sh)])
        T = np.concatenate(T); C = np.concatenate(C)
        keep = T[:, :, 0].mean(1) > -12
        T, C = T[keep], C[keep]
        ax = fig.add_subplot(1, 3, k + 1, projection="3d")
        ax.add_collection3d(Poly3DCollection(T, facecolors=C, edgecolor="none"))
        V = T.reshape(-1, 3); lo, hi = V.min(0), V.max(0); c = (lo + hi) / 2; r = (hi - lo).max() / 2 * 0.72
        ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(c[2] - r, c[2] + r)
        ax.set_box_aspect((1, 1, 1)); ax.view_init(22, 40); ax.set_axis_off(); ax.set_title(title, fontsize=14)
    plt.tight_layout(); plt.savefig(path, dpi=80)


if __name__ == "__main__":
    if "--check" in sys.argv:
        sys.exit(0 if check() else 1)
    export()
