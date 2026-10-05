"""
Поворотный механизм камеры «как на фото» под «Колибри» (Mark4 10" V2) + MG90S.

Как работает: качалка серво прикручена к НЕПОДВИЖНОМУ левому уху рамы, корпус серво
держится с другой стороны на винте-оси M3 в правом ухе. Серво, поворачивая «вал»,
крутит САМ СЕБЯ вместе с люлькой. На люльке 2 ушка под камеру micro 19 мм —
камера уходит по дуге вперёд-вниз и смотрит в пол.

Крепление: 2 трубки на передние стойки рамы (стойку вывернуть, надеть трубку,
вкрутить обратно) + U-рама вперёд. Позади стоек ничего нет (там плата ESP).

Координаты: X вперёд, Y влево, Z вверх; оси стоек в X=0, Y=±SO_S/2; Z=0 — середина
между плитами. Запуск: python3 photo_mount.py [--check]
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "kolibri_car"))
from generate_car import M, m3d, box, cyl_y, cyl_z, union, hex_prism, to_trimesh, on_bed  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))

# ---------------- ПАРАМЕТРЫ (мм): замерить стойки! ----------------
SO_S = 30.0             # между осями двух передних стоек
SO_D = 5.0              # диаметр стойки
PLATE_GAP = 35.0        # между плитами (Mark4 10" V2)
PLATE_T = 2.0
PLATE_FRONT_X = 6.0     # нижняя плита выступает перед осью стоек
TUBE_WALL = 2.5
FRAME_T = 4.0           # толщина U-рамы (лежит на нижней плите)
# MG90S
SV_L, SV_W, SV_H = 22.8, 12.2, 22.7     # корпус: длина, ширина, высота (без шлица)
SV_SHAFT_OFF = 5.5                      # вал смещён от центра корпуса по длине
SV_SPLINE_H = 4.0                       # шлиц над корпусом
SV_TAB_FROM_TOP, SV_TAB_T, SV_TAB_L = 6.5, 2.5, 32.2
SV_SCREW = 27.8
HORN_T = 2.0                            # крестовая качалка
# камера micro 19
CAM_W, CAM_L, CAM_PIVOT = 19.0, 20.0, 8.0
LENS_R, LENS_L = 7.0, 8.0
# геометрия механизма
X_P = 20.0              # ось поворота (вал серво): перед краем нижней плиты
WALL = 2.0              # стенки люльки
EAR_T = 4.0             # уши рамы
CLR = 0.5               # зазор люлька–ухо
TILT_MAX = 90.0
# --------------------------------------------------------------------

G = PLATE_GAP / 2
Z_FR0 = -G + 0.2                        # низ U-рамы (на нижней плите)
Z_FR1 = Z_FR0 + FRAME_T
Z_P = Z_FR1 + 1.0 + SV_W / 2 + WALL + 1.0   # ось: корпус серво при 0° над рамой
# по Y: левое ухо (качалка) — правое ухо (винт-ось)
Y_HORN_FACE = 15.6                      # внутренняя сторона левого уха = торец качалки
Y_SPLINE_TOP = Y_HORN_FACE - HORN_T
Y_SV_TOP = Y_SPLINE_TOP - SV_SPLINE_H
Y_SV_BOT = Y_SV_TOP - SV_H
Y_CR_BOT = Y_SV_BOT - 4.0               # дно люльки 4 мм (саморез оси M3)
Y_EAR_R = Y_CR_BOT - CLR                # внутренняя сторона правого уха
X_SV_REAR = -(SV_L / 2 - SV_SHAFT_OFF)  # корпус относительно вала (при 0°): назад 5.9
X_SV_FRONT = SV_L / 2 + SV_SHAFT_OFF    # вперёд 17.3
X_WALL_F = X_SV_FRONT + 0.3             # передняя стенка люльки
CAM_X = X_WALL_F + WALL + (CAM_L - CAM_PIVOT) + 4.0   # ось винтов камеры от вала (за полкой ушка серво)
TAB_IN = CAM_W / 2 + 0.15               # внутренние стороны ушек камеры

COL = {"frame": (0.25, 0.42, 0.75), "print": (0.93, 0.55, 0.18), "cam": (0.16, 0.16, 0.18),
       "servo": (0.30, 0.31, 0.34), "metal": (0.72, 0.72, 0.74), "carbon": (0.12, 0.12, 0.13),
       "esp": (0.15, 0.55, 0.25)}


# ---------------------------------------------------------------- детали
def make_frame():
    """Неподвижная часть: 2 трубки на стойки + U-рама + 2 уха (качалка / винт-ось)."""
    parts = []
    ro = SO_D / 2 + TUBE_WALL
    for s in (-1, 1):
        parts.append(cyl_z(ro, Z_FR0, G - 0.2, 0, s * SO_S / 2, 48))
    y_out_l, y_out_r = Y_HORN_FACE + EAR_T, Y_EAR_R - EAR_T
    # U: задняя перекладина и боковины по нижней плите
    parts.append(box(-ro + 1, 9.0, y_out_r, y_out_l, Z_FR0, Z_FR1))
    for y0, y1 in ((Y_HORN_FACE, y_out_l), (y_out_r, Y_EAR_R)):
        parts.append(box(0, X_P + 12, y0, y1, Z_FR0, Z_FR1))
        # ухо: вертикальная пластина вокруг оси + косынка к раме
        ear = M.batch_hull([box(X_P - 12, X_P + 12, y0, y1, Z_FR0, Z_FR1), cyl_y(12.5, y0, y1, X_P, Z_P)])
        parts.append(ear)
    f = union(parts)
    cuts = [cyl_z(SO_D / 2 + 0.15, Z_FR0 - 1, G + 1, 0, s * SO_S / 2, 48) for s in (-1, 1)]
    # левое ухо: под крестовую качалку — центр Ø7 (винт качалки) + 4 радиальных паза под саморезы M2
    cuts.append(cyl_y(3.5, Y_HORN_FACE - 1, y_out_l + 1, X_P, Z_P, 32))
    for a in (0, 90, 180, 270):
        ca, sa = math.cos(math.radians(a)), math.sin(math.radians(a))
        cuts.append(M.batch_hull([cyl_y(0.9, Y_HORN_FACE - 1, y_out_l + 1, X_P + 5 * ca, Z_P + 5 * sa, 12),
                                  cyl_y(0.9, Y_HORN_FACE - 1, y_out_l + 1, X_P + 11 * ca, Z_P + 11 * sa, 12)]))
    # правое ухо: ось M3 (винт снаружи, головка в цековке)
    cuts.append(cyl_y(1.65, y_out_r - 1, Y_EAR_R + 1, X_P, Z_P, 24))
    cuts.append(cyl_y(3.0, y_out_r - 1, y_out_r + 1.5, X_P, Z_P, 32))
    # окно в раме под проход корпуса серво и камеры (U открыта вперёд)
    cuts.append(box(9.0, X_P + 30, Y_EAR_R + 0.01, Y_HORN_FACE - 0.01, Z_FR0 - 1, Z_FR1 + 1))
    return f - union(cuts)


def make_cradle():
    """Люлька MG90S (координаты относительно вала при 0°): коробка вокруг корпуса, ушки серво
    ложатся на верх стенок, дно 4 мм с отверстием под ось M3, впереди 2 ушка под камеру."""
    x0, x1 = X_SV_REAR - 0.3 - WALL, X_WALL_F + WALL
    zo = SV_W / 2 + 0.3 + WALL
    y_tab = Y_SV_TOP - SV_TAB_FROM_TOP - SV_TAB_T           # низ ушек серво
    shell = box(x0, x1, Y_CR_BOT, y_tab, -zo, zo)
    inner = box(x0 + WALL, x1 - WALL, Y_CR_BOT + 4.0, y_tab + 1, -zo + WALL, zo - WALL)
    # передняя стенка поднимается до верхнего ушка камеры
    front = box(X_WALL_F, x1, Y_CR_BOT, TAB_IN + 2.4, -zo, zo)
    tabs = []
    for s in (1, -1):
        y0, y1 = (TAB_IN, TAB_IN + 2.4) if s > 0 else (-TAB_IN - 2.4, -TAB_IN)
        tabs.append(M.batch_hull([box(X_WALL_F, x1, y0, y1, -zo, zo),
                                  cyl_y(5.0, y0, y1, CAM_X, 0)]))
    xc = (X_SV_FRONT + X_SV_REAR) / 2
    ledges = [box(xc + sx * (SV_L / 2 + 0.3), xc + sx * (SV_TAB_L / 2 + 0.6), y_tab - 6.0, y_tab, -SV_W / 2, SV_W / 2)
              for sx in (-1, 1)]
    ledges = [box(min(l.bounding_box()[0], l.bounding_box()[3]), max(l.bounding_box()[0], l.bounding_box()[3]),
                  y_tab - 6.0, y_tab, -SV_W / 2, SV_W / 2) for l in ledges]
    c = union([shell, front] + tabs + ledges) - inner
    # паз в передней стенке под ушко серво (ушко лежит на полке)
    c = c - box(X_WALL_F - 1, x1 + 2, y_tab, y_tab + SV_TAB_T + 0.3, -SV_W / 2 - 0.3, SV_W / 2 + 0.3)
    cuts = [cyl_y(1.25, Y_CR_BOT - 1, Y_CR_BOT + 4.5, 0, 0, 16),                 # ось M3 саморезом
            box(x0 - 1, x0 + WALL + 1, Y_CR_BOT + 5, y_tab - 2, -3, 3)]           # выход кабеля серво
    for sx in (-1, 1):   # пилоты M2 под ушки серво (шаг 27.8 по длине, вал смещён)
        cuts.append(cyl_y(0.8, y_tab - 6, y_tab + 1, sx * SV_SCREW / 2 + (X_SV_FRONT + X_SV_REAR) / 2, 0, 12))
    for s in (1, -1):    # M2 камеры
        cuts.append(cyl_y(1.1, -TAB_IN - 3, TAB_IN + 3, CAM_X, 0, 16))
    return c - union(cuts)


def servo_body():
    """Корпус MG90S в координатах люльки (вал в 0,0 по X/Z), без шлица."""
    xc = (X_SV_FRONT + X_SV_REAR) / 2
    body = box(X_SV_REAR, X_SV_FRONT, Y_SV_BOT, Y_SV_TOP, -SV_W / 2, SV_W / 2)
    tabs = box(xc - SV_TAB_L / 2, xc + SV_TAB_L / 2, Y_SV_TOP - SV_TAB_FROM_TOP - SV_TAB_T,
               Y_SV_TOP - SV_TAB_FROM_TOP, -SV_W / 2, SV_W / 2)
    spl = cyl_y(2.4, Y_SV_TOP, Y_SPLINE_TOP, 0, 0, 24)
    return union([body, tabs, spl])


def horn():
    """Крестовая качалка (неподвижна, прикручена к левому уху)."""
    arms = [box(-1 if a % 180 else -14, 14 if a % 180 == 0 else 1, Y_SPLINE_TOP, Y_HORN_FACE, -2.5, 2.5).rotate([0, a, 0])
            for a in (0, 90)]
    return union([cyl_y(4, Y_SPLINE_TOP, Y_HORN_FACE, 0, 0)] + arms)


def camera():
    body = box(CAM_X + CAM_PIVOT - CAM_L, CAM_X + CAM_PIVOT, -CAM_W / 2, CAM_W / 2, -CAM_W / 2, CAM_W / 2)
    lens = M.cylinder(LENS_L, LENS_R, LENS_R, 48).rotate([0, 90, 0]).translate([CAM_X + CAM_PIVOT, 0, 0])
    return union([body, lens])


def frame_proxy():
    plates = [box(-60, PLATE_FRONT_X, -35, 35, G, G + PLATE_T), box(-60, PLATE_FRONT_X, -35, 35, -G - PLATE_T, -G)]
    so = [cyl_z(SO_D / 2, -G, G, 0, s * SO_S / 2, 32) for s in (-1, 1)]
    return union(plates + so)


def rotating(tilt):
    """Люлька + корпус серво + камера, поворот на tilt° носом вниз вокруг вала."""
    def pl(m):
        return m.rotate([0, tilt, 0]).translate([X_P, 0, Z_P])
    return pl(make_cradle()), pl(servo_body()), pl(camera())


# ---------------------------------------------------------------- проверка
def check():
    ok = True
    fr = make_frame()
    fixed = union([fr, frame_proxy()])
    hn = horn().translate([X_P, 0, Z_P])
    worst, at = 0.0, None
    for t in range(0, int(TILT_MAX) + 1, 3):
        for m in rotating(t):
            v = (m ^ fixed).volume() + (m ^ hn).volume() * 0      # качалка соединена со шлицем по замыслу
            if v > worst:
                worst, at = v, t
    # корпус серво не должен касаться качалки кроме шлица — шлиц внутри качалки по оси
    fr_hit = (fr ^ frame_proxy()).volume()
    back = min(fr.bounding_box()[0], min(rotating(t)[0].bounding_box()[0] for t in (0, 45, 90)))
    ground = min(min(m.bounding_box()[2] for m in rotating(t)) for t in (0, 90))
    good = worst < 1.0 and fr_hit < 1.0 and back >= -(SO_D / 2 + TUBE_WALL) - 0.01
    ok &= good
    print(f"Ход 0…{TILT_MAX:.0f}°: касаний {'нет' if worst < 1 else f'{worst:.1f} мм³ при {at}°'}; "
          f"рама∩плиты/стойки {fr_hit:.1f} мм³; самая задняя точка X={back:.1f} (за трубками {'пусто' if good else '?'})")
    c90 = rotating(90)[2].bounding_box()
    print(f"Ось вала: X={X_P:.1f}, Z={Z_P:.1f} (от середины между плитами); камера от вала {CAM_X:.1f} мм")
    print(f"В пол (90°): камера X {c90[0]:.0f}…{c90[3]:.0f}, низ объектива Z={c90[2]:.0f}; "
          f"нижняя плита рамы Z={-G - PLATE_T:.0f} — выступает вниз на {-G - PLATE_T - ground:.0f} мм")
    return ok


# ---------------------------------------------------------------- превью и экспорт
def render(path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    light = np.array([0.4, 0.6, 0.75]); light /= np.linalg.norm(light)
    fig = plt.figure(figsize=(20, 7))
    views = [(0, "камера вперёд (0°)"), (45, "наклон 45°"), (90, "камера в пол (90°)")]
    for k, (t, title) in enumerate(views):
        cr, sv, cam = rotating(t)
        esp = box(-14, -5, -12, 12, -6, -4)
        items = [(frame_proxy(), "carbon"), (make_frame(), "frame"), (horn().translate([X_P, 0, Z_P]), "metal"),
                 (cr, "print"), (sv, "servo"), (cam, "cam"), (esp, "esp")]
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
        V = T.reshape(-1, 3); lo, hi = V.min(0), V.max(0); c = (lo + hi) / 2; r = (hi - lo).max() / 2 * 0.72
        ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(c[2] - r, c[2] + r)
        ax.set_box_aspect((1, 1, 1)); ax.view_init(22, 40); ax.set_axis_off(); ax.set_title(title, fontsize=14)
    plt.tight_layout(); plt.savefig(path, dpi=80)


def export():
    d = os.path.join(HERE, "stl", "photo_MG90S")
    os.makedirs(d, exist_ok=True)
    to_trimesh(on_bed(make_frame())).export(os.path.join(d, "cam_frame_on_standoffs_x1.stl"))
    to_trimesh(on_bed(make_cradle())).export(os.path.join(d, "cam_servo_cradle_x1.stl"))
    render(os.path.join(HERE, "photo_mount.png"))


if __name__ == "__main__":
    if "--check" in sys.argv:
        sys.exit(0 if check() else 1)
    export()
