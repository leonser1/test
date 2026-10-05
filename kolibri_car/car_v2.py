"""
«Колибри-кар v2»: печатная рама под дроном Mark4 10".

Перед — как в generate_car.py: 2 поворотных кулака на шкворнях, серво MG90S
поворачивает оба колеса через поперечную тягу (кинематика проверена).
Зад — один мотор 2814 крутит длинную сквозную палку Ø10 через зубчатую передачу
(без ремней), оба задних колеса надеты на концы палки. Палка в двух подшипниках
6800ZZ в стенках коробки, мотор стоит позади палки параллельно ей.

Запуск:  python3 car_v2.py           -> stl/v2/*.stl + preview_v2.png
         python3 car_v2.py --check   -> руль, зацепление, касания, клиренс, винты
"""
import math
import os
import sys

import numpy as np

import generate_car as gc
import drive_variants as dv
from generate_car import M, m3d, box, cyl_y, cyl_z, union, hex_prism, mirror_y, to_trimesh, on_bed

HERE = os.path.dirname(os.path.abspath(__file__))

# ---------------- ПАРАМЕТРЫ (мм) ----------------
DRIVE = "V4"            # передача из drive_variants.py (выбор агентов: V4 косозубые 12→48)
ROD_X = -60.0           # задняя ось (палка); перед — FRONT_X = +50
ROD_Y = 62.0            # палка от -62 до +62 (124 мм)
WALL_IN = 24.0          # стенка мотора: внутренняя сторона Y=-24 (правая — по ступице шестерни)
MOTOR_SLOT = 0.5        # пазы под винты мотора ±0.5 мм: регулировка зазора в зубьях
WALL_T = 6.0
WALL_TOP = 40.0         # нижняя плита дрона на 42
REAR_WHEEL_IN_Y = 40.0  # колесо Y 40..60
CUT_X = -40.0           # от старой палубы берём всё, что впереди этой линии
# ------------------------------------------------

Z = gc.AXLE_Z
DY = -WALL_IN           # сдвиг координат модуля привода: его левая стенка y=0 -> машина y=-24


def drive():
    v = dict(dv.VARIANTS)[DRIVE]()
    return v


def to_car(man):
    """Координаты drive_variants (палка X=0, Z=ROD_Z, мотор в -X) -> координаты машины."""
    return man.translate([ROD_X, DY, Z - dv.ROD_Z])


def motor_x(v):
    x0, _, _, x1, _, _ = v["motor"].bounding_box()
    return ROD_X + (x0 + x1) / 2


def rear_frame(v):
    mx = motor_x(v)
    back_x = mx - dv.MOTOR_R - 6
    x0, x1 = back_x - 4, ROD_X + 14
    yA0, yA1 = -WALL_IN - WALL_T, -WALL_IN
    yB0 = max(to_car(p).bounding_box()[4] for _, p, _, _ in v["parts"]) + 1.5
    yB1 = yB0 + WALL_T
    parts = [box(x0 - 2, CUT_X + 0.5, yA0 - 1, yB1 + 1, -gc.DECK_T, 0),     # основание
             box(x0, x1, yA0, yA1, 0, WALL_TOP),
             box(x0, x1, yB0, yB1, 0, WALL_TOP - 4),
             cyl_y(13, yA0, yA1, ROD_X, Z), cyl_y(13, yB0, yB1, ROD_X, Z)]
    back = box(x0, x0 + 4, yA0, yB1, 0, WALL_TOP) - box(x0 - 1, x0 + 5, -WALL_IN + 4, WALL_IN - 4, 7, WALL_TOP - 6)
    parts.append(back)
    # косынки: сзади снаружи, спереди изнутри (снаружи спереди — колёса)
    for s, yo, yi in ((1, yB1, yB0), (-1, -yA0, -yA1)):
        g = M.hull_points([[x0, yo, 0], [x0 + 4, yo, 0], [x0, yo + 8, 0], [x0 + 4, yo + 8, 0],
                           [x0, yo, 30], [x0 + 4, yo, 30]])
        gi = M.hull_points([[x1 - 4, yi, 0], [x1, yi, 0], [x1 - 4, yi - 7, 0],
                            [x1, yi - 7, 0], [x1 - 4, yi, 28], [x1, yi, 28]])
        parts += [g, gi] if s > 0 else [mirror_y(g), mirror_y(gi)]
    # ось промежуточного блока (V5): бобышка от стенки B
    idler = v.get("idler_x")
    if idler is not None:
        ix = ROD_X + idler
        parts.append(cyl_y(6, v["idler_y1"] + DY + 0.8, yB0 + 0.01, ix, Z))
    f = union(parts)
    cuts = []
    # мотор на стенке A: 4 × M3 по диагонали Ø19 (квадрат 13.4) + центр
    sl = MOTOR_SLOT
    cuts.append(M.batch_hull([cyl_y(6.0, yA0 - 1, yA1 + 1, mx - sl, Z), cyl_y(6.0, yA0 - 1, yA1 + 1, mx + sl, Z)]))
    for dx, dz in dv.MOTOR_HOLES:
        cuts.append(M.batch_hull([cyl_y(1.65, yA0 - 1, yA1 + 1, mx + dx - sl, Z + dz, 20),
                                  cyl_y(1.65, yA0 - 1, yA1 + 1, mx + dx + sl, Z + dz, 20)]))
    # подшипники 6800 снаружи, отверстие под палку
    for y0, y1, out in ((yA0, yA1, yA0), (yB0, yB1, yB1)):
        cuts.append(cyl_y(6.5, y0 - 1, y1 + 1, ROD_X, Z))
        cuts.append(cyl_y(dv.BRG_D / 2, out - 1 if out == yA0 else out - dv.BRG_W,
                          out + dv.BRG_W if out == yA0 else out + 1, ROD_X, Z))
    cuts.append(cyl_y(7.0, yB0 - 1, yB1 + 1, mx, Z))   # торец вала мотора / гайка
    if idler is not None:
        cuts.append(cyl_y(2.6, v["idler_y0"] + DY - 1, yB1 + 1, ROD_X + idler, Z, 24))
        cuts.append(hex_prism(8.2, 0, 4.2).rotate([-90, 0, 0]).translate([ROD_X + idler, yB1 - 4.2, Z]))
    # прорези под шестерни (зазор 1 мм)
    for _, p, _, _ in v["parts"]:
        q = to_car(p)
        bx0, by0, bz0, bx1, by1, bz1 = q.bounding_box()
        r = max(bx1 - bx0, bz1 - bz0) / 2
        cuts.append(cyl_y(r + 1.0, by0 - 0.6, by1 + 0.6, (bx0 + bx1) / 2, (bz0 + bz1) / 2))
    # окно под провода мотора в основании
    cuts.append(box(mx - 10, mx + 10, -WALL_IN + 3, -WALL_IN + 9, -gc.DECK_T - 1, 1))
    return f - union(cuts)


def make_deck_v2(v):
    front = gc.make_deck() ^ box(CUT_X, 200, -100, 100, -50, 100)
    return union([front, rear_frame(v)])


def make_rear_wheel_rod():
    """Заднее колесо на палку Ø10: обод-стакан как передний, ступица с поперечным M3."""
    W = gc.WHEEL_W
    rim = union([cyl_z(gc.RIM_R, 0, W), cyl_z(gc.FLANGE_R, 0, 1.5),
                 M.cylinder(2.0, gc.RIM_R, gc.FLANGE_R, gc.SEG).translate([0, 0, W - 3.5]),
                 cyl_z(gc.FLANGE_R, W - 1.5, W)]) - cyl_z(gc.RIM_IN_R, gc.HUB_T, W + 1)
    rim = union([rim, cyl_z(10, 0, 16)])
    cuts = [cyl_z(dv.ROD_R + 0.15, -1, 17, seg=48)]
    # 2 стопорных M3 под 90° на лыску палки, гайки в пазах с внутреннего торца ступицы
    for ang in (0, 90):
        a = math.radians(ang)
        ux, uy = math.cos(a), math.sin(a)
        rn = dv.ROD_R + 2.3
        cuts.append(M.cylinder(11, 1.6, 1.6, 16).rotate([0, 90, 0]).rotate([0, 0, ang]).translate([0, 0, 11]))
        nut = hex_prism(5.8, -1.3, 1.3).rotate([0, 90, 0]).rotate([0, 0, ang]).translate([rn * ux, rn * uy, 11])
        slot = box(-1.3, 1.3, -2.9, 2.9, 11, 17).rotate([0, 0, ang]).translate([rn * ux, rn * uy, 0])
        cuts += [nut, slot]
    for i in range(6):
        a = math.radians(i * 60 + 30)
        cuts.append(cyl_z(3.5, -1, gc.HUB_T + 1, 16.5 * math.cos(a), 16.5 * math.sin(a), 32))
    return rim - union(cuts)


def place_rear(w, side):
    m = w.rotate([90, 0, 0]).translate([ROD_X, REAR_WHEEL_IN_Y + gc.WHEEL_W, Z])
    return m if side > 0 else mirror_y(m)


def rod_proxy():
    return cyl_y(dv.ROD_R, -ROD_Y, ROD_Y, ROD_X, Z, 48)


def assembly(v, servo=0.0, with_drone=False):
    st = gc.steer_state(gc.delta_for_servo(servo)) if servo else gc.steer_state(0.0)
    deck = make_deck_v2(v)
    kn = gc.make_knuckle()
    tire = gc.make_tire().translate([0, 0, 1.7])
    fw = union([gc.make_rim(True), tire])
    rw = union([make_rear_wheel_rod(), tire])
    parts = [deck, gc.servo_proxy(st["phi"]), gc.stack_proxy(), gc.place_bar(gc.make_tie_bar(), st),
             rod_proxy(), to_car(v["motor"])] + [to_car(p) for _, p, _, _ in v["parts"]]
    for s in (-1, 1):
        front = union([kn, gc.place_wheel(fw, gc.FRONT_X)])
        parts.append(gc.place_knuckle_state(front, st["dl"] if s > 0 else st["dr"], s))
        parts.append(place_rear(rw, s))
    if with_drone:
        parts.append(gc.drone_proxy())
    return union(parts)


def colored(v, servo=0.0):
    """Список (тело, цвет) для рендера."""
    st = gc.steer_state(gc.delta_for_servo(servo)) if servo else gc.steer_state(0.0)
    kn = gc.make_knuckle()
    tire = gc.make_tire().translate([0, 0, 1.7])
    out = [(make_deck_v2(v), "frame"), (gc.servo_proxy(st["phi"]), "motor"), (gc.stack_proxy(), "motor"),
           (gc.place_bar(gc.make_tie_bar(), st), "print"), (rod_proxy(), "rod"), (to_car(v["motor"]), "motor")]
    out += [(to_car(p), "print") for _, p, _, _ in v["parts"]]
    for s in (-1, 1):
        d = st["dl"] if s > 0 else st["dr"]
        out.append((gc.place_knuckle_state(kn, d, s), "print"))
        out.append((gc.place_knuckle_state(gc.place_wheel(gc.make_rim(True), gc.FRONT_X), d, s), "print"))
        out.append((gc.place_knuckle_state(gc.place_wheel(tire, gc.FRONT_X), d, s), "tpu"))
        out.append((place_rear(make_rear_wheel_rod(), s), "print"))
        out.append((place_rear(tire, s), "tpu"))
    return out


# ---------------------------------------------------------------- проверки
def check(v):
    ok = True
    deck = make_deck_v2(v)
    kn = gc.make_knuckle()
    tire = gc.make_tire().translate([0, 0, 1.7])
    fw = union([gc.make_rim(True), tire])
    print(f"Передача {DRIVE}: {v['name']}, 1:{v['ratio']:.3g}; мотор X={motor_x(v):.1f}, палка X={ROD_X}")
    ok &= gc.steering_check(deck, kn, fw)
    front = union([kn, gc.place_wheel(fw, gc.FRONT_X)])
    env = union([gc.servo_body_proxy(), gc.stack_proxy()])
    free = None
    for ang in range(0, 41):
        hit = any(((gc.place_knuckle(front, a, s) ^ deck).volume() + (gc.place_knuckle(front, a, s) ^ env).volume()) > 0.5
                  for s in (-1, 1) for a in (ang, -ang))
        if hit:
            break
        free = ang
    print(f"Передние колёса свободно до ±{free}°")
    ok &= free >= gc.STEER_MAX
    for (dc, dp, dph, nc, npos, nph, ratio, internal) in v.get("mesh", []):
        worst, eng = dv.mesh_sweep(dc, dp, dph, nc, npos, nph, ratio, internal=internal)
        good = worst < 0.05 and eng > 0.05
        ok &= good
        print(f"Зацепление: клин {worst:.2f} мм², в зацеплении {eng:.2f} мм² [{'OK' if good else 'ПРОБЛЕМА'}]")
    rw = union([make_rear_wheel_rod(), tire])
    mot = to_car(v["motor"])
    moving = [to_car(p) for _, p, _, _ in v["parts"]]
    tests = [("мотор", mot)] + [(n, to_car(p)) for n, p, _, _ in v["parts"]] + \
            [(f"заднее колесо {'Л' if s > 0 else 'П'}", place_rear(rw, s)) for s in (-1, 1)]
    for name, body in tests:
        vol = (body ^ deck).volume()
        if vol > 0.5:
            print(f"  {name} ∩ рама: {vol:.1f} мм³")
            ok = False
    for name, body in [(f"заднее колесо {'Л' if s > 0 else 'П'}", place_rear(rw, s)) for s in (-1, 1)]:
        for m2 in [mot] + moving:
            if (body ^ m2).volume() > 0.5:
                print(f"  {name} задевает привод"); ok = False
    print(f"Задний мост: касаний {'нет' if ok else 'ЕСТЬ'}")
    ground = Z - gc.TIRE_R
    low = min(deck.bounding_box()[2], min(p.bounding_box()[2] for p in moving))
    print(f"Земля Z={ground:.0f}; самая низкая точка рамы/передачи Z={low:.1f} -> клиренс {low - ground:.1f} мм")
    asm = assembly(v)
    top = max(deck.bounding_box()[5], mot.bounding_box()[5])
    print(f"Верх рамы/мотора Z={top:.1f}, нижняя плита дрона Z={gc.TOWER_H:.0f}")
    ok &= top <= gc.TOWER_H
    proj = asm.project()
    disks = None
    for sx in (-1, 1):
        for sy in (-1, 1):
            c = m3d.CrossSection.circle(gc.PROP_R, 128).translate([sx * gc.DRONE_MOTOR, sy * gc.DRONE_MOTOR])
            disks = c if disks is None else disks + c
    inside = (proj ^ disks).area()
    print(f"Под дисками винтов {inside / 100:.1f} см² = {100 * inside / (4 * math.pi * gc.PROP_R ** 2):.2f} %")
    bb = asm.bounding_box()
    print(f"Габарит машинки {bb[3] - bb[0]:.0f} × {bb[4] - bb[1]:.0f} × {bb[5] - bb[2]:.0f} мм; "
          f"база {gc.FRONT_X - ROD_X:.0f}, колея перед {2 * (gc.WHEEL_OUT_Y - 10):.0f} / зад {2 * (REAR_WHEEL_IN_Y + 10):.0f}")
    print(f"Скорость (2814 {dv.KV}KV, 6S, Ø70): ≈{dv.speed_kmh(v['ratio']):.0f} км/ч без нагрузки, "
          f"на 30 % газа ≈{0.3 * dv.speed_kmh(v['ratio']):.0f} км/ч")
    return ok


# ---------------------------------------------------------------- экспорт и превью
def render(v, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    views = [("Колибри-кар v2: колёса прямо", 0.0, (24, -125)), ("Серво +30°: поворот влево", 30.0, (35, -60)),
             ("Вид сверху", 30.0, (89.9, -90)), ("Задний мост: мотор 2814 → палка Ø10", 0.0, (18, -160))]
    fig = plt.figure(figsize=(16, 12.5))
    light = np.array([0.4, -0.6, 0.75]); light /= np.linalg.norm(light)
    cache = {}
    for k, (title, servo, (el, az)) in enumerate(views):
        if servo not in cache:
            items = colored(v, servo)
            T, C = [], []
            for man, col in items:
                tm = to_trimesh(man)
                sh = 0.35 + 0.65 * np.clip(tm.face_normals @ light, 0, 1)
                T.append(tm.vertices[tm.faces]); C.append(np.c_[np.outer(sh, dv.COL[col]), np.ones_like(sh)])
            cache[servo] = (np.concatenate(T), np.concatenate(C))
        T, C = cache[servo]
        if k == 3:   # крупно задний мост
            keep = T[:, :, 0].mean(1) < -30
            T, C = T[keep], C[keep]
        ax = fig.add_subplot(2, 2, k + 1, projection="3d")
        ax.add_collection3d(Poly3DCollection(T, facecolors=C, edgecolor="none"))
        V = T.reshape(-1, 3); lo, hi = V.min(0), V.max(0); c = (lo + hi) / 2; r = (hi - lo).max() / 2 * 0.75
        ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(c[2] - r, c[2] + r)
        ax.set_box_aspect((1, 1, 1)); ax.view_init(el, az); ax.set_axis_off(); ax.set_title(title, fontsize=14)
    plt.tight_layout(); plt.savefig(path, dpi=80)


def export(v):
    out = os.path.join(HERE, "stl", "v2")
    os.makedirs(out, exist_ok=True)
    kn = gc.make_knuckle()
    parts = {
        "v2_frame_x1.stl": on_bed(make_deck_v2(v)),
        "v2_rear_wheel_rod10_x2.stl": on_bed(make_rear_wheel_rod()),
        "v2_front_rim_625_x2.stl": on_bed(gc.make_rim(True)),
        "v2_bearing_spacer_x2.stl": on_bed(gc.make_bearing_spacer()),
        "v2_tire_TPU_x4.stl": on_bed(gc.make_tire()),
        "v2_knuckle_L_x1.stl": on_bed(kn.rotate([180, 0, 0])),
        "v2_knuckle_R_x1.stl": on_bed(mirror_y(kn).rotate([180, 0, 0])),
        "v2_tie_bar_x1.stl": on_bed(gc.make_tie_bar().rotate([180, 0, 0])),
        "v2_adapter_B_optional_x1.stl": on_bed(gc.make_adapter_b()),
    }
    for name, p, col, n in v["parts"]:
        if name and n:
            parts[f"v2_{DRIVE}_{name}_x{n}.stl"] = on_bed(p.rotate([90, 0, 0]))
    parts["preview_v2_assembly.stl"] = assembly(v)
    parts["preview_v2_with_drone.stl"] = assembly(v, with_drone=True)
    for name, man in parts.items():
        tm = to_trimesh(man)
        tm.export(os.path.join(out, name))
        lo, hi = tm.bounds
        print(f"{name:40s} {hi[0]-lo[0]:6.1f} × {hi[1]-lo[1]:6.1f} × {hi[2]-lo[2]:5.1f}  watertight={tm.is_watertight}")
    render(v, os.path.join(HERE, "preview_v2.png"))


if __name__ == "__main__":
    v = drive()
    if "--check" in sys.argv:
        sys.exit(0 if check(v) else 1)
    export(v)
