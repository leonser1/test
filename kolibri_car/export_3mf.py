"""
Собирает STL «Колибри-кар v2» в 3MF по материалам: детали уже лежат как печатать,
разложены на стол BED × BED с зазором, у каждой своё имя, нужное количество копий.

Запуск: python3 export_3mf.py   (после python3 car_v2.py)  -> 3mf/*.3mf
"""
import os
import zipfile
from xml.sax.saxutils import escape

import numpy as np
import trimesh
import manifold3d as m3d

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "stl", "v2")
OUT = os.path.join(HERE, "3mf")
BED = 220.0
GAP = 6.0

PLATES = {
    "Kolibri_v2_1_PETG": [("v2_frame_x1.stl", 1), ("v2_front_rim_625_x2.stl", 2),
                          ("v2_rear_wheel_rod10_x2.stl", 2), ("v2_bearing_spacer_x2.stl", 2)],
    "Kolibri_v2_2_PA-CF": [("v2_V4_pinion_12T_x1.stl", 1), ("v2_V4_gear_48T_x1.stl", 1),
                           ("v2_knuckle_L_x1.stl", 1), ("v2_knuckle_R_x1.stl", 1), ("v2_tie_bar_x1.stl", 1),
                           ("toe/v2_tie_bar_minus0.4_toe_out_x1.stl", 1), ("toe/v2_tie_bar_plus0.4_toe_in_x1.stl", 1)],
    "Kolibri_v2_3_TPU": [("v2_tire_TPU_x4.stl", 4)],
    "Kolibri_v2_4_adapter_B_optional_PETG": [("v2_adapter_B_optional_x1.stl", 1)],
}


def pack(sizes):
    """Полки: крупные первыми. sizes: [(w, h)] -> [(x, y)] левый нижний угол; ошибка, если не влезло."""
    order = sorted(range(len(sizes)), key=lambda i: -sizes[i][1])
    pos = [None] * len(sizes)
    x = y = GAP
    shelf_h = 0.0
    for i in order:
        w, h = sizes[i]
        if x + w + GAP > BED:
            x, y = GAP, y + shelf_h + GAP
            shelf_h = 0.0
        if y + h + GAP > BED or x + w + GAP > BED:
            raise ValueError(f"не влезает на стол {BED:.0f}: {sizes[i]}")
        pos[i] = (x, y)
        x += w + GAP
        shelf_h = max(shelf_h, h)
    return pos


def write_3mf(path, items):
    """items: [(name, trimesh, (tx, ty))] — меш в своих координатах, ставится сдвигом."""
    objs, build = [], []
    for oid, (name, m, (tx, ty)) in enumerate(items, start=1):
        v = "".join(f'<vertex x="{a:.4f}" y="{b:.4f}" z="{c:.4f}"/>' for a, b, c in m.vertices)
        t = "".join(f'<triangle v1="{a}" v2="{b}" v3="{c}"/>' for a, b, c in m.faces)
        objs.append(f'<object id="{oid}" name="{escape(name)}" type="model"><mesh><vertices>{v}</vertices>'
                    f'<triangles>{t}</triangles></mesh></object>')
        build.append(f'<item objectid="{oid}" transform="1 0 0 0 1 0 0 0 1 {tx:.3f} {ty:.3f} 0"/>')
    model = ('<?xml version="1.0" encoding="UTF-8"?>\n'
             '<model unit="millimeter" xml:lang="ru-RU" '
             'xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">'
             '<metadata name="Title">' + escape(os.path.basename(path)) + '</metadata>'
             '<resources>' + "".join(objs) + '</resources><build>' + "".join(build) + '</build></model>')
    ctypes = ('<?xml version="1.0" encoding="UTF-8"?>\n'
              '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
              '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>'
              '<Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>'
              '</Types>')
    rels = ('<?xml version="1.0" encoding="UTF-8"?>\n'
            '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
            '<Relationship Target="/3D/3dmodel.model" Id="rel0" '
            'Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/></Relationships>')
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("[Content_Types].xml", ctypes)
        z.writestr("_rels/.rels", rels)
        z.writestr("3D/3dmodel.model", model)


def read_back(path):
    """Читает 3MF обратно (свой разбор XML): суммарный объём и все ли детали в пределах стола."""
    import xml.etree.ElementTree as ET
    ns = {"m": "http://schemas.microsoft.com/3dmanufacturing/core/2015/02"}
    with zipfile.ZipFile(path) as z:
        root = ET.fromstring(z.read("3D/3dmodel.model"))
    meshes = {}
    for o in root.findall(".//m:object", ns):
        v = np.array([[float(e.get(k)) for k in "xyz"] for e in o.findall(".//m:vertex", ns)])
        f = np.array([[int(e.get(k)) for k in ("v1", "v2", "v3")] for e in o.findall(".//m:triangle", ns)])
        meshes[o.get("id")] = trimesh.Trimesh(v, f, process=False)
    vol, inside = 0.0, True
    for it in root.findall(".//m:item", ns):
        t = [float(x) for x in it.get("transform").split()]
        m = meshes[it.get("objectid")]
        vol += m.volume
        lo, hi = m.bounds[0] + t[9:12], m.bounds[1] + t[9:12]
        inside &= bool(lo[0] >= 0 and lo[1] >= 0 and hi[0] <= BED and hi[1] <= BED and abs(lo[2]) < 1e-6)
    return vol, inside


# ---------------------------------------------------------------- всё на один стол
ONE_PLATE = [  # (файл, кол-во, материал)
    ("v2_frame_x1.stl", 1, "PETG"), ("v2_front_rim_625_x2.stl", 2, "PETG"),
    ("v2_rear_wheel_rod10_x2.stl", 2, "PETG"), ("v2_V4_gear_48T_x1.stl", 1, "PA-CF"),
    ("v2_tie_bar_x1.stl", 1, "PA-CF"), ("toe/v2_tie_bar_minus0.4_toe_out_x1.stl", 1, "PA-CF"),
    ("toe/v2_tie_bar_plus0.4_toe_in_x1.stl", 1, "PA-CF"), ("v2_tire_TPU_x4.stl", 4, "TPU"),
]
# мелочь кладётся в отверстия шин (Ø49.6): (файл, материал, номер шины, смещение от центра)
NESTED = [("v2_knuckle_L_x1.stl", "PA-CF", 0, (0, 0)), ("v2_knuckle_R_x1.stl", "PA-CF", 1, (0, 0)),
          ("v2_V4_pinion_12T_x1.stl", "PA-CF", 2, (0, 0)),
          ("v2_bearing_spacer_x2.stl", "PETG", 3, (-9, 0)), ("v2_bearing_spacer_x2.stl", "PETG", 3, (9, 0))]
PGAP = 3.0


def _round(m):
    """Круглая деталь? (габарит XY почти квадрат и площадь проекции ≈ круга)."""
    w, h = m.extents[:2]
    return abs(w - h) < 0.5 and w > 30


def _fits(shape, x, y, placed, bed):
    kind, a, b = shape
    if kind == "c":
        r = a
        if x - r < PGAP or y - r < PGAP or x + r > bed - PGAP or y + r > bed - PGAP:
            return False
    else:
        w, h = a, b
        if x < PGAP or y < PGAP or x + w > bed - PGAP or y + h > bed - PGAP:
            return False
    for (k2, a2, b2), (x2, y2) in placed:
        if kind == "c" and k2 == "c":
            if (x - x2) ** 2 + (y - y2) ** 2 < (a + a2 + PGAP) ** 2:
                return False
        elif kind == "r" and k2 == "r":
            if x < x2 + a2 + PGAP and x2 < x + a + PGAP and y < y2 + b2 + PGAP and y2 < y + b + PGAP:
                return False
        else:
            (cx, cy, r), (rx, ry, rw, rh) = ((x, y, a), (x2, y2, a2, b2)) if kind == "c" else ((x2, y2, a2), (x, y, a, b))
            dx = max(rx - cx, 0, cx - (rx + rw)); dy = max(ry - cy, 0, cy - (ry + rh))
            if dx * dx + dy * dy < (r + PGAP) ** 2:
                return False
    return True


def pack_one(shapes, bed, step=1.0):
    """Жадная раскладка «ниже-левее»: круги как круги, остальное прямоугольники (с поворотом 90°)."""
    order = sorted(range(len(shapes)), key=lambda i: -(shapes[i][1] ** 2 * 3.14 if shapes[i][0] == "c"
                                                       else shapes[i][1] * shapes[i][2]))
    placed, res = [], [None] * len(shapes)
    grid = np.arange(0, bed + step, step)
    for i in order:
        kind, a, b = shapes[i]
        variants = [(shapes[i], False)] + ([(("r", b, a), True)] if kind == "r" else [])
        best = None
        for shp, rot in variants:
            for y in grid:
                if best and y > best[1]:
                    break
                for x in grid:
                    if _fits(shp, x, y, placed, bed):
                        if not best or (y, x) < (best[1], best[0]):
                            best = (x, y, shp, rot)
                        break
        if not best:
            return None
        placed.append((best[2], (best[0], best[1])))
        res[i] = best
    return res


def _proj(m):
    """Контур детали на столе (CrossSection) с припуском PGAP/2."""
    mm = m3d.Manifold(m3d.Mesh(vert_properties=np.asarray(m.vertices, np.float32),
                               tri_verts=np.asarray(m.faces, np.uint32)))
    return mm.project().offset(PGAP / 2, m3d.JoinType.Round)


def pack_true(meshes, bed, step=2.0):
    """«Ниже-левее» по реальным контурам: мелочь сама попадает в отверстия шин и карманы рамы."""
    order = sorted(range(len(meshes)), key=lambda i: -meshes[i][1].area_projected if False else
                   -np.prod(meshes[i][1].extents[:2]))
    placed = None
    res = [None] * len(meshes)
    lim = bed - PGAP / 2
    for i in order:
        name, m = meshes[i]
        best = None
        for rot in (0, 90):
            mm = m.copy()
            if rot:
                mm.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [0, 0, 1]))
            mm.apply_translation(-mm.bounds[0])
            pr = _proj(mm)
            (bx0, by0), (bx1, by1) = pr.bounds()[:2], pr.bounds()[2:]
            w, h = bx1 - bx0, by1 - by0
            for y in np.arange(PGAP / 2, lim - h + 1e-6, step):
                if best and y > best[1]:
                    break
                for x in np.arange(PGAP / 2, lim - w + 1e-6, step):
                    cand = pr.translate([x - bx0, y - by0])
                    if placed is None or (cand ^ placed).area() < 1e-3:
                        if not best or (y, x) < (best[1], best[0]):
                            best = (x - bx0, y - by0, mm, cand)
                        break
        if not best:
            return None
        placed = best[3] if placed is None else placed + best[3]
        res[i] = (name, best[2], (best[0], best[1]))
    return res


def one_plate():
    meshes = []
    for fn, n, mat in ONE_PLATE + [(f, 1, mt) for f, mt, _, _ in NESTED]:
        m = trimesh.load(os.path.join(SRC, fn))
        m.apply_translation(-m.bounds[0])
        base = os.path.basename(fn).replace(".stl", "").replace("v2_", "")
        for k in range(n):
            meshes.append((f"{mat}_{base}_{k + 1}" if n > 1 else f"{mat}_{base}", m))
    # одинаковые имена (2 проставки из NESTED) — пронумеровать
    seen = {}
    for i, (nm, m) in enumerate(meshes):
        seen[nm] = seen.get(nm, 0) + 1
        if seen[nm] > 1:
            meshes[i] = (f"{nm}_{seen[nm]}", m)
    for bed in (220.0, 235.0, 250.0, 256.0):
        res = pack_true(meshes, bed)
        print(f"  стол {bed:.0f}: {'влезло' if res else 'не влезло'}")
        if res:
            return res, bed
    raise RuntimeError("не влезает даже на 256")


def main():
    os.makedirs(OUT, exist_ok=True)
    items, bed = one_plate()
    global BED
    keep, BED = BED, bed
    path = os.path.join(OUT, f"Kolibri_v2_ALL_one_plate_{bed:.0f}.3mf")
    write_3mf(path, items)
    vol, inside = read_back(path)
    print(f"ВСЁ НА ОДНОМ СТОЛЕ {bed:.0f}×{bed:.0f}: деталей {len(items)}, объём {vol / 1000:.1f} см³, на столе: {'да' if inside else 'НЕТ'}")
    BED = keep
    for plate, files in PLATES.items():
        meshes = []
        for fn, n in files:
            m = trimesh.load(os.path.join(SRC, fn))
            m.apply_translation(-m.bounds[0])          # в угол, низ на столе
            base = os.path.basename(fn).replace(".stl", "")
            for k in range(n):
                meshes.append((f"{base}_{k + 1}" if n > 1 else base, m))
        sizes = [tuple(m.extents[:2]) for _, m in meshes]
        pos = pack(sizes)
        items = [(name, m, p) for (name, m), p in zip(meshes, pos)]
        path = os.path.join(OUT, plate + ".3mf")
        write_3mf(path, items)
        # проверка: перечитать и сверить
        vol, inside = read_back(path)
        ref = sum(m.volume for _, m in meshes)
        print(f"{plate + '.3mf':44s} деталей {len(items):2d}  объём {vol / 1000:6.1f} см³ (STL {ref / 1000:6.1f})  "
              f"{os.path.getsize(path) / 1024:6.0f} КБ  на столе: {'да' if inside else 'НЕТ'}")


if __name__ == "__main__":
    main()
