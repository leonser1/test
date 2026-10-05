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


def main():
    os.makedirs(OUT, exist_ok=True)
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
