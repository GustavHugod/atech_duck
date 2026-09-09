#!/usr/bin/env python3
"""Mechanical BOM generated from the CAD (robot_motors.xml part list + the Atech board).

    python scripts/build_bom.py   -> cad/BOM.csv and the generated section of BOM.md

Quantities are the number of occurrences in the assembly. Volume is the closed-mesh volume
of the part; "solid g" = volume x 1.24 g/cm3 (PLA) or 1.21 (TPU), i.e. an upper bound
before infill. Nothing here is a supplier part number.
"""
import os, sys, csv, collections, trimesh
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from duck_parts import ROOT, load_parts, print_file, board_mesh, REPLACED_BY_ATECH

DENSITY = {'printed PLA': 1.24, 'printed TPU': 1.21}
START, END = '<!-- bom:generated:start -->', '<!-- bom:generated:end -->'

def main():
    parts = load_parts(include_replaced=True)
    qty = collections.Counter(p['name'] for p in parts)
    rows = []
    ORDER = ['printed PLA', 'printed TPU', 'servo horn (STS3215)', 'bearing', 'micro servo SG90', 'antenna (steel wire)', 'battery cell 18650', 'electronics', 'PCB']
    kind_of_name = {p['name']: p['kind'] for p in parts}
    rank = lambda k: ORDER.index(k) if k in ORDER else len(ORDER)
    for name in sorted(qty, key=lambda n: (rank(kind_of_name[n]), n)):
        p = next(x for x in parts if x['name'] == name)
        m = trimesh.load(p['file'], force='mesh')
        ext = m.bounding_box.extents * 1000
        vol = abs(m.volume) * 1e6 if m.is_watertight else float('nan')
        dens = DENSITY.get(p['kind'])
        rows.append(dict(part=name, qty=qty[name], kind=p['kind'], print_file=print_file(name),
                         bbox_mm=f'{ext[0]:.0f} x {ext[1]:.0f} x {ext[2]:.0f}',
                         volume_cm3=f'{vol:.1f}' if vol == vol else 'open mesh',
                         solid_g=f'{vol * dens:.0f}' if dens and vol == vol else '',
                         faces=len(m.faces), note='replaced by ' + REPLACED_BY_ATECH[name] if name in REPLACED_BY_ATECH else ''))
    b = board_mesh(); ext = b.bounding_box.extents * 1000
    rows.append(dict(part='atech_motherboard_14_port', qty=1, kind='Atech board', print_file='',
                     bbox_mm=f'{ext[0]:.0f} x {ext[1]:.0f} x {ext[2]:.0f}', volume_cm3='', solid_g='', faces=len(b.faces), note='position in the trunk assumed (DESIGN.md Q1)'))
    os.makedirs(os.path.join(ROOT, 'cad'), exist_ok=True)
    with open(os.path.join(ROOT, 'cad/BOM.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
    printed = [r for r in rows if r['kind'].startswith('printed')]
    pla = sum(float(r['solid_g']) * r['qty'] for r in printed if r['kind'] == 'printed PLA' and r['solid_g'])
    tpu = sum(float(r['solid_g']) * r['qty'] for r in printed if r['kind'] == 'printed TPU' and r['solid_g'])
    md = [START, '', '## Mechanical parts — generated from the CAD',
          '', f'`scripts/build_bom.py` reads the part list of the Open Duck Mini v2 assembly (`robot_motors.xml`, '
          f'{len(parts)} part occurrences, {len(qty)} distinct) and the Atech board. Full table with volumes: `cad/BOM.csv`. '
          f'Posed CAD: `cad/atech_duck_stand.glb` (`scripts/export_cad.py`).', '',
          f'Printed parts: {sum(r["qty"] for r in printed if r["kind"]=="printed PLA")} PLA pieces ≈ {pla:.0f} g solid, '
          f'{sum(r["qty"] for r in printed if r["kind"]=="printed TPU")} TPU pieces ≈ {tpu:.0f} g solid (100 % infill upper bound; '
          f'the ODM v2 guide prints shells at 3 walls / 15 % infill, so plan ~40–50 % of that).', '',
          '| part | qty | kind | print file | bbox mm | solid g | note |', '|---|---|---|---|---|---|---|']
    for r in rows:
        md.append(f"| {r['part']} | {r['qty']} | {r['kind']} | {r['print_file'].replace('reference/Open_Duck_Mini/print/', '') or '—'} | {r['bbox_mm']} | {r['solid_g'] or '—'} | {r['note']} |")
    md += ['', END]
    p = os.path.join(ROOT, 'BOM.md'); s = open(p).read()
    if START in s: s = s[:s.index(START)] + '\n'.join(md) + s[s.index(END) + len(END):]
    else: s = s.rstrip() + '\n\n' + '\n'.join(md) + '\n'
    open(p, 'w').write(s)
    print(f'{len(rows)} BOM rows -> cad/BOM.csv, BOM.md')

if __name__ == '__main__':
    main()
