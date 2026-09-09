#!/usr/bin/env python3
"""Bundle cad/atech_duck_stand.glb + cad/BOM.csv into docs/cad_page.html (self-contained viewer).

    python scripts/build_cad_page.py  -> docs/cad_page.html
"""
import os, sys, csv, json, base64
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
tpl = open(os.path.join(ROOT, 'docs/cad_page.tpl.html')).read()
glb = base64.b64encode(open(os.path.join(ROOT, 'cad/atech_duck_stand.glb'), 'rb').read()).decode('ascii')
bom = list(csv.DictReader(open(os.path.join(ROOT, 'cad/BOM.csv'))))
bom = [dict(part=r['part'].replace('atech_motherboard_14_port', 'atech_board'), qty=r['qty'], kind=r['kind'], print_file=os.path.basename(r['print_file']), bbox_mm=r['bbox_mm'], solid_g=r['solid_g']) for r in bom]
html = tpl.replace('/*__DATA__*/', 'const GLB_B64 = ' + json.dumps(glb) + ';\nconst BOM = ' + json.dumps(bom) + ';')
out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, 'docs/cad_page.html')
open(out, 'w').write(html)
print(out, len(html) // 1024, 'KB')
