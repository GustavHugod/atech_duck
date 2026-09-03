#!/usr/bin/env python3
"""Bundle out/motion/{meshes,stock,atech}.json into the interactive motion page.

    python scripts/build_motion_page.py  -> docs/motion_page.html (also copied to the scratchpad if given)
"""
import os, sys, json
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
tpl = open(os.path.join(ROOT, 'docs/motion_page.tpl.html')).read()
meshes = json.load(open(os.path.join(ROOT, 'out/motion/meshes_hires.json')))
runs = {}
for n in ['stock', 'atech']:
    d = json.load(open(os.path.join(ROOT, f'out/motion/{n}_hires.json')))
    runs[n] = dict(dt=d['dt'], bodies=d['bodies'], actuators=d['actuators'], frames=d['frames'], torques=d['torques'],
                   feet=d['feet'], cmd=d['cmd'], label=d['label'], base=d['base'], stats=d['stats'], schedule=d['schedule'])
payload = json.dumps(dict(meshes=meshes, runs=runs), separators=(',', ':'))
html = tpl.replace('/*__DATA__*/', 'const DATA = ' + payload + ';')
out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, 'docs/motion_page.html')
open(out, 'w').write(html)
print(out, len(html) // 1024, 'KB')
