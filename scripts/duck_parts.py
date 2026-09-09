"""Shared loader for the Open Duck Mini v2 high-resolution part model (onshape-to-robot export
in reference/Open_Duck_Mini/mini_bdx/robots/open_duck_mini_v2) plus the Atech board.

Every visual geom of robot_motors.xml becomes one part: body, mesh name, source file, pose in
the body frame, CAD colour, kind (printed PLA / TPU / servo horn / bearing / electronics ...).
Used by replay_hires.py (motion page meshes), export_cad.py (posed GLB/STL) and build_bom.py.
"""
import os, glob, base64, numpy as np, trimesh
import xml.etree.ElementTree as ET
from assemble_mjcf import tf, vec

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HR = os.path.join(ROOT, 'reference/Open_Duck_Mini/mini_bdx/robots/open_duck_mini_v2')
PRINT_DIR = os.path.join(ROOT, 'reference/Open_Duck_Mini/print')
BOARD = os.path.expanduser('~/atech_code/public/modules/3d/motherboard_14_port.stl')
BOARD_POS = (-0.06, 0, 0.075)           # assumed position in the trunk (DESIGN.md Q1)

# Parts of the stock ODM v2 that the Atech build replaces (DESIGN.md §5): kept out of the
# Atech CAD and the motion page, kept in the BOM as "replaced".
REPLACED_BY_ATECH = {'raspberrypizerow': 'Atech 14-port board', 'bno055': 'Atech Motion Sensor module'}

# Duck colour scheme, keyed by the CAD rgb of the export (same keys as docs/motion_page.tpl.html)
DUCK_COLOURS = {
    '0.914,0.914,0.914': 0xF5C518, '0.902,0.902,0.902': 0xF7D65A, '0.647,0.647,0.647': 0xF39C2B,
    '0.91,0.569,0.165': 0xE0503C,  '0.976,0.714,0.004': 0xF06A1C, '0.302,0.294,0.275': 0x4A3B32,
    '0.62,0.667,0.702': 0x2E3440,  '0.22,0.22,0.22': 0x1F2328,    '0.176,0.176,0.176': 0x3B4252,
    '0.4,0.4,0.4': 0x3B4252,       '0.412,0.412,0.412': 0x3B4252, '0.502,0.502,0.502': 0x4C566A,
    '0.6,0.6,0.6': 0xB8C0CC,       '0.231,0.376,0.702': 0x2F7BE0, '0.431,0.667,0.976': 0x4AA8F5,
    '0.612,0.812,0.929': 0x7CC7F0, '0.0,0.502,0.0': 0x2F6B4A,
}
ATECH_GREEN = 0x2F6B4A

def colour_key(rgba):
    return ','.join(str(round(int(c * 255) / 255, 3)) for c in rgba[:3])

def kind_of(name):
    if name.endswith('_tpu'): return 'printed TPU'
    if os.path.exists(os.path.join(PRINT_DIR, name + '.stl')): return 'printed PLA'
    if 'knee_to_ankle' in name: return 'printed PLA'          # print file is knee_to_ankle_*_sheet.stl (no left_ prefix)
    if 'palonier' in name: return 'servo horn (STS3215)'
    if name == 'roll_bearing': return 'bearing'
    if name == 'cell': return 'battery cell 18650'
    if name in ('bms', 'usb_c_charger', 'power_switch'): return 'electronics'
    if name in REPLACED_BY_ATECH: return 'electronics, replaced by ' + REPLACED_BY_ATECH[name]
    if name == 'sg90': return 'micro servo SG90'
    if name == 'antenna': return 'antenna (steel wire)'
    if name == 'board': return 'PCB'
    if name == 'holder': return 'placeholder box (holder)'
    return 'other'

def print_file(name):
    import re
    for cand in (name, re.sub(r'^(left|right)_', '', name)):
        p = os.path.join(PRINT_DIR, cand + '.stl')
        if os.path.exists(p): return os.path.relpath(p, ROOT)
    return ''

def load_parts(include_replaced=False):
    """-> list of dicts: body, name, file, pos, quat, rgba, key, kind (order = XML order)."""
    root = ET.parse(os.path.join(HR, 'robot_motors.xml')).getroot()
    files = {m.get('name'): os.path.join(HR, m.get('file')) for m in root.iter('mesh')}
    parts = []
    for body in root.iter('body'):
        for g in body.findall('geom'):
            if g.get('type') != 'mesh' or g.get('mesh') not in files: continue
            if g.get('contype') == '0' and g.get('conaffinity') == '0' and g.get('group') == '3': continue
            name = g.get('mesh')
            if name in REPLACED_BY_ATECH and not include_replaced: continue
            rgba = [float(v) for v in g.get('rgba').split()] if g.get('rgba') else [0.8, 0.8, 0.8, 1]
            parts.append(dict(body=body.get('name'), name=name, file=files[name],
                              pos=vec(g.get('pos'), 3, [0, 0, 0]), quat=vec(g.get('quat'), 4, [1, 0, 0, 0]),
                              rgba=rgba, key=colour_key(rgba), kind=kind_of(name)))
    return parts

_cache = {}
def part_mesh(p):
    """Full-resolution mesh of one part in its body frame (no decimation)."""
    if p['file'] not in _cache:
        _cache[p['file']] = trimesh.load(p['file'], force='mesh')
    m = _cache[p['file']].copy()
    m.apply_transform(tf(p['pos'], p['quat']))
    trimesh.repair.fix_normals(m)
    return m

def board_mesh():
    """Atech 14-port board (STL is Y-up, mm) -> Z-up metres at BOARD_POS in the trunk frame."""
    b = trimesh.load(BOARD, force='mesh')
    b.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0]))
    b.apply_scale(0.001)
    b.apply_translation(-b.bounding_box.centroid); b.apply_translation(BOARD_POS)
    return b

def b64(arr):
    return base64.b64encode(np.ascontiguousarray(arr).tobytes()).decode('ascii')
