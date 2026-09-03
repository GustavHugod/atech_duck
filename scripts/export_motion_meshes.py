#!/usr/bin/env python3
"""Export per-body visual meshes (body-local frame, metres) of the Open Duck Mini v2 MJCF,
plus the Atech board placed in the trunk, as JSON for the motion page.

    python scripts/export_motion_meshes.py  -> out/motion/meshes.json
"""
import os, sys, json, numpy as np, trimesh
import xml.etree.ElementTree as ET
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from assemble_mjcf import quat_to_mat, tf, vec

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
XML = os.path.join(ROOT, 'reference/Open_Duck_Playground/playground/open_duck_mini_v2/xmls/open_duck_mini_v2.xml')
BOARD = os.path.expanduser('~/atech_code/public/modules/3d/motherboard_14_port.stl')

def main(board_pos=(-0.06, 0, 0.075)):
    root = ET.parse(XML).getroot()
    meshdir = os.path.join(os.path.dirname(XML), root.find('compiler').get('meshdir', '.'))
    meshes = {}
    for m in root.iter('mesh'):
        nm = m.get('name') or os.path.splitext(os.path.basename(m.get('file')))[0]
        meshes[nm] = (os.path.join(meshdir, m.get('file')), vec(m.get('scale'), 3, [1, 1, 1]))
    materials = {m.get('name'): [float(v) for v in m.get('rgba').split()] for m in root.iter('material') if m.get('rgba')}
    out = {}
    for body in root.iter('body'):
        parts = []
        for g in body.findall('geom'):
            cls = g.get('class', '') or ''
            if g.get('type') != 'mesh' or 'collision' in cls: continue
            path, scale = meshes[g.get('mesh')]
            mm = trimesh.load(path, force='mesh'); mm.apply_scale(scale)
            mm.apply_transform(tf(vec(g.get('pos'), 3, [0, 0, 0]), vec(g.get('quat'), 4, [1, 0, 0, 0])))
            rgba = materials.get(g.get('material')) or ([float(v) for v in g.get('rgba').split()] if g.get('rgba') else [0.8, 0.8, 0.8, 1])
            mm.visual = trimesh.visual.ColorVisuals(mm, face_colors=[int(c * 255) for c in rgba])
            parts.append(mm)
        if not parts: continue
        merged = trimesh.util.concatenate(parts)
        cols = merged.visual.face_colors[:, :3].astype(float) / 255.0
        out[body.get('name')] = dict(v=np.round(merged.vertices, 5).ravel().tolist(), f=merged.faces.ravel().tolist(),
                                     c=np.round(cols, 3).ravel().tolist())
    # Atech board: STL is Y-up, mm -> Z-up metres, long axis along y (trunk width), assumed position
    b = trimesh.load(BOARD, force='mesh')
    b.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0]))   # (x,y,z)->(x,-z,y)
    b.apply_scale(0.001)
    b.apply_translation(-b.bounding_box.centroid)
    b.apply_translation(board_pos)
    out['atech_board'] = dict(v=np.round(b.vertices, 5).ravel().tolist(), f=b.faces.ravel().tolist(), c=[], parent='trunk_assembly')
    tri = sum(len(v['f']) // 3 for v in out.values())
    json.dump(out, open(os.path.join(ROOT, 'out/motion/meshes.json'), 'w'))
    print(f'{len(out)} bodies, {tri} triangles -> out/motion/meshes.json ({os.path.getsize(os.path.join(ROOT, "out/motion/meshes.json"))//1024} KB)')

if __name__ == '__main__':
    main()
