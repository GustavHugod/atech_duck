#!/usr/bin/env python3
"""Replay a recorded simulation on the high-resolution Open Duck Mini v2 model.

The Playground model used for physics carries 104-face convex hulls as visuals. The
onshape-to-robot export in reference/Open_Duck_Mini/mini_bdx/robots/open_duck_mini_v2
has the same kinematic tree with every real part (servos, horns, bearings, sheets,
Pi, cells, BMS). This script feeds the recorded base pose + 14 joint angles through
that model's forward kinematics and writes per-body world poses plus decimated
body-local meshes for the page.

    python scripts/replay_hires.py stock atech      -> out/motion/<name>_hires.json, out/motion/meshes_hires.json
"""
import os, sys, json, numpy as np, mujoco, trimesh
import xml.etree.ElementTree as ET
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from assemble_mjcf import tf, vec

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HR = os.path.join(ROOT, 'reference/Open_Duck_Mini/mini_bdx/robots/open_duck_mini_v2')
BOARD = os.path.expanduser('~/atech_code/public/modules/3d/motherboard_14_port.stl')
FACE_BUDGET = 140000

def export_meshes(model, board_pos=(-0.06, 0, 0.075)):
    root = ET.parse(os.path.join(HR, 'robot_motors.xml')).getroot()
    meshdir = HR
    files = {m.get('name'): os.path.join(meshdir, m.get('file')) for m in root.iter('mesh')}
    out = {}; raw = {}
    for body in root.iter('body'):
        parts = []
        for g in body.findall('geom'):
            if g.get('type') != 'mesh' or g.get('mesh') not in files: continue
            if g.get('contype') == '0' and g.get('conaffinity') == '0' and g.get('group') == '3': continue
            mm = trimesh.load(files[g.get('mesh')], force='mesh')
            mm.apply_transform(tf(vec(g.get('pos'), 3, [0, 0, 0]), vec(g.get('quat'), 4, [1, 0, 0, 0])))
            rgba = [float(v) for v in g.get('rgba').split()] if g.get('rgba') else [0.8, 0.8, 0.8, 1]
            parts.append((mm, rgba))
        if parts: raw[body.get('name')] = parts
    total = sum(len(mm.faces) for parts in raw.values() for mm, _ in parts)
    # Decimation policy: thin/small parts (sheets, brackets, horns, bearings) are kept whole —
    # they are the visible connections; only big shells are reduced, and never below 35 %.
    KEEP_WHOLE = 4000
    big = sum(len(mm.faces) for parts in raw.values() for mm, _ in parts if len(mm.faces) > KEEP_WHOLE)
    small = total - big
    ratio = max(0.35, min(1.0, (FACE_BUDGET - small) / max(big, 1)))
    for name, parts in raw.items():
        merged_parts = []
        for mm, rgba in parts:
            if len(mm.faces) > KEEP_WHOLE and ratio < 1.0:
                mm = mm.simplify_quadric_decimation(percent=1 - ratio)
            trimesh.repair.fix_normals(mm)
            mm.visual = trimesh.visual.ColorVisuals(mm, face_colors=[int(c * 255) for c in rgba])
            merged_parts.append(mm)
        merged = trimesh.util.concatenate(merged_parts)
        cols = merged.visual.face_colors[:, :3].astype(float) / 255.0
        out[name] = dict(v=np.round(merged.vertices, 5).ravel().tolist(), f=merged.faces.ravel().tolist(), c=np.round(cols, 3).ravel().tolist())
    b = trimesh.load(BOARD, force='mesh')
    b.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0])); b.apply_scale(0.001)
    b.apply_translation(-b.bounding_box.centroid); b.apply_translation(board_pos)
    out['atech_board'] = dict(v=np.round(b.vertices, 5).ravel().tolist(), f=b.faces.ravel().tolist(), c=[], parent='trunk_assembly')
    json.dump(out, open(os.path.join(ROOT, 'out/motion/meshes_hires.json'), 'w'))
    print(f'meshes: {len(out)} bodies, {total} -> {sum(len(v["f"])//3 for v in out.values())} faces')

def main(names):
    model = mujoco.MjModel.from_xml_path(os.path.join(HR, 'scene.xml'))
    data = mujoco.MjData(model)
    bodies = [model.body(b).name for b in range(1, model.nbody)]
    export_meshes(model)
    for name in names:
        src = json.load(open(os.path.join(ROOT, f'out/motion/{name}.json')))
        assert src['bodies'][0] == 'base'
        jadr = {a: model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, a)] for a in src['actuators']}
        frames = []
        for fr, q in zip(src['frames'], src['q']):
            data.qpos[:] = 0
            data.qpos[0:7] = fr[0:7]
            for a, v in zip(src['actuators'], q): data.qpos[jadr[a]] = v
            mujoco.mj_kinematics(model, data)
            pose = np.concatenate([np.concatenate([data.xpos[b], data.xquat[b]]) for b in range(1, model.nbody)])
            frames.append(np.round(pose, 5).tolist())
        out = dict(src); out['bodies'] = bodies; out['frames'] = frames
        json.dump(out, open(os.path.join(ROOT, f'out/motion/{name}_hires.json'), 'w'))
        print(f'{name}: {len(frames)} frames on {len(bodies)} bodies')

if __name__ == '__main__':
    main(sys.argv[1:] or ['stock', 'atech'])
