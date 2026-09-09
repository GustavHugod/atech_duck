#!/usr/bin/env python3
"""Replay a recorded simulation on the high-resolution Open Duck Mini v2 model.

The Playground model used for physics carries 104-face convex hulls as visuals. The
onshape-to-robot export in reference/Open_Duck_Mini/mini_bdx/robots/open_duck_mini_v2
has the same kinematic tree with every real part (servos, horns, bearings, sheets,
Pi, cells, BMS). This script feeds the recorded base pose + 14 joint angles through
that model's forward kinematics and writes per-body world poses plus full-resolution
body-local meshes (binary, base64) for the page.

    python scripts/replay_hires.py stock atech      -> out/motion/<name>_hires.json, out/motion/meshes_hires.json
"""
import os, sys, json, numpy as np, mujoco, trimesh
import xml.etree.ElementTree as ET
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from assemble_mjcf import tf, vec
from duck_parts import load_parts, part_mesh, board_mesh, b64

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HR = os.path.join(ROOT, 'reference/Open_Duck_Mini/mini_bdx/robots/open_duck_mini_v2')

def export_meshes(model):
    """Full-resolution meshes per body for the page (no decimation): vertices float32, faces
    uint16/uint32, one CAD-colour index per face, all base64 -> out/motion/meshes_hires.json."""
    parts = load_parts()
    bodies = {}
    for p in parts: bodies.setdefault(p['body'], []).append(p)
    out = {}; total = 0
    for name, plist in bodies.items():
        keys = []; merged = []; face_key = []
        for p in plist:
            m = part_mesh(p)
            if p['key'] not in keys: keys.append(p['key'])
            face_key.append(np.full(len(m.faces), keys.index(p['key']), dtype=np.uint8))
            merged.append(m)
        mm = trimesh.util.concatenate(merged)
        f = mm.faces.ravel(); small = len(mm.vertices) < 65535
        out[name] = dict(vb=b64(mm.vertices.astype(np.float32)), fb=b64(f.astype(np.uint16 if small else np.uint32)),
                         f16=small, pb=b64(np.concatenate(face_key)), keys=keys)
        total += len(mm.faces)
    b = board_mesh()
    out['atech_board'] = dict(vb=b64(b.vertices.astype(np.float32)), fb=b64(b.faces.ravel().astype(np.uint32)), f16=False,
                              pb='', keys=[], parent='trunk_assembly')
    total += len(b.faces)
    path = os.path.join(ROOT, 'out/motion/meshes_hires.json')
    json.dump(out, open(path, 'w'), separators=(',', ':'))
    print(f'meshes: {len(out)} bodies, {total} faces, full resolution -> {path} ({os.path.getsize(path)//1024} KB)')

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
