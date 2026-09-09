#!/usr/bin/env python3
"""Posed CAD of the Atech Duck: the full-resolution Open Duck Mini v2 parts in the STAND pose
(first frame of the recorded stock run), Pi Zero + BNO055 removed, the Atech 14-port board in
the trunk, duck colours per part.

    python scripts/export_cad.py   -> cad/atech_duck_stand.glb (one node per part, vertex colours)
                                      out/atech_duck_stand.stl (single merged mesh, mm)
"""
import os, sys, json, numpy as np, mujoco, trimesh
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from duck_parts import ROOT, HR, load_parts, part_mesh, board_mesh, DUCK_COLOURS, ATECH_GREEN

def hex_rgba(h): return [(h >> 16) & 255, (h >> 8) & 255, h & 255, 255]

def stand_pose():
    """Body world poses (pos, quat wxyz) of every body in the STAND pose."""
    model = mujoco.MjModel.from_xml_path(os.path.join(HR, 'scene.xml'))
    data = mujoco.MjData(model)
    src = json.load(open(os.path.join(ROOT, 'out/motion/stock.json')))
    jadr = {a: model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, a)] for a in src['actuators']}
    data.qpos[:] = 0
    fr = src['frames'][0]; data.qpos[0:3] = [0, 0, fr[2]]; data.qpos[3:7] = [1, 0, 0, 0]   # upright, at the origin
    for a, v in zip(src['actuators'], src['q'][0]): data.qpos[jadr[a]] = v
    mujoco.mj_kinematics(model, data)
    return {model.body(b).name: (data.xpos[b].copy(), data.xquat[b].copy()) for b in range(model.nbody)}

def main():
    poses = stand_pose()
    parts = load_parts()
    scene = trimesh.Scene()
    merged = []
    counts = {}
    for p in parts:
        m = part_mesh(p)
        col = DUCK_COLOURS.get(p['key'], 0xF5C518)
        m.visual = trimesh.visual.ColorVisuals(m, vertex_colors=np.tile(hex_rgba(col), (len(m.vertices), 1)))
        pos, q = poses[p['body']]
        T = np.eye(4); T[:3, :3] = trimesh.transformations.quaternion_matrix(q)[:3, :3]; T[:3, 3] = pos
        counts[p['name']] = counts.get(p['name'], 0) + 1
        scene.add_geometry(m, node_name=f"{p['name']}_{counts[p['name']]}", geom_name=f"{p['name']}_{counts[p['name']]}", transform=T)
        mw = m.copy(); mw.apply_transform(T); merged.append(mw)
    b = board_mesh()
    b.visual = trimesh.visual.ColorVisuals(b, vertex_colors=np.tile(hex_rgba(ATECH_GREEN), (len(b.vertices), 1)))
    pos, q = poses['trunk_assembly']
    T = np.eye(4); T[:3, :3] = trimesh.transformations.quaternion_matrix(q)[:3, :3]; T[:3, 3] = pos
    scene.add_geometry(b, node_name='atech_board', geom_name='atech_board', transform=T)
    bw = b.copy(); bw.apply_transform(T); merged.append(bw)
    os.makedirs(os.path.join(ROOT, 'cad'), exist_ok=True)
    glb = os.path.join(ROOT, 'cad/atech_duck_stand.glb'); scene.export(glb)
    all_m = trimesh.util.concatenate(merged); all_m.apply_scale(1000)
    stl = os.path.join(ROOT, 'out/atech_duck_stand.stl'); all_m.export(stl)
    ext = all_m.bounding_box.extents
    print(f'{len(parts)} parts + board, {len(all_m.faces)} faces, bbox {ext[0]:.0f} x {ext[1]:.0f} x {ext[2]:.0f} mm')
    print(f'{glb} {os.path.getsize(glb)//1024} KB, {stl} {os.path.getsize(stl)//1024} KB')

if __name__ == '__main__':
    main()
