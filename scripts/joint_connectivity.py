#!/usr/bin/env python3
"""Objective 'is it connected' check: for every joint, the minimum distance between the
child body's meshes and the parent body's meshes over sample frames. A mechanical joint
(horn on a servo, shaft in a bearing) shows up as overlap or contact (≤ ~0.5 mm); a real
gap is a modelling error.

    python scripts/joint_connectivity.py [stock] [frames...]
"""
import sys, json, numpy as np, trimesh, mujoco
HR = 'reference/Open_Duck_Mini/mini_bdx/robots/open_duck_mini_v2/scene.xml'

def main(name='stock', frames=(0, 520)):
    M = json.load(open('out/motion/meshes_hires.json')); R = json.load(open(f'out/motion/{name}_hires.json'))
    bi = {b: i for i, b in enumerate(R['bodies'])}
    model = mujoco.MjModel.from_xml_path(HR)
    parent = {model.body(b).name: model.body(model.body_parentid[b]).name for b in range(1, model.nbody)}
    joints = [(model.jnt(j).name, model.body(model.jnt_bodyid[j]).name) for j in range(model.njnt) if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE]
    def world(bname, fr):
        md = M.get(bname)
        if md is None: return None
        i = bi[bname]; o = i * 7; T = trimesh.transformations.quaternion_matrix(fr[o + 3:o + 7]); T[:3, 3] = fr[o:o + 3]
        m = trimesh.Trimesh(np.array(md['v']).reshape(-1, 3), np.array(md['f']).reshape(-1, 3), process=False); m.apply_transform(T); return m
    print(f'{"joint":18s} {"child":28s} {"parent":28s} ' + ' '.join(f'f{f:>4d}' for f in frames) + '   (min gap mm, 0 = touching/overlapping)')
    worst = 0
    for jn, child in joints:
        par = parent[child]
        while par not in M and par in parent: par = parent[par]      # skip mesh-less bodies
        row = []
        for f in frames:
            fr = R['frames'][f]; a = world(child, fr); b = world(par, fr)
            if a is None or b is None: row.append(float('nan')); continue
            pts, _ = trimesh.sample.sample_surface(a, 400)
            _, dist, _ = trimesh.proximity.closest_point(b, pts)      # unsigned distance to the parent's surface
            inside = b.contains(pts) if b.is_watertight else np.zeros(len(pts), bool)
            gap = 0.0 if inside.any() else float(dist.min()) * 1000
            row.append(gap)
        worst = max(worst, max(r for r in row if r == r))
        print(f'{jn:18s} {child:28s} {par:28s} ' + ' '.join(f'{r:5.1f}' for r in row))
    print('worst gap over checked joints/frames: %.1f mm' % worst)

if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else 'stock')
