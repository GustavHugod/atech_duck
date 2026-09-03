#!/usr/bin/env python3
"""Pose a MuJoCo MJCF robot and export it as one STL/GLB plus an anatomy table.

    python scripts/assemble_mjcf.py <robot.xml> <out_name> [--keyframe home] [--scene scene.xml]

Walks <worldbody>, composes body pos/quat down the tree, applies every visual
mesh geom's pos/quat, and writes
    out/<out_name>.stl     merged, millimetres, Z-up
    out/<out_name>.glb     one node per body (colours from the MJCF materials)
    docs/anatomy_<out_name>.md   joints (world position, axis, range), link
                                 lengths between consecutive joints, masses.
Joint angles come from a keyframe if one is found (in the robot file or --scene),
otherwise the zero pose.
"""
import sys, os, math, argparse, json
import xml.etree.ElementTree as ET
import numpy as np
import trimesh

def quat_to_mat(q):
    w, x, y, z = q
    n = math.sqrt(w*w + x*x + y*y + z*z) or 1.0
    w, x, y, z = w/n, x/n, y/n, z/n
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]])

def tf(pos, quat):
    T = np.eye(4)
    T[:3, :3] = quat_to_mat(quat)
    T[:3, 3] = pos
    return T

def vec(s, n=3, default=None):
    if s is None:
        return np.array(default)
    return np.array([float(v) for v in s.split()][:n])

def axis_angle(axis, ang):
    axis = np.asarray(axis, float); axis /= (np.linalg.norm(axis) or 1)
    T = np.eye(4)
    T[:3, :3] = trimesh.transformations.rotation_matrix(ang, axis)[:3, :3]
    return T

def load_keyframe(xml_paths, name):
    for p in xml_paths:
        if not p or not os.path.exists(p):
            continue
        root = ET.parse(p).getroot()
        for key in root.iter('key'):
            if name is None or key.get('name') == name:
                return [float(v) for v in key.get('qpos').split()]
    return None

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('xml'); ap.add_argument('name')
    ap.add_argument('--keyframe', default='home')
    ap.add_argument('--scene', default=None)
    a = ap.parse_args()

    root = ET.parse(a.xml).getroot()
    xml_dir = os.path.dirname(os.path.abspath(a.xml))
    comp = root.find('compiler')
    meshdir = os.path.join(xml_dir, comp.get('meshdir', '.')) if comp is not None else xml_dir

    meshes = {}
    for m in root.iter('mesh'):
        f = m.get('file'); nm = m.get('name') or os.path.splitext(os.path.basename(f))[0]
        meshes[nm] = (os.path.join(meshdir, f), vec(m.get('scale'), 3, [1, 1, 1]))
    materials = {}
    for m in root.iter('material'):
        rgba = m.get('rgba')
        if rgba:
            materials[m.get('name')] = [float(v) for v in rgba.split()]

    qpos = load_keyframe([a.xml, a.scene], a.keyframe)
    joint_order = []
    for j in root.find('worldbody').iter('joint'):
        if j.get('type', 'hinge') in ('hinge', 'slide') and j.get('name'):
            joint_order.append(j.get('name'))
    joint_angle = {}
    if qpos:
        # first body has a freejoint (7 values) if the qpos is longer than the hinge count
        off = len(qpos) - len(joint_order)
        for i, jn in enumerate(joint_order):
            joint_angle[jn] = qpos[off + i]
    cache = {}
    def get_mesh(nm):
        if nm not in cache:
            path, scale = meshes[nm]
            mm = trimesh.load(path, force='mesh')
            mm.apply_scale(scale)
            cache[nm] = mm
        return cache[nm].copy()

    parts = []          # (body, mesh, rgba)
    joints = []         # dicts
    bodies = []         # (name, mass, world pos)
    def walk(body, T_parent, depth, parent_name):
        name = body.get('name', '?')
        T = T_parent @ tf(vec(body.get('pos'), 3, [0, 0, 0]), vec(body.get('quat'), 4, [1, 0, 0, 0]))
        for j in body.findall('joint'):
            if j.get('type') == 'free':
                continue
            jpos = vec(j.get('pos'), 3, [0, 0, 0]); jaxis = vec(j.get('axis'), 3, [0, 0, 1])
            ang = joint_angle.get(j.get('name'), 0.0)
            rng = j.get('range')
            wp = (T @ np.append(jpos, 1))[:3]; wa = T[:3, :3] @ jaxis
            joints.append(dict(name=j.get('name'), body=name, parent=parent_name, depth=depth,
                               pos=wp, axis=wa, range=[float(v) for v in rng.split()] if rng else None,
                               angle=ang))
            # rotate about the joint (hinge at jpos in body frame)
            T = T @ tf(jpos, [1, 0, 0, 0]) @ axis_angle(jaxis, ang) @ tf(-jpos, [1, 0, 0, 0])
        inert = body.find('inertial')
        mass = float(inert.get('mass')) if inert is not None else 0.0
        bodies.append((name, mass, T[:3, 3].copy(), depth))
        for g in body.findall('geom'):
            cls = g.get('class', '') or ''
            if g.get('type') != 'mesh' or 'collision' in cls:
                continue
            mm = get_mesh(g.get('mesh'))
            mm.apply_transform(T @ tf(vec(g.get('pos'), 3, [0, 0, 0]), vec(g.get('quat'), 4, [1, 0, 0, 0])))
            rgba = materials.get(g.get('material'), None)
            if rgba is None and g.get('rgba'):
                rgba = [float(v) for v in g.get('rgba').split()]
            parts.append((name, mm, rgba or [0.7, 0.7, 0.7, 1]))
        for child in body.findall('body'):
            walk(child, T, depth + 1, name)

    for b in root.find('worldbody').findall('body'):
        walk(b, np.eye(4), 0, None)

    merged = trimesh.util.concatenate([p[1] for p in parts])
    ext = merged.bounding_box.extents
    unit_scale = 1000.0 if ext.max() < 5 else 1.0     # metres -> mm
    os.makedirs('out', exist_ok=True); os.makedirs('docs', exist_ok=True)
    merged_mm = merged.copy(); merged_mm.apply_scale(unit_scale)
    merged_mm.export(f'out/{a.name}.stl')
    scene = trimesh.Scene()
    for i, (bname, mm, rgba) in enumerate(parts):
        mm = mm.copy(); mm.apply_scale(unit_scale)
        mm.visual = trimesh.visual.ColorVisuals(mm, face_colors=[int(c*255) for c in rgba])
        scene.add_geometry(mm, node_name=f'{bname}_{i}', geom_name=f'{bname}_{i}')
    scene.export(f'out/{a.name}.glb')

    zmin = merged_mm.bounds[0][2]
    total_mass = sum(b[1] for b in bodies)
    L = []
    L.append(f'# Anatomy — {a.name}\n')
    L.append(f'Source: `{os.path.relpath(a.xml)}`, pose = keyframe `{a.keyframe}`' + ('' if qpos else ' (not found → zero pose)') + '\n')
    e = merged_mm.bounding_box.extents
    L.append(f'| | |\n|---|---|\n| Bounding box (X depth × Y width × Z height) | {e[0]:.0f} × {e[1]:.0f} × {e[2]:.0f} mm |')
    L.append(f'| Standing height (ground → top) | {e[2]:.0f} mm |')
    L.append(f'| Total mass (sum of MJCF inertials) | {total_mass*1000:.0f} g |')
    L.append(f'| Bodies / joints / visual meshes | {len(bodies)} / {len(joints)} / {len(parts)} |\n')
    L.append('## Joints (world frame, standing pose, mm above ground)\n')
    L.append('| # | joint | body | height z | x | y | axis (world) | range | pose |\n|---|---|---|---|---|---|---|---|---|')
    for i, j in enumerate(joints):
        p = j['pos']*unit_scale; ax = j['axis']
        rng = f"{math.degrees(j['range'][0]):+.0f}…{math.degrees(j['range'][1]):+.0f}°" if j['range'] else '—'
        L.append(f"| {i} | {j['name']} | {j['body']} | {p[2]-zmin:.1f} | {p[0]:.1f} | {p[1]:.1f} | {ax[0]:+.2f} {ax[1]:+.2f} {ax[2]:+.2f} | {rng} | {math.degrees(j['angle']):+.1f}° |")
    L.append('\n## Link lengths (straight-line distance between consecutive joints in each chain)\n')
    L.append('| from | to | length mm |\n|---|---|---|')
    by_body = {j['body']: j for j in joints}
    for j in joints:
        par = by_body.get(j['parent'])
        # walk up until a body with a joint
        pb = j['parent']; hops = 0
        while pb is not None and pb not in by_body and hops < 10:
            pb = next((b[0] for b in [] ), None); hops += 1
        if par is not None:
            d = np.linalg.norm((j['pos'] - par['pos'])*unit_scale)
            L.append(f"| {par['name']} | {j['name']} | {d:.1f} |")
    L.append('\n## Masses per body (MJCF inertial)\n')
    L.append('| body | mass g | depth |\n|---|---|---|')
    for name, mass, pos, depth in sorted(bodies, key=lambda b: -b[1]):
        L.append(f'| {name} | {mass*1000:.1f} | {depth} |')
    open(f'docs/anatomy_{a.name}.md', 'w').write('\n'.join(L) + '\n')
    json.dump(dict(joints=[dict(name=j['name'], pos=(j['pos']*unit_scale).tolist(), axis=j['axis'].tolist(), range=j['range']) for j in joints],
                   bodies=[dict(name=b[0], mass_g=b[1]*1000) for b in bodies], extents_mm=e.tolist(), mass_g=total_mass*1000),
              open(f'out/{a.name}_anatomy.json', 'w'), indent=1)
    print(f'{a.name}: {len(parts)} meshes, {len(joints)} joints, bbox {e.round(0)} mm, mass {total_mass*1000:.0f} g, pose={"keyframe" if qpos else "zero"}')

if __name__ == '__main__':
    main()
