#!/usr/bin/env python3
"""Headless MuJoCo motion simulation of the Open Duck Mini v2 body running its trained
walking policy, optionally with the Atech board added to the trunk.

    python scripts/sim_walk.py --name stock
    python scripts/sim_walk.py --name atech --board-mass 0.06

Replicates reference/Open_Duck_Playground/.../mujoco_infer.py (50 Hz control, sim dt 2 ms,
101-D observation, action_scale 0.25, motor speed limit 5.24 rad/s, gait phase from the
polynomial reference motion) without the viewer, drives a scripted command schedule, and
records per control step: every body's world pose, joint angles, actuator torques, feet
contacts, base velocity. Output: out/motion/<name>.json + a stats summary printed and saved.
"""
import os, sys, json, argparse, pickle, math
import numpy as np
import mujoco, onnxruntime

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PG = os.path.join(ROOT, 'reference/Open_Duck_Playground/playground/open_duck_mini_v2')
XML_DIR = os.path.join(PG, 'xmls')
ONNX = os.path.join(ROOT, 'reference/Open_Duck_Mini/BEST_WALK_ONNX_2.onnx')
POLY = os.path.join(PG, 'data/polynomial_coefficients.pkl')

# command schedule: (t_start, vx, vy, wz, neck_pitch, head_pitch, head_yaw, head_roll, label)
SCHEDULE = [
    (0.0,  0.00, 0.00, 0.0, 0, 0, 0, 0, 'stand'),
    (1.5,  0.15, 0.00, 0.0, 0, 0, 0, 0, 'walk forward 0.15 m/s'),
    (6.5,  0.05, 0.00, 1.0, 0, 0, 0, 0, 'turn 1.0 rad/s'),
    (9.5,  0.00, 0.20, 0.0, 0, 0, 0, 0, 'sidestep 0.20 m/s'),
    (12.5, 0.00, 0.00, 0.0, 0.3, -0.3, 0.8, 0, 'stand, look up-left'),
    (14.5, -0.15, 0.00, 0.0, 0, 0, 0, 0, 'walk backward 0.15 m/s'),
    (17.5, 0.00, 0.00, 0.0, 0, 0, 0, 0, 'stand'),
]
# The policy treats |vx| < ~0.1 and |vy| < ~0.15 m/s as "stand" (diagnosed 2026-09-02).
T_END = 19.0

def command_at(t):
    cur = SCHEDULE[0]
    for row in SCHEDULE:
        if t >= row[0]: cur = row
    return list(cur[1:8]), cur[8]

def merge_inertial(inertial_tag, mb, pb, size):
    """Add a box of mass mb at pb (body frame) to an MJCF <inertial pos mass fullinertia/> tag."""
    import re
    def attr(n): return re.search(n + r'="([^"]+)"', inertial_tag).group(1)
    m = float(attr('mass')); p = np.array([float(v) for v in attr('pos').split()])
    I = [float(v) for v in attr('fullinertia').split()]        # Ixx Iyy Izz Ixy Ixz Iyz about the CoM
    Im = np.array([[I[0], I[3], I[4]], [I[3], I[1], I[5]], [I[4], I[5], I[2]]])
    pb = np.array(pb); sx, sy, sz = size
    Ib = mb / 12.0 * np.diag([sy*sy + sz*sz, sx*sx + sz*sz, sx*sx + sy*sy])
    mt = m + mb; pt = (m * p + mb * pb) / mt
    def shift(Ic, mass, d):
        d = np.asarray(d); return Ic + mass * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    It = shift(Im, m, p - pt) + shift(Ib, mb, pb - pt)
    fi = f'{It[0,0]:.6g} {It[1,1]:.6g} {It[2,2]:.6g} {It[0,1]:.6g} {It[0,2]:.6g} {It[1,2]:.6g}'
    out = re.sub(r'mass="[^"]+"', f'mass="{mt:.6g}"', inertial_tag)
    out = re.sub(r'pos="[^"]+"', f'pos="{pt[0]:.6g} {pt[1]:.6g} {pt[2]:.6g}"', out, count=1)
    out = re.sub(r'fullinertia="[^"]+"', f'fullinertia="{fi}"', out)
    return out, mt, pt

def build_model(board_mass, board_pos, board_size, tag='run'):
    xml = open(os.path.join(XML_DIR, 'scene_flat_terrain.xml')).read()
    if board_mass > 0:
        robot = open(os.path.join(XML_DIR, 'open_duck_mini_v2.xml')).read()
        i = robot.index('<body name="trunk_assembly"')
        k = robot.index('<inertial', i); k2 = robot.index('/>', k) + 2
        merged, mt, pt = merge_inertial(robot[k:k2], board_mass, board_pos, board_size)
        geom = (f'<geom name="atech_board" type="box" size="{board_size[0]/2} {board_size[1]/2} {board_size[2]/2}" '
                f'pos="{board_pos[0]} {board_pos[1]} {board_pos[2]}" rgba="0.16 0.45 0.30 1" contype="0" conaffinity="0" group="1"/>\n        ')
        robot = robot[:k] + geom + merged + robot[k2:]
        print(f'trunk_assembly inertial: +{board_mass*1000:.0f} g -> {mt*1000:.0f} g, CoM {np.round(pt*1000,1).tolist()} mm')
        tmp = os.path.join(XML_DIR, f'_open_duck_mini_v2_{tag}.xml')
        open(tmp, 'w').write(robot)
        xml = xml.replace('open_duck_mini_v2.xml', f'_open_duck_mini_v2_{tag}.xml')
    tmp_scene = os.path.join(XML_DIR, f'_scene_{tag}.xml')
    open(tmp_scene, 'w').write(xml)
    return mujoco.MjModel.from_xml_path(tmp_scene)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--name', default='stock')
    ap.add_argument('--board-mass', type=float, default=0.0, help='kg; 0 = stock body')
    ap.add_argument('--board-pos', default='-0.06,0,0.075', help='m, trunk frame (assumed placement)')
    ap.add_argument('--board-size', default='0.06,0.12,0.007')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--schedule', default=None, help='JSON list of [t, vx, vy, wz, np, hp, hy, hr, label]')
    ap.add_argument('--t-end', type=float, default=None)
    a = ap.parse_args()
    global SCHEDULE, T_END
    if a.schedule: SCHEDULE = [tuple(r) for r in json.loads(a.schedule)]
    if a.t_end: T_END = a.t_end
    board_pos = [float(v) for v in a.board_pos.split(',')]; board_size = [float(v) for v in a.board_size.split(',')]

    model = build_model(a.board_mass, board_pos, board_size, tag=a.name)
    sim_dt, decimation = 0.002, 10
    model.opt.timestep = sim_dt
    data = mujoco.MjData(model)
    key = model.keyframe('home')
    data.qpos[:] = key.qpos; data.ctrl[:] = key.ctrl
    mujoco.mj_forward(model, data)

    nu = model.nu
    act_names = [model.actuator(k).name for k in range(nu)]
    act_joint = [model.actuator_trnid[k][0] for k in range(nu)]
    qpos_adr = np.array([model.jnt_qposadr[j] for j in act_joint])
    qvel_adr = np.array([model.jnt_dofadr[j] for j in act_joint])
    default_act = key.ctrl.copy()
    gyro_adr = model.sensor_adr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, 'gyro')]
    acc_adr = model.sensor_adr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, 'accelerometer')]
    lv_adr = model.sensor_adr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, 'local_linvel')]
    floor_geom = next(g for g in range(model.ngeom) if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_PLANE)
    foot_bodies = {mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'foot_assembly'): 0,
                   mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'foot_assembly_2'): 1}
    def feet_contacts():
        c = [0.0, 0.0]
        for i in range(data.ncon):
            con = data.contact[i]; g1, g2 = con.geom1, con.geom2
            if floor_geom not in (g1, g2): continue
            other = g2 if g1 == floor_geom else g1
            b = model.geom_bodyid[other]
            # walk up to a foot body
            while b > 0 and b not in foot_bodies: b = model.body_parentid[b]
            if b in foot_bodies: c[foot_bodies[b]] = 1.0
        return c

    poly = pickle.load(open(POLY, 'rb'))
    k0 = list(poly.keys())[0]
    nb_steps = int(poly[k0]['period'] * poly[k0]['fps'])
    sess = onnxruntime.InferenceSession(ONNX, providers=['CPUExecutionProvider'])
    in_name = sess.get_inputs()[0].name

    last = [np.zeros(nu) for _ in range(3)]
    motor_targets = default_act.copy(); prev_targets = default_act.copy()
    imitation_i = 0.0; max_motor_vel = 5.24
    frames = []; torques = []; qs = []; feet = []; base = []; cmds = []; labels = []; contacts_log = []
    body_ids = list(range(1, model.nbody))
    torque_peak = np.zeros(nu); torque_rms = np.zeros(nu); n_ctrl = 0
    fell_at = None
    steps = int(T_END / sim_dt)
    for step in range(steps):
        t = step * sim_dt
        if step % decimation == 0:
            cmd, label = command_at(t)
            walking = True
            imitation_i = (imitation_i + 1.0) % nb_steps
            phase = np.array([math.cos(imitation_i / nb_steps * 2 * math.pi), math.sin(imitation_i / nb_steps * 2 * math.pi)])
            gyro = data.sensordata[gyro_adr:gyro_adr + 3].copy()
            acc = data.sensordata[acc_adr:acc_adr + 3].copy(); acc[0] += 1.3
            q = data.qpos[qpos_adr].copy(); qd = data.qvel[qvel_adr].copy()
            fc = feet_contacts()
            obs = np.concatenate([gyro, acc, cmd, q - default_act, qd * 0.05, last[0], last[1], last[2], motor_targets, fc, phase]).astype(np.float32)
            action = sess.run(None, {in_name: [obs]})[0][0]
            last[2] = last[1].copy(); last[1] = last[0].copy(); last[0] = action.copy()
            motor_targets = default_act + action * 0.25
            lim = max_motor_vel * sim_dt * decimation
            motor_targets = np.clip(motor_targets, prev_targets - lim, prev_targets + lim)
            prev_targets = motor_targets.copy()
            data.ctrl[:] = motor_targets
            # record
            tau = data.actuator_force.copy()
            torque_peak = np.maximum(torque_peak, np.abs(tau)); torque_rms += tau**2; n_ctrl += 1
            pose = np.concatenate([np.concatenate([data.xpos[b], data.xquat[b]]) for b in body_ids])
            frames.append(np.round(pose, 5).tolist())
            torques.append(np.round(tau, 3).tolist())
            qs.append(np.round(q, 4).tolist())
            feet.append(fc); cmds.append(cmd); labels.append(label)
            root_z = data.qpos[2]; vel = data.sensordata[lv_adr:lv_adr + 3].copy()   # body frame (velocimeter at the IMU site)
            base.append([round(float(root_z), 4), round(float(vel[0]), 3), round(float(vel[1]), 3), round(float(vel[2]), 3)])
            if fell_at is None and root_z < 0.12: fell_at = t
        mujoco.mj_step(model, data)

    torque_rms = np.sqrt(torque_rms / max(n_ctrl, 1))
    # per-segment speed stats
    seg = []
    for i, lab in enumerate(labels):
        if seg and seg[-1][0] == lab: seg[-1][1].append(i)
        else: seg.append((lab, [i]))
    speeds = {}
    for lab, idx in seg:
        if lab in speeds: lab = lab + ' (end)'
        idx = idx[len(idx)//3:]  # settle
        vx = np.mean([base[i][1] for i in idx]); vy = np.mean([base[i][2] for i in idx])
        yaws = [math.atan2(2*(frames[i][3]*frames[i][6]+frames[i][4]*frames[i][5]), 1-2*(frames[i][5]**2+frames[i][6]**2)) for i in idx]
        wz = float(np.unwrap(yaws)[-1] - np.unwrap(yaws)[0]) / (len(idx) * sim_dt * decimation)
        speeds[lab] = dict(vx=round(float(vx), 3), vy=round(float(vy), 3), wz=round(wz, 3), height=round(float(np.mean([base[i][0] for i in idx])), 3))
    stats = dict(name=a.name, board_mass_kg=a.board_mass, board_pos=board_pos if a.board_mass > 0 else None,
                 duration_s=T_END, control_hz=50, sim_dt=sim_dt, frames=len(frames), fell_at_s=fell_at,
                 final_height_m=base[-1][0], distance_m=round(float(np.linalg.norm(np.array(frames[-1][:2]) - np.array(frames[0][:2]))), 3),
                 torque_peak_Nm=dict(zip(act_names, np.round(torque_peak, 3).tolist())),
                 torque_rms_Nm=dict(zip(act_names, np.round(torque_rms, 3).tolist())),
                 torque_limit_Nm=float(model.actuator_forcerange[0][1]), speeds=speeds, gait_period_steps=nb_steps)
    out = dict(name=a.name, dt=sim_dt * decimation, bodies=[model.body(b).name for b in body_ids], actuators=act_names,
               frames=frames, torques=torques, q=qs, feet=feet, cmd=cmds, label=labels, base=base, stats=stats,
               schedule=[dict(t=r[0], label=r[8]) for r in SCHEDULE])
    os.makedirs(os.path.join(ROOT, 'out/motion'), exist_ok=True)
    json.dump(out, open(os.path.join(ROOT, f'out/motion/{a.name}.json'), 'w'))
    json.dump(stats, open(os.path.join(ROOT, f'out/motion/{a.name}_stats.json'), 'w'), indent=1)
    print(json.dumps(stats, indent=1))

if __name__ == '__main__':
    main()
