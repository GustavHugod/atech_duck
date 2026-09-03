"""Controller factory for `partsmith motion` (FRAMEWORK.md §4): the Open Duck Mini v2
walking policy with the same observation contract as scripts/sim_walk.py.

    motion:
      controller: python:~/projects/atech_duck/scripts/duck_controller.py:make_controller
      policy: ~/projects/atech_duck/reference/Open_Duck_Mini/BEST_WALK_ONNX_2.onnx
      schedule: [[0, 0, 0, 0, 0, 0, 0, 0, "stand"], [1.5, 0.15, 0, 0, 0, 0, 0, 0, "forward"]]
"""
import os, math, pickle
import numpy as np


def make_controller(model, data, cfg):
    import mujoco, onnxruntime
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    policy = os.path.expanduser(cfg.get('policy', os.path.join(root, 'reference/Open_Duck_Mini/BEST_WALK_ONNX_2.onnx')))
    poly = os.path.expanduser(cfg.get('reference_motion', os.path.join(root, 'reference/Open_Duck_Playground/playground/open_duck_mini_v2/data/polynomial_coefficients.pkl')))
    schedule = [tuple(r) for r in cfg.get('schedule') or [(0, 0, 0, 0, 0, 0, 0, 0, 'stand'), (1.5, 0.15, 0, 0, 0, 0, 0, 0, 'forward')]]
    sess = onnxruntime.InferenceSession(policy, providers=['CPUExecutionProvider']); in_name = sess.get_inputs()[0].name
    pk = pickle.load(open(poly, 'rb')); k0 = list(pk.keys())[0]; nb = int(pk[k0]['period'] * pk[k0]['fps'])
    nu = model.nu
    key = model.keyframe(cfg.get('keyframe', 'home')); default = key.ctrl.copy()
    act_joint = [model.actuator_trnid[k][0] for k in range(nu)]
    qadr = np.array([model.jnt_qposadr[j] for j in act_joint]); vadr = np.array([model.jnt_dofadr[j] for j in act_joint])
    gyro = model.sensor_adr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, 'gyro')]
    acc = model.sensor_adr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, 'accelerometer')]
    floor = next(g for g in range(model.ngeom) if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_PLANE)
    feet = {mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'foot_assembly'): 0, mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'foot_assembly_2'): 1}
    st = dict(last=[np.zeros(nu) for _ in range(3)], targets=default.copy(), prev=default.copy(), i=0.0)
    dt_ctrl = 1.0 / float(cfg.get('control_hz', 50)); vmax = 5.24

    def contacts():
        c = [0.0, 0.0]
        for i in range(data.ncon):
            con = data.contact[i]
            if floor not in (con.geom1, con.geom2): continue
            b = model.geom_bodyid[con.geom2 if con.geom1 == floor else con.geom1]
            while b > 0 and b not in feet: b = model.body_parentid[b]
            if b in feet: c[feet[b]] = 1.0
        return c

    def command(t):
        cur = schedule[0]
        for r in schedule:
            if t >= r[0]: cur = r
        return list(cur[1:8])

    def controller(model, data, t):
        st['i'] = (st['i'] + 1.0) % nb
        phase = [math.cos(st['i'] / nb * 2 * math.pi), math.sin(st['i'] / nb * 2 * math.pi)]
        g = data.sensordata[gyro:gyro + 3].copy(); a = data.sensordata[acc:acc + 3].copy(); a[0] += 1.3
        q = data.qpos[qadr]; qd = data.qvel[vadr]
        obs = np.concatenate([g, a, command(t), q - default, qd * 0.05, st['last'][0], st['last'][1], st['last'][2], st['targets'], contacts(), phase]).astype(np.float32)
        act = sess.run(None, {in_name: [obs]})[0][0]
        st['last'] = [act.copy(), st['last'][0], st['last'][1]]
        tg = np.clip(default + act * 0.25, st['prev'] - vmax * dt_ctrl, st['prev'] + vmax * dt_ctrl)
        st['prev'] = tg.copy(); st['targets'] = tg.copy()
        return tg
    return controller
