# Motion simulation — Open Duck Mini v2 body under its walking policy (2026-09-02)

`scripts/sim_walk.py` runs the trained policy `BEST_WALK_ONNX_2.onnx` headless in MuJoCo on
the Playground model (STS3215 position actuators, ±3.23 N·m, 2 ms physics, 50 Hz control,
the 101-value observation the real runtime builds). Two variants:

| | stock body | + Atech board (60 g at (−60, 0, 75) mm in the trunk, merged into the inertial) |
|---|---|---|
| fell over in 19 s | no | no |
| distance covered | 0.297 m | 0.274 m |
| forward, commanded 0.15 m/s | 0.096 m/s | 0.091 m/s |
| turn, commanded 1.0 rad/s | 0.75 rad/s | 0.76 rad/s |
| sidestep, commanded 0.20 m/s | 0.110 m/s | 0.105 m/s |
| backward, commanded 0.15 m/s | 0.039 m/s | 0.039 m/s |
| trunk height | 163–167 mm | same |
| knee torque peak / RMS | 2.77–3.23 (clip) / 1.21 N·m | 2.76–3.23 / 1.21 N·m |

Findings:
- The policy has a command dead zone: forward below about 0.10 m/s and sideways below
  0.15 m/s are treated as "stand" (diagnosed with isolated commands). The schedule uses
  0.15 / 0.20 / 1.0 for that reason.
- Achieved speeds are 60–75 % of commanded, backward only 25 %. That is the policy, not
  the Atech change.
- The 60 g board costs ~5 % forward speed and nothing in stability. Mass and placement
  are assumptions until weighed and fitted (DESIGN.md Q1, Q2).
- Knees are the hot servos: RMS 1.2 N·m of the STS3215's 3.23 N·m, peaks on the clip.
  Hips and ankles stay under 0.7 N·m RMS.

Outputs: `out/motion/{stock,atech}.json` (per 20 ms: every body pose, 14 torques, joint
angles, feet contacts, body-frame velocity), `out/motion/*_stats.json`, meshes for the
page `out/motion/meshes.json` (`scripts/export_motion_meshes.py`), page
`scripts/build_motion_page.py` → `docs/motion_page.html`.

## Visual check of the replay (2026-09-02 night)

Gustav: the head and feet "do not look connected". Checked three ways:
1. Body world poses and joint axes of the physics model (Playground) and the display
   model (onshape-to-robot export) agree to < 0.5 mm / 1° at every frame tested
   (`scripts/replay_hires.py` feeds the same joint angles into both).
2. The page's embedded data places the head shell from z 325 to 414 mm with the yaw
   assembly (332–383 mm) inside it, matching MuJoCo's own compile.
3. `scripts/joint_connectivity.py` measures the minimum distance between child and
   parent meshes at every joint over four frames (table in the run log).
The exposed neck (two plates, a servo, a bracket, then the head) and the visible ankle
blocks are the real Open Duck Mini v2 design — see `docs/img/odm_v2_photo_readme.png`
(photo from the ODM v2 README). What was wrong: uniform 15 % decimation had thinned the
sheets and brackets that make the connections visible. The export now keeps every part
under 4000 faces whole and decimates only the shells (min 35 %), and the page renders
both faces of every triangle.
