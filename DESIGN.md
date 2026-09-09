# Atech Duck — recreating the Hugging Face / Pollen "Microduck" on the Atech platform

Status 2026-09-02: **CONCEPT + digital groundwork. Nothing bought, printed or flashed.**
Author: Claude (session gustavhugod-86), brief for Gustav.

## 0. The short version

- The Microduck is a 25 cm, 737 g biped: 15 Dynamixel XL330 bus servos, a Linux SoC, an
  8×8 ToF, two IMUs, camera, mic, speaker, and RL policies at 50 Hz.
- There is **no 3D scan** of it and Pollen has not opened the CAD, but the **official
  simulation meshes are public** (43 STLs, every joint origin/axis/range/mass in the MJCF,
  CC BY-NC-SA). I fetched them, posed the duck in its STAND keyframe and exported a
  single STL/GLB plus an anatomy table (`out/`, `docs/anatomy_microduck.md`).
- The Atech platform already speaks the one thing that matters for a duck: **a Feetech
  STS bus-servo chain** (the Robot Arm module: half-duplex UART, 1 Mbps, STS3215). It also
  has the IMU, the very same VL53L5CX 8×8 ToF, a MAX98357A speaker, a mic and NeoPixels.
- The Microduck's own STS3215 sibling already exists and is fully open: **Open Duck Mini
  v2** (same engineer, Apache-2.0, 129 print STLs, BOM, assembly guide, trained walking
  policies). Its electronics are a Pi Zero 2 W + a bus adapter + a BNO055 — exactly the
  three things the Atech board replaces.
- **Recommendation: build the Open Duck Mini v2 body unchanged, put the Atech 14-port
  board in the trunk as the brain, and add the Microduck features it lacks (ToF, eyes,
  sound) as Atech modules.** The existing ODM v2 walking policy runs on the ESP32-S3
  as-is (firmware written and compiled today; 101-D obs → 14-D action MLP, 218 k params).
- A 1:1 Microduck (XL330, 25 cm) is the *less* Atech-shaped route: the board does not fit
  in an 82 × 64 mm trunk, the servos need a new Dynamixel-2.0 driver, and the meshes are
  non-commercial. Kept as route B below.

## 1. What the Microduck is (from the sources in `reference/`)

| | Microduck | source |
|---|---|---|
| Size / mass | 25 cm tall, 14 cm wide, 737 g in the MJCF, "< 800 g" | press kit, `robot_walk.xml` |
| Actuators | 15 × Dynamixel XL330-M288-T (18 g, 288:1, calibrated ±0.96 N·m); 14 in the policy, #15 = beak | MJCF, teardown |
| Joints | per leg hip_yaw, hip_roll, hip_pitch, knee, ankle; neck_pitch, head_pitch, head_yaw, head_roll | MJCF |
| Link lengths (joint to joint) | hip_yaw→roll 20.7, roll→pitch 31.1, hip→knee 42.2, knee→ankle 49.4 mm | `docs/anatomy_microduck.md` |
| Compute | Radxa Zero 3W (RK3566, 4× A55, NPU), Armbian, Rust runtime | teardown |
| Servo bus | Dynamixel Protocol 2.0, 1 Mbps, TTL half-duplex; IDs L 20-24 / head 30-34 / R 10-14; IMU as bus slave ID 200 | teardown |
| Sensors | VL53L5CX/L8CX 8×8 ToF @15 Hz; LSM6DSV16X body IMU (on the bus); head IMU; IMX219 camera; mic + speaker (TLV320AIC3104); NFC ×2 | teardown, press kit |
| Battery | NP-F550 2600 mAh 2S (6.6–8.2 V under load), ~1 h, no fuel gauge (servos report Vin) | teardown |
| Policies | 9 ONNX, `obs[61] → act[14]`, MLP 61→512→256→128→14 ELU, 198 k params, 50 Hz | `policies/`, this repo's exporter |
| Fasteners | an M2 system (Ø2.2 clearance + Ø4.4 counterbore + Ø1.6 tapping) | fastener study |
| Licence | code Apache-2.0; **meshes CC BY-NC-SA 4.0**; hardware not open (no BOM/PCB) | repos |

Look: `docs/img/reference_lineup.png` (Microduck, Open Duck Mini v2 and the Atech
board at one scale), `reference/press/*.jpg`.

## 2. Reference geometry — what exists instead of a scan

`reference/README.md` lists every source with its licence. In one line each:

1. **Official meshes + MJCF** (`microduck_rl`): the real geometry, exported from Pollen's
   Onshape. `scripts/assemble_mjcf.py` walks the body tree, applies the STAND keyframe
   and writes `out/microduck.stl` (40 MB, mm, Z-up) and `.glb` (coloured per part).
2. **Community reconstructions**: `microduck-hardware-replica` (70 STL occurrences already
   in world coordinates, FreeCAD assembly), `microduck-replica` (15 per-assembly STLs,
   electronics teardown from the Rust source, fastener study), `microduck-3d` (combined
   GLB + `kinematics.json`).
3. **Open Duck Mini v2** (`Open_Duck_Mini/print/`): 129 printable STLs, Onshape, BOM,
   assembly + wiring guides. Posed from the Playground MJCF →
   `out/open_duck_mini_v2.stl`, `docs/anatomy_open_duck_mini_v2.md`.

| | Microduck | Open Duck Mini v2 |
|---|---|---|
| height / mass | 272 mm / 737 g | 489 mm / 2107 g |
| servo | XL330 18 g, 0.96 N·m | STS3215 55 g, 3.35 N·m (Playground calibration) |
| hip→knee / knee→ankle | 42 / 49 mm | 79 / 79 mm |
| neck→head_pitch→yaw→roll | 50 / 24 / 23 mm | 66 / 60 / 45 mm |
| trunk outer | ~82 × 64 × 36 mm | body_middle 150 × 110 × (86 + 57) mm |
| electronics | Radxa + HAT + IMU board | Pi Zero 2 W + Waveshare bus adapter + BNO055 |

The community actuator study (`microduck-replica/docs/actuator-selection.en.md`) makes
the point that decides route A vs B: swapping XL330 → STS3215 is not a servo swap, it is
a different robot (3× the servo mass, different cavities, different actuator model). ODM
v2 *is* that robot, already designed, printed by many and walking.

## 3. What the Atech platform brings

14-port board (ESP32-S3, 8 MB flash), 12 module slots, each port = Line A + Line B GPIO +
GND/3V3/5V/12V. Relevant modules (`~/atech_code/backend/modules/my_modules.yaml`):

| module | interface | duck role |
|---|---|---|
| **Robot Arm** (Feetech STS3215 bus, 1 Mbps half-duplex, TX via 330 Ω) | UART port 3/4 | the servo bus master — same servos, same protocol as ODM v2 |
| Motion Sensor (ICM-40608 / LSM6DSOX, 100 Hz task) | I²C | body IMU (replaces BNO055) |
| Distance Sensor (VL53L5CX 8×8 @15 Hz) | I²C (hardware Wire) | Microduck's ToF, same chip family |
| Speaker (MAX98357A I²S) | 2 ports | quacks (ODM v2 uses the same amp) |
| Microphone (ICS-43434 I²S) | 2 ports | optional |
| LED Grid (9× WS2812) | 1 port | eyes |
| Button | 1 port | stand / sit |
| DC motor (N20), Stepper | — | not used (a duck walks) |

Gaps: no camera module (Microduck streams WebRTC video — out of scope for an MCU); no
7.4 V rail (servos take a 2S pack directly, as on ODM v2); the Robot Arm driver is
hard-wired to 6 joints (rewritten here as `DuckBus` for N servos with SYNC READ).

## 4. Routes

**A. Open Duck Mini v2 body + Atech brain — recommended.**
Print the 129 ODM v2 parts as they are, 14 × STS3215 7.4 V, 2S 18650 pack; the Atech
board replaces Pi + adapter + IMU inside the trunk; Atech ToF + eyes + speaker on short
cables into the head; feet switches and antenna servos on bare port lines. The trained
ODM v2 walking policy transfers because the body is unchanged (mass delta = board vs Pi).
Cost ≈ €295 of non-Atech parts (BOM.md). Licence Apache-2.0 — usable for an Atech demo.

**B. 1:1 Microduck (XL330).** Print the official meshes, 15 × XL330-M288 (~€25 each), a
new Dynamixel-2.0 driver for the Atech bus module (same wire, different framing), an
Atech board that cannot go inside (trunk 82 × 64 mm vs board 60 × 120 mm — it would ride
as a backpack on a 737 g robot), and retrain every policy for the new mass. Meshes are
CC BY-NC-SA: fine for a personal build, not for an Atech product. Keep as a stretch goal
once A walks.

**C. Atech-native "duckling".** Two dc_motor modules and the duck shell as a case: a
rolling toy, not a Microduck. Not pursued.

## 5. Concept A in detail

### Architecture

```
 2S 18650 pack ──BMS──┬── 7.4 V ── STS3215 ×14 (one daisy chain, IDs 10-14 / 20-24 / 30-33)
                      │                 ▲ data (3-wire)
                      └── 5 V UBEC ── USB-C ── Atech 14-port board (ESP32-S3)
                                                 port 3  Robot Arm module ── bus master
                                                 port 1  IMU          port 2  ToF (head)
                                                 port 7  eyes (head)  port 4  button
                                                 port 5  feet L/R     port 6  antennas L/R
                                                 port 13+14 speaker   port 9+10 mic (opt.)
```

### Port map (firmware/platformio.ini)

| port | GPIO A / B | use |
|---|---|---|
| 1 | 9 / 8 | IMU SDA / SCL (Wire) |
| 2 | 5 / 4 | ToF SDA / SCL (Wire1) |
| 3 | 17 / 18 | servo bus TX (330 Ω) / RX — Serial1 @ 1 Mbps |
| 4 | 16 / 15 | button / spare |
| 5 | 11 / 10 | foot switch L / R (pull-up, closed = LOW) |
| 6 | 13 / 12 | antenna servo L / R (LEDC 50 Hz) |
| 7 | 6 / 7 | NeoPixel eyes (both module revisions) |
| 9+10 | 40/41, 1/2 | microphone (optional) |
| 13+14 | 39/38, 36/35 | speaker LRCLK / BCLK / DIN |
| 11 | 43 / 44 | free |

`board.yaml` says ports 3 and 4 are the UART-capable ones; the Robot Arm module notes say
10 and 11. Bench-check which pair the module's driver actually uses (Q5).

### Control contract (= Open Duck Mini v2 runtime, so its policies transfer)

- 50 Hz loop. Observation (101): gyro[3] rad/s, accel[3] m/s², command[7] (vx vy wz +
  4 head angles), q − init[14], q̇ × 0.05 [14], last three actions [42], motor_targets
  [14], feet contacts [2], gait phase cos/sin [2] (period 0.54 s = 27 steps).
- Action[14] → motor_targets = init + 0.25 · action; head joints overwritten by the
  command. Servo gains P = 32 legs / 8 head; "limp" P = 2 when tilt > 60° (Microduck's
  limp-fall idea).
- Policy: `BEST_WALK_ONNX_2.onnx` (ODM v2) exported to `policy_weights.h`
  (101→512→256→128→14, swish, tanh head, 218 k params = 853 KB fp32 in flash). The header
  matches onnxruntime to 6e-7. Cost per forward ≈ 220 k MACs — expected 1–3 ms on the
  ESP32-S3 (`BENCH` measures it; budget 20 ms).
- Bus timing per tick: one SYNC WRITE (14 × 7 B) + one SYNC READ (14 replies × 14 B) at
  1 Mbps ≈ 3 ms. Fine.

### Budgets

| | ODM v2 | Atech Duck |
|---|---|---|
| mass | 2107 g (MJCF) | ≈ same − (Pi + adapter + BNO055 ≈ 40 g) + (board + 7 modules, **unweighed**, Q2) |
| servo rail | 2S, 14 × up to 2.7 A stall | unchanged; the Atech 12 V / 5 V rails are not used for servos |
| logic | Pi 5 V ~0.5 A | ESP32-S3 + modules < 0.5 A from the UBEC via USB-C |
| compute | Pi Zero 2 W, Python + onnxruntime | ESP32-S3 240 MHz, C float MLP |
| runtime | ~1 h | ≈ same (servos dominate) |

## 6. What exists in this repo today

| path | what | verified |
|---|---|---|
| `reference/` | 8 upstream repos + press photos (`scripts/fetch_reference.sh`) | fetched |
| `scripts/assemble_mjcf.py` | MJCF → posed STL/GLB + anatomy table + JSON | ran on both ducks |
| `scripts/render_reference.py` | lineup render | `docs/img/reference_lineup.png` |
| `out/microduck.{stl,glb}`, `out/open_duck_mini_v2.{stl,glb}` | posed reference bodies, mm | gitignored, regenerate |
| `docs/anatomy_*.md` | joints, link lengths, masses | generated |
| `firmware/` | PlatformIO: `DuckBus` (STS bus, N servos, SYNC READ/WRITE), `Policy` (MLP), `main.cpp` (OFF/STAND/POLICY/LIMP, JSON serial, fall guard), Atech drivers copied | **compiles** (`pio run`: RAM 8 %, flash 39 %); never flashed, no hardware here |
| `firmware/tools/onnx_to_header.py` | ONNX MLP → C header, cross-checked vs onnxruntime | both duck policies export |
| `BOM.md` | parts and prices | draft |

### Motion simulation (added 2026-09-02 evening)

`scripts/sim_walk.py` runs the ODM v2 walking policy headless in MuJoCo, stock and with a
60 g board merged into the trunk inertial: neither falls in 19 s, forward 0.096 → 0.091 m/s
(commanded 0.15), turn 0.75 rad/s (commanded 1.0), sidestep 0.11 m/s, knees at 1.2 N·m RMS
with peaks on the 3.23 N·m clip. The policy ignores commands below ~0.1 m/s. Full numbers in
`docs/motion_sim.md`; interactive page `docs/motion_page.html` (orbit, scrub, torque chart).
The page replays the recorded motion on the full-resolution onshape-to-robot export
(`reference/Open_Duck_Mini/mini_bdx/robots/open_duck_mini_v2`, servos, horns, bearings,
sheets, Pi, cells) via `scripts/replay_hires.py`; the physics model's own visuals are
104-face hulls and must not be shown to a human (`docs/img/odm_joint_closeups*.png`).
The partsmith framework written from this work: `~/projects/partsmith/FRAMEWORK.md`.

## 7. Plan

| phase | digital (done here) | physical (Gustav) | gate |
|---|---|---|---|
| P0 order | BOM.md | 14 × STS3215 7.4 V, 2 × 18650 + holder + BMS + UBEC, switches, inserts | parts in hand |
| P1 bus bench | firmware `SCAN`, `STATUS`, `BENCH`, `J` | one STS3215 on the Robot Arm module, 7.4 V bench supply | servo answers, BENCH < 5 ms |
| P2 one leg | `J` per joint, offsets in `duck_config.h` | print one ODM v2 leg (≈ 10 parts), 5 servos | joint ranges match the anatomy table |
| P3 full body | `STAND` ramp, fall guard | print the rest, wire the chain, IMU in the trunk | stands 60 s on `STAND` |
| P4 walk | `POLICY` with the ODM v2 policy, IMU frame calibration (Q4) | gamepad → `CMD` from a laptop script | walks on a flat floor |
| P5 duck-ness | ToF theremin, eyes, quacks (Microduck cheat-sheet behaviours) | head modules on cables | demo |
| P6 (optional) | retrain in Open_Duck_Playground for our mass / new moves; route B | | |

## 8. Questions for Gustav (bold assumptions used meanwhile)

1. **Trunk fit.** Does the 14-port board (60 × 120 × 7 + ~20 mm of modules) go inside the
   ODM v2 body (150 × 110 outer, 143 mm tall, shared with 2 × 18650 and three servos)?
   Assumed yes, across the width. Alternative: the 8-port board, or a backpack.
2. **Mass of the board + modules** — weigh them on the Atech scale; the CoM shifts if it
   is > ~80 g.
3. **Servos** — buy 14 × STS3215 **7.4 V** (≈ €196)? Feetech STS3032 (22 g, 0.44 N·m)
   would be the "Microduck-sized" Feetech, but no open body exists for it.
4. **IMU frame** — the Atech IMU's axes vs. the duck frame (x forward, y left, z up) and
   its mounting rotation (`setMountRotation`) — a 1-minute test once mounted.
5. **Which port pair is the module's UART** — board.yaml (3/4) vs the driver notes (10/11).
6. **The 15th servo (beak).** Dropped in v1; ODM v2 has a static beak. Add later on ID 34.
7. **Camera.** Out of scope on an MCU; an ESP32-CAM in the head is the only cheap route.
8. **Licensing intent.** Personal build → anything goes. Atech product / marketing →
   stay on ODM v2 (Apache-2.0); do not ship the Microduck meshes (NC).

## 9. Sources

Pollen press kit (pollen-robotics.com/microduck/press-kit), `pollen-robotics/microduck`,
`pollen-robotics/microduck_rl`, `fanhao375/microduck-replica` (teardown, actuators,
fasteners), `lingzolabs/microduck-hardware-replica`, `boris721/microduck-3d`,
`apirrone/Open_Duck_Mini` (+ Playground, Runtime), TechCrunch / CNX Software launch
coverage 2026-08-27/28, Atech `backend/modules/my_modules.yaml` + `motherboard/board.yaml`.
