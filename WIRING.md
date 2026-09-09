# Wiring — Atech Duck (concept, nothing built yet)

Every number here comes from `firmware/platformio.ini`, the Atech module driver headers in
`firmware/lib/athera_modules/` and the Open Duck Mini v2 (ODM v2) wiring. Nothing has been
flashed or bench-checked; the open items are at the bottom.

## 1. Overview

```
 2× 18650 (2S) ──BMS──switch──XT30─┬── 7.4 V ──► 14× Feetech STS3215, one daisy chain
                                    │             IDs 10-14 right leg · 20-24 left leg · 30-33 neck/head
                                    │                     ▲ DATA (single wire, half duplex)
                                    │                     │
                                    │            Robot Arm module, port 3 ── Atech 14-port board (ESP32-S3)
                                    │
                                    └── 5 V UBEC ──► USB-C on the board  (logic + modules)

 board ports:  1 IMU   2 ToF (head)   3 servo bus   4 button   5 feet L/R   6 antennas L/R
               7 eyes (head)   9+10 microphone (optional)   13+14 speaker   8, 11, 12 free
```

Two power domains, common ground:

| domain | source | feeds | note |
|---|---|---|---|
| servo rail, 7.4 V nominal (6.6–8.4 V) | 2S 18650 through the BMS and the main switch | the 14 STS3215 only | 14 servos can pull 2.7 A each at stall; keep this on its own wiring (XT30, ≥ 18 AWG to the chain) |
| logic, 5 V | UBEC (≥ 3 A) off the servo rail, into the board's USB-C | board, every module, eyes, antenna servos | the board's own 12 V / 5 V rails are not used for the duck |

The servo rail must **never** come from the Atech board. GND of the two domains is joined at
the UBEC and again at the Robot Arm module (its GND pin is on the servo cable).

## 2. Atech modules and ports

Each Atech port carries Line A, Line B (two GPIOs), GND, 3V3, 5V and 12V. Module slots
are on the board; the head parts sit on short cables (module unplugged from the slot,
cable from the slot to the module). GPIO numbers are the ESP32-S3 pins the firmware uses
(`board.yaml` Line A / Line B).

| port | GPIO A / B | module | role | Line A | Line B | where |
|---|---|---|---|---|---|---|
| 1 | 9 / 8 | Motion Sensor (ICM-40608) | body IMU, 100 Hz | SDA | SCL | in the trunk, on the board, flat |
| 2 | 5 / 4 | Distance Sensor (VL53L5CX 8×8) | Microduck's "LiDAR", 15 Hz | SDA | SCL | in the head, on a 6-wire cable |
| 3 | 17 / 18 | Robot Arm (Feetech STS bus) | servo bus master, 1 Mbps | TX → 330 Ω → DATA | RX ← DATA | in the trunk; servo cable to the chain |
| 4 | 16 / 15 | Button | stand / sit (one button) | button | spare | trunk, reachable through the shell |
| 5 | 11 / 10 | — bare lines | foot contact switches | left foot | right foot | 2× SS-10 per foot in parallel, to GND; internal pull-up, closed = LOW |
| 6 | 13 / 12 | — bare lines | antenna micro servos (SG90) | left signal | right signal | servo + and − on the port's 5V and GND |
| 7 | 6 / 7 | LED Grid (3×3 NeoPixel) | eyes | data, RGB revision | data, RGBW revision | in the head, on a cable; the driver writes both lines |
| 13 + 14 | 39 / 38, 36 / 35 | Speaker (MAX98357A, double module) | quacks | 13 A = LRCLK | 13 B = BCLK, 14 B = DIN | trunk, grille through the shell |
| 9 + 10 | 40 / 41, 1 / 2 | Microphone (ICS-43434, double module), optional | not in the control loop | | | trunk |
| 8, 11, 12 | — | free | | | | |

I²C: the IMU is on `Wire` (port 1) and the ToF on `Wire1` (port 2), so the two 400 kHz buses
do not share a line. Both Atech I²C modules put SDA on Line A and SCL on Line B (driver
headers `icm40608.h`, `vl53l5cx.h`).

## 3. Servo bus (Robot Arm module → 14 servos)

The STS3215 has two identical 3-pin connectors (DATA, +V, GND) so the chain is plug to plug,
servo to servo, in any physical order; the bus ID in each servo's EEPROM decides who is who.

| pin on the Robot Arm module | to |
|---|---|
| Line A (TX, through the module's 330 Ω) | servo DATA |
| Line B (RX, direct) | servo DATA (same wire) |
| GND | servo GND (= 2S pack negative) |
| +V | **not from the module** — the chain's +V pins are fed by the 2S rail |

Everything the ESP32 transmits is echoed back on RX and discarded by `DuckBus`. One
control tick = one SYNC WRITE (14 × 7 B) + one SYNC READ (14 replies × 14 B) ≈ 3 ms at 1 Mbps.

Bus IDs and joint order (`firmware/src/duck_config.h`, same scheme as ODM v2 / Microduck):

| ID | joint | ID | joint | ID | joint |
|---|---|---|---|---|---|
| 10 | right_hip_yaw | 20 | left_hip_yaw | 30 | neck_pitch |
| 11 | right_hip_roll | 21 | left_hip_roll | 31 | head_pitch |
| 12 | right_hip_pitch | 22 | left_hip_pitch | 32 | head_yaw |
| 13 | right_knee | 23 | left_knee | 33 | head_roll |
| 14 | right_ankle | 24 | left_ankle | (34) | beak, not in v1 |

Servos ship with ID 1. Set IDs one servo at a time on the bench (only that servo on the
bus), then `SCAN` from the serial console must list all 14. Buy the **7.4 V** STS3215, not
the 12 V variant.

## 4. Head cable

Three things live in the head: the ToF (port 2), the eyes (port 7) and, later, the beak
servo (on the servo chain). The two modules travel up through the neck on one bundle:

| wire | port 2 (ToF) | port 7 (eyes) |
|---|---|---|
| 3V3 | yes | — |
| 5V | — | yes (LED power) |
| GND | yes | yes |
| Line A | SDA | data (RGB module) |
| Line B | SCL | data (RGBW module) |

Keep the I²C pair under ~30 cm; the NeoPixel data line is fine at that length.

## 5. Feet and antennas (no module, bare port lines)

- **Feet**: two SS-10 micro switches per foot wired in parallel between the port-5 line and
  GND. `INPUT_PULLUP` in firmware, pressed = LOW. These feed the policy's `feet contacts[2]`.
- **Antennas**: two 9 g servos, signal on port 6 Line A (left) / Line B (right), + on the
  port's 5V, − on GND. 50 Hz PWM (LEDC). Pins are reserved in `platformio.ini`; firmware 0.1
  does not drive them yet.

## 6. Bring-up order

1. Board alone on USB-C: `pio run -t upload`, `pio device monitor`, `HELP`, `BENCH`
   (policy forward time on the ESP32-S3; budget 20 ms).
2. Add the IMU module: `STATUS` shows gyro/accel; check the axes against the duck frame
   (x forward, y left, z up) and set the mount rotation.
3. Robot Arm module + one servo on a bench supply: `SCAN`. Then all 14.
4. `STAND` with the duck held in the air; check every joint's sign and offset
   (`J <name> <rad>`), edit `sign` / `offset_counts` in `duck_config.h`.
5. Eyes, speaker, ToF, button, feet: `EYES r g b`, `BEEP`, `STATUS`.
6. On the floor: `STAND` → `POLICY` with `CMD 0 0 0`, then `CMD 0.15 0 0`.

## 7. Open items (bench checks)

- Which port pair is the Robot Arm module's UART on this board revision: `board.yaml` says
  3/4, the module notes say 10/11 (DESIGN.md Q5). The firmware assumes port 3.
- The ToF pins were swapped in the first firmware draft (SCL on A). Fixed 2026-09-09 to
  match the module header (SDA on A); confirm on the bench.
- Mass and fit of the board + 7 modules inside the ODM v2 trunk (DESIGN.md Q1, Q2).
- Antenna servo current on the 5 V port pin (two SG90 can spike ~0.5 A each).
