# Bill of materials — Atech Duck (concept, prices from the ODM v2 sheet 2025 + Atech catalogue)

Quantities are for the recommended concept (Open Duck Mini v2 body, Atech electronics).
Nothing ordered yet. Prices are indicative EUR; "TBD" = check before ordering.

## Atech (from the kit)

| qty | item | port(s) | note |
|---|---|---|---|
| 1 | Atech 14-port motherboard (ESP32-S3) | — | the brain; replaces Pi Zero 2 W + Waveshare bus adapter + BNO055 |
| 1 | Robot Arm module (Feetech STS bus, half-duplex UART) | 3 | the servo bus interface; 1 Mbps; servos powered externally |
| 1 | Motion Sensor module (ICM-40608 / LSM6DSOX) | 1 | body IMU, 100 Hz |
| 1 | Distance Sensor module (VL53L5CX 8×8) | 2 | Microduck's "LiDAR" (same chip family), on a cable into the head |
| 1 | Speaker module (MAX98357A) | 13+14 | quacks; ODM v2 has the same amp |
| 1 | LED Grid module (3×3 NeoPixel) | 7 | eyes, on a cable into the head (or 2 bare WS2812 on the same line) |
| 1 | Button module | 4 | stand / sit |
| 1 | Microphone module (optional) | 9+10 | Microduck has one; not in the control loop |

## Actuators and power (ODM v2 BOM, Feetech 7.4 V family)

| qty | item | unit € | total € | note |
|---|---|---|---|---|
| 14 | Feetech STS3215 **7.4 V / 19 kg·cm** bus servo | 14 | 196 | buy the 7.4 V variant, not 12 V; Alibaba/AliExpress |
| 2 | 9 g micro servo (antennas) | 3.3 | 7 | PWM on port 6 |
| 4 | SS-10 micro switch (feet contacts) | 1 | 4 | 2 used per foot in parallel |
| 2 | 18650 Li-ion 3000 mAh 30 A (Molicel P30B) | 5 | 10 | 2S = 7.4 V servo rail |
| 1 | 2S 18650 holder | 5 | 5 | |
| 1 | 2S BMS | 8.4 | 8.4 | |
| 1 | 5 V UBEC (≥3 A) | 4 | 4 | feeds the Atech board via USB-C (board's own 12 V/5 V rails are not used) |
| 1 | power switch, XT30 pair, 2× 2.1 mm barrel | ~14 | 14 | |
| 1 | USB-C 2S charger | 10 | 10 | |
| 3 | bearing (ODM v2 sheet) | 3.5 | 10.5 | |
| — | M3 heat-set inserts, M3/M2 screws | 6 | 6 | |
| — | PLA ~500 g + a little TPU (feet soles) | 20 | 20 | |
| | **subtotal** | | **≈ 295** | vs. ODM v2 sheet €398 incl. Pi/IMU/adapter which Atech replaces |

## Not needed from the ODM v2 list

Raspberry Pi Zero 2 W (26 €), SD card (10 €), Waveshare bus servo adapter (5 €), BNO055 IMU (40 €) — all replaced by the Atech board and modules.

## Microduck features and how they map

| Microduck | here |
|---|---|
| 15× Dynamixel XL330-M288 | 14× STS3215 (ODM v2 body) — beak servo dropped for v1, see DESIGN.md Q6 |
| Radxa Zero 3W + HAT + IMU-on-bus board | Atech 14-port board + IMU module |
| VL53L5CX/L8CX 8×8 ToF | Atech Distance module (VL53L5CX) |
| Camera IMX219, Wi-Fi WebRTC | not in v1 (no Atech camera module); ESP32-S3 Wi-Fi exists for telemetry |
| Mic + speaker + codec | Atech Speaker + Microphone modules |
| NFC ×2 | dropped |
| NP-F550 2600 mAh 2S | 2× 18650 2S (ODM v2) |
| 9 ONNX policies | retrain in Open_Duck_Playground (STS3215 actuator model already there); firmware runs the MLP on-chip |

<!-- bom:generated:start -->

## Mechanical parts — generated from the CAD

`scripts/build_bom.py` reads the part list of the Open Duck Mini v2 assembly (`robot_motors.xml`, 89 part occurrences, 42 distinct) and the Atech board. Full table with volumes: `cad/BOM.csv`. Posed CAD: `cad/atech_duck_stand.glb` (`scripts/export_cad.py`).

Printed parts: 42 PLA pieces ≈ 1919 g solid, 2 TPU pieces ≈ 58 g solid (100 % infill upper bound; the ODM v2 guide prints shells at 3 walls / 15 % infill, so plan ~40–50 % of that).

| part | qty | kind | print file | bbox mm | solid g | note |
|---|---|---|---|---|---|---|
| battery_pack_lid | 1 | printed PLA | battery_pack_lid.stl | 8 x 51 x 86 | 29 |  |
| body_back | 1 | printed PLA | body_back.stl | 40 x 110 x 125 | 131 |  |
| body_front | 1 | printed PLA | body_front.stl | 10 x 110 x 143 | 177 |  |
| body_middle_bottom | 1 | printed PLA | body_middle_bottom.stl | 150 x 110 x 86 | 183 |  |
| body_middle_top | 1 | printed PLA | body_middle_top.stl | 150 x 110 x 57 | 95 |  |
| foot_bottom_pla | 2 | printed PLA | foot_bottom_pla.stl | 93 x 28 x 8 | 20 |  |
| foot_side | 2 | printed PLA | foot_side.stl | 103 x 12 x 46 | 27 |  |
| foot_top | 2 | printed PLA | foot_top.stl | 103 x 35 x 46 | 49 |  |
| head | 1 | printed PLA | head.stl | 200 x 197 x 59 | 269 |  |
| head_bot_sheet | 1 | printed PLA | head_bot_sheet.stl | 194 x 192 x 3 | 82 |  |
| head_pitch_to_yaw | 1 | printed PLA | head_pitch_to_yaw.stl | 32 x 45 x 72 | 17 |  |
| head_roll_mount | 1 | printed PLA | head_roll_mount.stl | 40 x 65 x 15 | 17 |  |
| head_yaw_to_roll | 1 | printed PLA | head_yaw_to_roll.stl | 88 x 80 x 42 | 36 |  |
| left_antenna_holder | 1 | printed PLA | left_antenna_holder.stl | 27 x 20 x 42 | 4 |  |
| left_cache | 1 | printed PLA | left_cache.stl | 90 x 40 x 153 | 80 |  |
| left_knee_to_ankle_left_sheet | 4 | printed PLA | knee_to_ankle_left_sheet.stl | 31 x 8 x 71 | 10 |  |
| left_knee_to_ankle_right_sheet | 4 | printed PLA | knee_to_ankle_right_sheet.stl | 31 x 7 x 71 | 8 |  |
| left_roll_to_pitch | 1 | printed PLA | left_roll_to_pitch.stl | 47 x 86 x 31 | 36 |  |
| leg_spacer | 4 | printed PLA | leg_spacer.stl | 28 x 37 x 10 | 9 |  |
| neck_left_sheet | 1 | printed PLA | neck_left_sheet.stl | 31 x 7 x 58 | 10 |  |
| neck_right_sheet | 1 | printed PLA | neck_right_sheet.stl | 31 x 6 x 58 | 8 |  |
| right_antenna_holder | 1 | printed PLA | right_antenna_holder.stl | 27 x 20 x 42 | 4 |  |
| right_cache | 1 | printed PLA | right_cache.stl | 90 x 40 x 153 | 80 |  |
| right_roll_to_pitch | 1 | printed PLA | right_roll_to_pitch.stl | 47 x 86 x 31 | 36 |  |
| roll_motor_bottom | 2 | printed PLA | roll_motor_bottom.stl | 32 x 31 x 31 | 11 |  |
| roll_motor_top | 2 | printed PLA | roll_motor_top.stl | 38 x 29 x 20 | 11 |  |
| trunk_bottom | 1 | printed PLA | trunk_bottom.stl | 54 x 108 x 74 | 102 |  |
| trunk_top | 1 | printed PLA | trunk_top.stl | 125 x 101 x 41 | 179 |  |
| foot_bottom_tpu | 2 | printed TPU | foot_bottom_tpu.stl | 102 x 41 x 8 | 29 |  |
| drive_palonier | 14 | servo horn (STS3215) | — | 20 x 5 x 20 | — |  |
| passive_palonier | 14 | servo horn (STS3215) | — | 20 x 3 x 20 | — |  |
| roll_bearing | 3 | bearing | — | 32 x 32 x 7 | — |  |
| sg90 | 2 | micro servo SG90 | — | 33 x 13 x 31 | — |  |
| antenna | 2 | antenna (steel wire) | — | 4 x 39 x 146 | — |  |
| cell | 2 | battery cell 18650 | — | 18 x 18 x 65 | — |  |
| bms | 1 | electronics | — | 41 x 16 x 4 | — |  |
| power_switch | 1 | electronics | — | 23 x 7 x 17 | — |  |
| usb_c_charger | 1 | electronics | — | 38 x 18 x 5 | — |  |
| board | 2 | PCB | — | 42 x 33 x 2 | — |  |
| bno055 | 1 | electronics, replaced by Atech Motion Sensor module | — | 20 x 27 x 3 | — | replaced by Atech Motion Sensor module |
| holder | 1 | placeholder box (holder) | — | 76 x 41 x 20 | — |  |
| raspberrypizerow | 1 | electronics, replaced by Atech 14-port board | — | 66 x 4 x 31 | — | replaced by Atech 14-port board |
| atech_motherboard_14_port | 1 | Atech board | — | 60 x 120 x 6 | — | position in the trunk assumed (DESIGN.md Q1) |

<!-- bom:generated:end -->
