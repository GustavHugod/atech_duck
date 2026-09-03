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
