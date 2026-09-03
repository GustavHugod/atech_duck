# Anatomy — open_duck_mini_v2

Source: `reference/Open_Duck_Playground/playground/open_duck_mini_v2/xmls/open_duck_mini_v2.xml`, pose = keyframe `home`

| | |
|---|---|
| Bounding box (X depth × Y width × Z height) | 255 × 249 × 489 mm |
| Standing height (ground → top) | 489 mm |
| Total mass (sum of MJCF inertials) | 2107 g |
| Bodies / joints / visual meshes | 16 / 14 / 44 |

## Joints (world frame, standing pose, mm above ground)

| # | joint | body | height z | x | y | axis (world) | range | pose |
|---|---|---|---|---|---|---|---|---|
| 0 | left_hip_yaw | hip_roll_assembly | 212.8 | -19.0 | 35.0 | -0.00 +0.00 +1.00 | -30…+30° | +0.1° |
| 1 | left_hip_roll | left_roll_to_pitch_assembly | 166.8 | 0.0 | 35.0 | -1.00 -0.00 -0.00 | -25…+25° | +3.0° |
| 2 | left_hip_pitch | knee_and_ankle_assembly | 162.9 | -35.2 | 109.0 | +0.00 -1.00 +0.05 | -70…+30° | -36.1° |
| 3 | left_knee | knee_and_ankle_assembly_2 | 99.4 | -81.5 | 105.5 | +0.00 -1.00 +0.05 | -90…+90° | +78.4° |
| 4 | left_ankle | foot_assembly | 41.3 | -28.6 | 102.4 | +0.00 -1.00 +0.05 | -90…+90° | -44.9° |
| 5 | neck_pitch | neck_pitch_assembly | 256.9 | 1.0 | 19.1 | -0.00 -1.00 +0.00 | -20…+65° | +0.0° |
| 6 | head_pitch | head_pitch_to_yaw | 322.9 | 1.0 | 19.2 | -0.00 -1.00 +0.00 | -45…+45° | +0.0° |
| 7 | head_yaw | neck_yaw_assembly | 379.9 | 1.0 | 0.1 | -0.00 +0.00 +1.00 | -160…+160° | +0.0° |
| 8 | head_roll | head_assembly | 360.7 | 41.9 | 0.1 | -1.00 -0.00 -0.00 | -30…+30° | +0.0° |
| 9 | right_hip_yaw | hip_roll_assembly_2 | 212.8 | -19.0 | -35.0 | -0.00 +0.00 +1.00 | -30…+30° | -0.2° |
| 10 | right_hip_roll | right_roll_to_pitch_assembly | 166.8 | 0.0 | -35.1 | -1.00 +0.00 -0.00 | -25…+25° | -3.7° |
| 11 | right_hip_pitch | knee_and_ankle_assembly_3 | 162.0 | -35.3 | -108.9 | +0.00 +1.00 +0.06 | -30…+70° | +36.4° |
| 12 | right_knee | knee_and_ankle_assembly_4 | 101.2 | -81.8 | -67.8 | -0.00 -1.00 -0.06 | -90…+90° | +79.0° |
| 13 | right_ankle | foot_assembly_2 | 43.5 | -28.5 | -64.3 | -0.00 -1.00 -0.06 | -90…+90° | -45.6° |

## Link lengths (straight-line distance between consecutive joints in each chain)

| from | to | length mm |
|---|---|---|
| left_hip_yaw | left_hip_roll | 49.8 |
| left_hip_roll | left_hip_pitch | 82.0 |
| left_hip_pitch | left_knee | 78.7 |
| left_knee | left_ankle | 78.7 |
| neck_pitch | head_pitch | 66.0 |
| head_pitch | head_yaw | 60.1 |
| head_yaw | head_roll | 45.2 |
| right_hip_yaw | right_hip_roll | 49.8 |
| right_hip_roll | right_hip_pitch | 82.0 |
| right_hip_pitch | right_knee | 86.9 |
| right_knee | right_ankle | 78.7 |

## Masses per body (MJCF inertial)

| body | mass g | depth |
|---|---|---|
| trunk_assembly | 698.5 | 1 |
| head_assembly | 406.6 | 5 |
| knee_and_ankle_assembly | 124.1 | 4 |
| knee_and_ankle_assembly_3 | 124.1 | 4 |
| neck_yaw_assembly | 91.8 | 4 |
| foot_assembly | 75.2 | 6 |
| foot_assembly_2 | 75.2 | 6 |
| left_roll_to_pitch_assembly | 75.2 | 3 |
| right_roll_to_pitch_assembly | 75.2 | 3 |
| knee_and_ankle_assembly_2 | 72.6 | 5 |
| knee_and_ankle_assembly_4 | 72.6 | 5 |
| hip_roll_assembly | 66.5 | 2 |
| hip_roll_assembly_2 | 66.5 | 2 |
| neck_pitch_assembly | 66.2 | 2 |
| head_pitch_to_yaw | 16.9 | 3 |
| base | 0.0 | 0 |
