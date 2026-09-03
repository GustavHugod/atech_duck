# Anatomy — microduck

Source: `reference/microduck_rl/src/mjlab_microduck/robot/microduck/robot_walk.xml`, pose = keyframe `STAND`

| | |
|---|---|
| Bounding box (X depth × Y width × Z height) | 123 × 142 × 272 mm |
| Standing height (ground → top) | 272 mm |
| Total mass (sum of MJCF inertials) | 737 g |
| Bodies / joints / visual meshes | 15 / 14 / 70 |

## Joints (world frame, standing pose, mm above ground)

| # | joint | body | height z | x | y | axis (world) | range | pose |
|---|---|---|---|---|---|---|---|---|
| 0 | left_hip_yaw | yaw2roll | 112.2 | 6.0 | 17.5 | +0.00 +0.00 -1.00 | -25…+30° | +0.0° |
| 1 | left_hip_roll | hip_l | 99.7 | 22.5 | 17.5 | +1.00 +0.00 -0.00 | -22…+22° | -5.0° |
| 2 | left_hip_pitch | upper_leg_left | 97.5 | 4.0 | 42.4 | +0.00 +1.00 -0.09 | -90…+90° | -26.2° |
| 3 | left_knee | leg | 62.4 | -18.4 | 35.3 | -0.00 -1.00 +0.09 | -90…+90° | -0.3° |
| 4 | left_ankle | ankle_left | 22.6 | 0.0 | 57.9 | +0.00 +1.00 -0.09 | -90…+90° | +26.0° |
| 5 | neck_pitch | neck | 149.6 | 26.0 | 14.5 | +0.00 -1.00 +0.00 | -90…+60° | +20.0° |
| 6 | head_pitch | neck_pitch | 196.6 | 8.9 | 14.5 | +0.00 +1.00 -0.00 | -90…+90° | +20.0° |
| 7 | head_yaw | yaw_roll_motion | 215.3 | 8.9 | -0.0 | +0.00 +0.00 +1.00 | -170…+170° | +0.0° |
| 8 | head_roll | jaw_soft | 229.8 | -9.0 | -0.0 | -1.00 +0.00 +0.00 | -25…+25° | +0.0° |
| 9 | right_hip_yaw | bearing_roll | 112.2 | 6.0 | -17.5 | +0.00 +0.00 -1.00 | -30…+25° | +0.0° |
| 10 | right_hip_roll | hip_l_2 | 99.7 | 22.5 | -17.5 | +1.00 +0.00 -0.00 | -22…+22° | +5.0° |
| 11 | right_hip_pitch | upper_leg_right | 97.5 | 4.0 | -42.4 | -0.00 -1.00 -0.09 | -90…+90° | +26.2° |
| 12 | right_knee | leg_2 | 62.4 | -18.4 | -35.3 | +0.00 +1.00 +0.09 | -90…+90° | +0.3° |
| 13 | right_ankle | ankle_right | 22.6 | 0.0 | -57.9 | -0.00 -1.00 -0.09 | -90…+90° | -26.0° |

## Link lengths (straight-line distance between consecutive joints in each chain)

| from | to | length mm |
|---|---|---|
| left_hip_yaw | left_hip_roll | 20.7 |
| left_hip_roll | left_hip_pitch | 31.1 |
| left_hip_pitch | left_knee | 42.2 |
| left_knee | left_ankle | 49.4 |
| neck_pitch | head_pitch | 50.0 |
| head_pitch | head_yaw | 23.7 |
| head_yaw | head_roll | 23.0 |
| right_hip_yaw | right_hip_roll | 20.7 |
| right_hip_roll | right_hip_pitch | 31.1 |
| right_hip_pitch | right_knee | 42.2 |
| right_knee | right_ankle | 49.4 |

## Masses per body (MJCF inertial)

| body | mass g | depth |
|---|---|---|
| trunk_base | 199.2 | 0 |
| jaw_soft | 188.8 | 4 |
| yaw_roll_motion | 48.6 | 3 |
| upper_leg_left | 48.2 | 3 |
| upper_leg_right | 48.2 | 3 |
| neck | 36.8 | 1 |
| ankle_right | 30.0 | 5 |
| ankle_left | 30.0 | 5 |
| yaw2roll | 23.0 | 1 |
| bearing_roll | 23.0 | 1 |
| leg | 21.6 | 4 |
| leg_2 | 21.6 | 4 |
| hip_l | 6.2 | 2 |
| hip_l_2 | 6.2 | 2 |
| neck_pitch | 5.7 | 2 |
