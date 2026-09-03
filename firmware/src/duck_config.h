// Atech Duck — joint table and control constants.
//
// Joint order = Open Duck Mini v2 policy order (action index). Bus IDs follow the
// Open Duck Mini v2 / Microduck scheme: left leg 20-24, neck+head 30-33, right leg 10-14.
// init_rad = the standing pose the policy is trained around (ODM v2 runtime init_pos).
// sign / offset_counts are per-robot calibration: set after assembly (find_soft_offsets).
#pragma once
#include <stdint.h>

struct JointDef {
    const char* name;
    uint8_t     id;
    float       init_rad;
    float       sign;           // +1 / -1: servo direction vs. the MJCF joint axis
    int16_t     offset_counts;  // mechanical zero correction (servo counts)
};

static const int DUCK_NUM_JOINTS = 14;
static const JointDef DUCK_JOINTS[DUCK_NUM_JOINTS] = {
    {"left_hip_yaw",    20,  0.002f, 1, 0},
    {"left_hip_roll",   21,  0.053f, 1, 0},
    {"left_hip_pitch",  22, -0.630f, 1, 0},
    {"left_knee",       23,  1.368f, 1, 0},
    {"left_ankle",      24, -0.784f, 1, 0},
    {"neck_pitch",      30,  0.000f, 1, 0},
    {"head_pitch",      31,  0.000f, 1, 0},
    {"head_yaw",        32,  0.000f, 1, 0},
    {"head_roll",       33,  0.000f, 1, 0},
    {"right_hip_yaw",   10, -0.003f, 1, 0},
    {"right_hip_roll",  11, -0.065f, 1, 0},
    {"right_hip_pitch", 12,  0.635f, 1, 0},
    {"right_knee",      13,  1.379f, 1, 0},
    {"right_ankle",     14, -0.796f, 1, 0},
};
static const int HEAD_JOINT_FIRST = 5;   // neck_pitch .. head_roll = indices 5..8
static const int HEAD_JOINT_COUNT = 4;

// Control loop (ODM v2 runtime: 50 Hz, action_scale 0.25, dof_vel * 0.05)
static const float    CONTROL_HZ        = 50.0f;
static const uint32_t CONTROL_PERIOD_US = 20000;
static const float    ACTION_SCALE      = 0.25f;
static const float    DOF_VEL_SCALE     = 0.05f;
static const int      NUM_COMMANDS      = 7;      // vx vy wz neck_pitch head_pitch head_yaw head_roll
// Imitation phase: one gait period in control steps. ODM v2 takes this from the
// polynomial reference motion: period 0.54 s x 50 Hz (scripts/polynomial_coefficients.pkl).
static const float    PHASE_STEPS_PER_PERIOD = 27.0f;

// Servo gains (STS3215 P coefficient register). ODM v2 runtime: legs 32, head 8, "limp" 2.
static const uint8_t KP_LEGS = 32;
static const uint8_t KP_HEAD = 8;
static const uint8_t KP_LIMP = 2;
static const uint8_t SERVO_ACCEL = 0;   // 0 = max

// Fall detection: tilt from upright beyond this -> LIMP (like Microduck limp_fall)
static const float FALL_TILT_DEG = 60.0f;

// Servo <-> radians. STS3215: 4096 counts / rev, centre 2048.
static const float COUNTS_PER_RAD = 4096.0f / 6.283185307f;
static const uint16_t SERVO_CENTER = 2048;
