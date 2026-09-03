// Atech Duck — Open-Duck-Mini-v2-class biped on the Atech 14-port board (ESP32-S3).
//
// One Feetech STS bus (Atech robot_arm module on port 3) carries 14 STS3215 servos;
// the Atech IMU module is the body IMU; feet switches, NeoPixel eyes, speaker and the
// 8x8 ToF are Atech modules too. The control loop mirrors the Open Duck Mini v2 runtime:
// 50 Hz, 101-D observation, 14-D action, action_scale 0.25 around the init pose,
// head joints driven straight from the command vector.
//
// Serial (USB-C, 115200), one command per line:
//   HELP | STATUS | SCAN | BENCH
//   OFF            torque off everywhere (the duck goes limp — hold it)
//   STAND          ramp to the init pose over 1.5 s, hold with full gains
//   POLICY         run the MLP at 50 Hz (from STAND only)
//   LIMP           low gains, still holding (fall recovery / handling)
//   CMD vx vy wz [neck_pitch head_pitch head_yaw head_roll]     (m/s, m/s, rad/s, rad)
//   J <index|name> <rad>       nudge one joint in STAND
//   KP <legs> [head]           servo P gains
//   EYES r g b | BEEP
// Telemetry: one JSON line at 10 Hz  {"t":ms,"mode":..,"q":[14],"v":volt,"tmax":C,"imu":[gx gy gz ax ay az],"feet":[l r],"loop_us":..,"pol_us":..}

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "duck_config.h"
#include "duck_bus.h"
#include "policy.h"
#include <i2c_hardware.h>
#include <icm40608.h>
#if DUCK_HAS_EYES
#include <neopixel.h>
#endif
#if DUCK_HAS_SPEAKER
#include <speaker.h>
#endif
#if DUCK_HAS_TOF
#include <vl53l5cx.h>
#endif
#if DUCK_HAS_BUTTON
#include <button.h>
#endif

#define FW_VERSION "atech-duck/0.1"

// ---------------------------------------------------------------- hardware
DuckBus  bus(Serial1, BUS_TX_PIN, BUS_RX_PIN);
WireI2C  imuI2c(Wire, IMU_SDA_PIN, IMU_SCL_PIN);
ICM40608 imu(&imuI2c);
#if DUCK_HAS_EYES
NeoPixelGrid eyes(EYES_PIN_A, EYES_PIN_B);
#endif
#if DUCK_HAS_SPEAKER
Speaker speaker(SPK_BCLK_PIN, SPK_LRCLK_PIN, SPK_DIN_PIN);
#endif
#if DUCK_HAS_TOF
VL53L5CX_Sensor tof(Wire1);
bool tofOk = false;
#endif
#if DUCK_HAS_BUTTON
ButtonModule button(BTN_PIN);
#endif

// ---------------------------------------------------------------- state
enum Mode { MODE_OFF, MODE_STAND, MODE_POLICY, MODE_LIMP };
static const char* MODE_NAMES[] = { "off", "stand", "policy", "limp" };
Mode mode = MODE_OFF;

uint8_t    ids[DUCK_NUM_JOINTS];
ServoState servo[DUCK_NUM_JOINTS];
bool       online[DUCK_NUM_JOINTS];
int        onlineCount = 0;
float      q[DUCK_NUM_JOINTS];        // joint angle, rad (MJCF convention)
float      qd[DUCK_NUM_JOINTS];       // rad/s
float      target[DUCK_NUM_JOINTS];   // commanded joint angle, rad
float      standFrom[DUCK_NUM_JOINTS];
uint32_t   standT0 = 0;
static const uint32_t STAND_RAMP_MS = 1500;

float command[NUM_COMMANDS] = { 0, 0, 0, 0, 0, 0, 0 };
float lastAct[3][DUCK_NUM_JOINTS];    // last, last_last, last_last_last
float motorTargets[DUCK_NUM_JOINTS];  // policy targets (rad) = init + act*scale
float phase = 0;                      // 0..1 gait phase
float obs[POLICY_OBS_MAX];
float act[DUCK_NUM_JOINTS];
uint32_t loopUs = 0, policyUs = 0, lastTick = 0, lastTelemetry = 0;
bool feetL = false, feetR = false;
float gyro[3] = { 0, 0, 0 }, accel[3] = { 0, 0, 0 };
bool imuOk = false;

// ---------------------------------------------------------------- helpers
static inline uint16_t radToCounts(int j, float rad) {
    float c = SERVO_CENTER + DUCK_JOINTS[j].sign * rad * COUNTS_PER_RAD + DUCK_JOINTS[j].offset_counts;
    if (c < 0) c = 0; if (c > 4095) c = 4095;
    return (uint16_t)(c + 0.5f);
}
static inline float countsToRad(int j, uint16_t counts) {
    return DUCK_JOINTS[j].sign * ((float)counts - SERVO_CENTER - DUCK_JOINTS[j].offset_counts) / COUNTS_PER_RAD;
}
static void setGains(uint8_t legs, uint8_t head) {
    for (int j = 0; j < DUCK_NUM_JOINTS; j++) {
        bool isHead = (j >= HEAD_JOINT_FIRST && j < HEAD_JOINT_FIRST + HEAD_JOINT_COUNT);
        bus.setKp(ids[j], isHead ? head : legs);
    }
}
static void torqueAll(bool on) { for (int j = 0; j < DUCK_NUM_JOINTS; j++) bus.torque(ids[j], on); }
static void writeTargets() {
    uint16_t pos[DUCK_NUM_JOINTS];
    for (int j = 0; j < DUCK_NUM_JOINTS; j++) pos[j] = radToCounts(j, target[j]);
    bus.syncWritePositions(ids, DUCK_NUM_JOINTS, pos, 0, 0);
}
static int readServos() {
    int n = bus.syncReadState(ids, DUCK_NUM_JOINTS, servo);
    for (int j = 0; j < DUCK_NUM_JOINTS; j++) {
        if (!servo[j].ok) continue;                // keep the last good value
        q[j]  = countsToRad(j, servo[j].pos);
        qd[j] = DUCK_JOINTS[j].sign * (float)servo[j].speed / COUNTS_PER_RAD;
    }
    return n;
}
static void readImu() {
    if (!imuOk) return;
    // Atech IMU: deg/s and g in the module frame -> rad/s and m/s^2. Axis mapping to the
    // duck frame (x forward, y left, z up) is a calibration step: see DESIGN.md Q4.
    gyro[0]  = imu.readGyroX() * DEG_TO_RAD;
    gyro[1]  = imu.readGyroY() * DEG_TO_RAD;
    gyro[2]  = imu.readGyroZ() * DEG_TO_RAD;
    accel[0] = imu.readAccelX() * 9.81f;
    accel[1] = imu.readAccelY() * 9.81f;
    accel[2] = imu.readAccelZ() * 9.81f;
}
static void readFeet() {
#if DUCK_HAS_FEET
    feetL = digitalRead(FOOT_L_PIN) == LOW;
    feetR = digitalRead(FOOT_R_PIN) == LOW;
#endif
}
static float tiltDeg() { return imuOk ? imu.getTiltAngle() : 0.0f; }

static void setEyes(uint8_t r, uint8_t g, uint8_t b) {
#if DUCK_HAS_EYES
    eyes.setAll(r, g, b); eyes.show();
#endif
}
static void beep() {
#if DUCK_HAS_SPEAKER
    speaker.playNote(523.25f, 90); speaker.playNote(659.25f, 90); speaker.playNote(783.99f, 120);
#endif
}

static void enterMode(Mode m) {
    switch (m) {
    case MODE_OFF:
        torqueAll(false); setEyes(20, 0, 0); break;
    case MODE_STAND:
        for (int j = 0; j < DUCK_NUM_JOINTS; j++) { standFrom[j] = q[j]; target[j] = q[j]; motorTargets[j] = DUCK_JOINTS[j].init_rad; }
        standT0 = millis();
        setGains(KP_LEGS, KP_HEAD); torqueAll(true); setEyes(0, 0, 25); break;
    case MODE_POLICY:
        for (int k = 0; k < 3; k++) for (int j = 0; j < DUCK_NUM_JOINTS; j++) lastAct[k][j] = 0;
        for (int j = 0; j < DUCK_NUM_JOINTS; j++) motorTargets[j] = DUCK_JOINTS[j].init_rad;
        phase = 0; setGains(KP_LEGS, KP_HEAD); setEyes(0, 25, 0); break;
    case MODE_LIMP:
        setGains(KP_LIMP, KP_LIMP); setEyes(25, 12, 0); break;
    }
    mode = m;
    Serial.printf("{\"event\":\"mode\",\"mode\":\"%s\"}\n", MODE_NAMES[mode]);
}

// ---------------------------------------------------------------- observation (ODM v2 layout, 101 floats)
static int buildObs() {
    int k = 0;
    for (int i = 0; i < 3; i++) obs[k++] = gyro[i];
    for (int i = 0; i < 3; i++) obs[k++] = accel[i];
    for (int i = 0; i < NUM_COMMANDS; i++) obs[k++] = command[i];
    for (int j = 0; j < DUCK_NUM_JOINTS; j++) obs[k++] = q[j] - DUCK_JOINTS[j].init_rad;
    for (int j = 0; j < DUCK_NUM_JOINTS; j++) obs[k++] = qd[j] * DOF_VEL_SCALE;
    for (int h = 0; h < 3; h++) for (int j = 0; j < DUCK_NUM_JOINTS; j++) obs[k++] = lastAct[h][j];
    for (int j = 0; j < DUCK_NUM_JOINTS; j++) obs[k++] = motorTargets[j];
    obs[k++] = feetL ? 1.0f : 0.0f;
    obs[k++] = feetR ? 1.0f : 0.0f;
    obs[k++] = cosf(2.0f * PI * phase);
    obs[k++] = sinf(2.0f * PI * phase);
    return k;
}

static void policyStep() {
    int n = buildObs();
    if (n != Policy::inputSize()) {            // wrong policy for this contract: hold still
        static bool warned = false;
        if (!warned) { Serial.printf("{\"event\":\"error\",\"msg\":\"obs %d != policy in %d\"}\n", n, Policy::inputSize()); warned = true; }
        return;
    }
    uint32_t t0 = micros();
    Policy::forward(obs, act);
    policyUs = micros() - t0;
    for (int j = 0; j < DUCK_NUM_JOINTS; j++) {
        lastAct[2][j] = lastAct[1][j]; lastAct[1][j] = lastAct[0][j]; lastAct[0][j] = act[j];
        motorTargets[j] = DUCK_JOINTS[j].init_rad + act[j] * ACTION_SCALE;
    }
    for (int h = 0; h < HEAD_JOINT_COUNT; h++) motorTargets[HEAD_JOINT_FIRST + h] = command[3 + h];
    for (int j = 0; j < DUCK_NUM_JOINTS; j++) target[j] = motorTargets[j];
    // gait phase: 1.0..1.2 x base frequency with forward speed (ODM v2 runtime)
    float f = 1.0f + (fabsf(command[0]) / 0.15f) * 0.2f;
    phase += f / PHASE_STEPS_PER_PERIOD;
    if (phase >= 1.0f) phase -= 1.0f;
}

// ---------------------------------------------------------------- serial
static int findJoint(const char* s) {
    for (int j = 0; j < DUCK_NUM_JOINTS; j++) if (!strcmp(s, DUCK_JOINTS[j].name)) return j;
    int i = atoi(s); return (i >= 0 && i < DUCK_NUM_JOINTS && (s[0] >= '0' && s[0] <= '9')) ? i : -1;
}
static void printStatus() {
    Serial.printf("{\"event\":\"status\",\"fw\":\"%s\",\"mode\":\"%s\",\"servos_online\":%d,\"imu\":%d,\"policy\":\"%s\",\"cmd\":[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f],\"joints\":[",
                  FW_VERSION, MODE_NAMES[mode], onlineCount, imuOk, Policy::describe(),
                  command[0], command[1], command[2], command[3], command[4], command[5], command[6]);
    for (int j = 0; j < DUCK_NUM_JOINTS; j++)
        Serial.printf("%s{\"n\":\"%s\",\"id\":%d,\"on\":%d,\"q\":%.3f,\"v\":%.1f,\"t\":%d}", j ? "," : "", DUCK_JOINTS[j].name, ids[j], online[j], q[j], servo[j].volt / 10.0f, servo[j].temp);
    Serial.println("]}");
}
static void scanBus() {
    onlineCount = 0;
    for (int j = 0; j < DUCK_NUM_JOINTS; j++) { online[j] = bus.ping(ids[j]); if (online[j]) onlineCount++; }
    Serial.printf("{\"event\":\"scan\",\"online\":%d,\"missing\":[", onlineCount);
    bool first = true;
    for (int j = 0; j < DUCK_NUM_JOINTS; j++) if (!online[j]) { Serial.printf("%s%d", first ? "" : ",", ids[j]); first = false; }
    Serial.println("]}");
}
static void handleLine(char* line) {
    char* cmd = strtok(line, " \t\r\n"); if (!cmd) return;
    for (char* p = cmd; *p; p++) *p = toupper(*p);
    if (!strcmp(cmd, "HELP")) {
        Serial.println("OFF STAND POLICY LIMP | CMD vx vy wz [np hp hy hr] | J <joint> <rad> | KP legs [head] | SCAN STATUS BENCH EYES r g b BEEP");
    } else if (!strcmp(cmd, "OFF"))    enterMode(MODE_OFF);
    else if (!strcmp(cmd, "STAND"))    enterMode(MODE_STAND);
    else if (!strcmp(cmd, "LIMP"))     enterMode(MODE_LIMP);
    else if (!strcmp(cmd, "POLICY")) {
        if (mode != MODE_STAND) Serial.println("{\"event\":\"error\",\"msg\":\"POLICY needs STAND first\"}");
        else enterMode(MODE_POLICY);
    } else if (!strcmp(cmd, "CMD")) {
        for (int i = 0; i < NUM_COMMANDS; i++) { char* v = strtok(nullptr, " \t\r\n"); if (!v) break; command[i] = atof(v); }
    } else if (!strcmp(cmd, "J")) {
        char* jn = strtok(nullptr, " \t\r\n"); char* v = strtok(nullptr, " \t\r\n");
        int j = jn ? findJoint(jn) : -1;
        if (j < 0 || !v) Serial.println("{\"event\":\"error\",\"msg\":\"J <joint> <rad>\"}");
        else if (mode != MODE_STAND) Serial.println("{\"event\":\"error\",\"msg\":\"J only in STAND\"}");
        else { target[j] = atof(v); standT0 = 0; }
    } else if (!strcmp(cmd, "KP")) {
        char* a = strtok(nullptr, " \t\r\n"); char* b = strtok(nullptr, " \t\r\n");
        if (a) setGains((uint8_t)atoi(a), (uint8_t)(b ? atoi(b) : KP_HEAD));
    } else if (!strcmp(cmd, "SCAN"))   scanBus();
    else if (!strcmp(cmd, "STATUS"))   printStatus();
    else if (!strcmp(cmd, "BENCH")) {
        uint32_t us = Policy::benchmarkUs(50);
        Serial.printf("{\"event\":\"bench\",\"policy\":\"%s\",\"forward_us\":%u,\"budget_us\":%u}\n", Policy::describe(), us, CONTROL_PERIOD_US);
    } else if (!strcmp(cmd, "EYES")) {
        char* r = strtok(nullptr, " "); char* g = strtok(nullptr, " "); char* b = strtok(nullptr, " ");
        if (r && g && b) setEyes(atoi(r), atoi(g), atoi(b));
    } else if (!strcmp(cmd, "BEEP"))   beep();
    else Serial.printf("{\"event\":\"error\",\"msg\":\"unknown %s\"}\n", cmd);
}
static void pollSerial() {
    static char buf[128]; static int n = 0;
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') { if (n) { buf[n] = 0; handleLine(buf); n = 0; } }
        else if (n < (int)sizeof(buf) - 1) buf[n++] = c;
    }
}
static void telemetry() {
    Serial.printf("{\"t\":%lu,\"mode\":\"%s\",\"q\":[", (unsigned long)millis(), MODE_NAMES[mode]);
    for (int j = 0; j < DUCK_NUM_JOINTS; j++) Serial.printf("%s%.3f", j ? "," : "", q[j]);
    uint8_t vmin = 255, tmax = 0;
    for (int j = 0; j < DUCK_NUM_JOINTS; j++) if (servo[j].ok) { if (servo[j].volt < vmin) vmin = servo[j].volt; if (servo[j].temp > tmax) tmax = servo[j].temp; }
    Serial.printf("],\"v\":%.1f,\"tmax\":%d,\"imu\":[%.3f,%.3f,%.3f,%.2f,%.2f,%.2f],\"tilt\":%.0f,\"feet\":[%d,%d],\"phase\":%.2f,\"loop_us\":%u,\"pol_us\":%u",
                  vmin == 255 ? 0.0f : vmin / 10.0f, tmax, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], tiltDeg(), feetL, feetR, phase, loopUs, policyUs);
#if DUCK_HAS_TOF
    if (tofOk) Serial.printf(",\"tof_min_mm\":%d", tof.getMinDistance());
#endif
    Serial.println("}");
}

// ---------------------------------------------------------------- arduino
void setup() {
    Serial.begin(115200);
    delay(300);
    for (int j = 0; j < DUCK_NUM_JOINTS; j++) { ids[j] = DUCK_JOINTS[j].id; q[j] = DUCK_JOINTS[j].init_rad; target[j] = q[j]; }

    bus.begin(1000000);
    scanBus();
    for (int j = 0; j < DUCK_NUM_JOINTS; j++) { bus.setPositionMode(ids[j]); bus.setAccel(ids[j], SERVO_ACCEL); }
    readServos();

    imuI2c.begin(400000);
    imuOk = imu.begin();
    if (imuOk) imu.calibrateGyro(100);

#if DUCK_HAS_FEET
    pinMode(FOOT_L_PIN, INPUT_PULLUP); pinMode(FOOT_R_PIN, INPUT_PULLUP);
#endif
#if DUCK_HAS_EYES
    eyes.begin(); eyes.setBrightness(40);
#endif
#if DUCK_HAS_SPEAKER
    speaker.begin(); speaker.setVolume(0.4f);
#endif
#if DUCK_HAS_TOF
    Wire1.begin(TOF_SDA_PIN, TOF_SCL_PIN, 400000);
    tofOk = tof.begin();
#endif
#if DUCK_HAS_BUTTON
    button.begin();
#endif
    enterMode(MODE_OFF);
    Serial.printf("{\"event\":\"boot\",\"fw\":\"%s\",\"servos_online\":%d,\"imu\":%d,\"policy\":\"%s\",\"policy_in\":%d,\"obs_len\":%d}\n",
                  FW_VERSION, onlineCount, imuOk, Policy::describe(), Policy::inputSize(), buildObs());
    lastTick = micros();
}

void loop() {
    pollSerial();
#if DUCK_HAS_BUTTON
    button.update();
    if (button.wasPressed()) enterMode(mode == MODE_OFF ? MODE_STAND : MODE_OFF);   // one button: stand / sit limp
#endif
    uint32_t now = micros();
    if (now - lastTick < CONTROL_PERIOD_US) return;
    lastTick += CONTROL_PERIOD_US;
    uint32_t t0 = micros();

    readServos();
    readImu();
    readFeet();

    // fall guard: any active mode -> LIMP when the body tilts too far
    if ((mode == MODE_STAND || mode == MODE_POLICY) && imuOk && tiltDeg() > FALL_TILT_DEG) {
        Serial.printf("{\"event\":\"fall\",\"tilt\":%.0f}\n", tiltDeg());
        enterMode(MODE_LIMP);
    }

    switch (mode) {
    case MODE_STAND: {
        if (standT0) {
            float a = (float)(millis() - standT0) / STAND_RAMP_MS; if (a >= 1.0f) { a = 1.0f; standT0 = 0; }
            float s = a * a * (3 - 2 * a);      // smoothstep
            for (int j = 0; j < DUCK_NUM_JOINTS; j++) target[j] = standFrom[j] + (DUCK_JOINTS[j].init_rad - standFrom[j]) * s;
        }
        writeTargets();
        break; }
    case MODE_POLICY:
        policyStep(); writeTargets(); break;
    case MODE_LIMP:
        writeTargets(); break;          // hold whatever the targets were, softly
    case MODE_OFF:
        break;
    }
    loopUs = micros() - t0;
    if (millis() - lastTelemetry >= 100) { lastTelemetry = millis(); telemetry(); }
}
