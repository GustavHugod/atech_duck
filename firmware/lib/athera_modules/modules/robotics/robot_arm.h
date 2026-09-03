/**
 * @file robot_arm.h
 * @brief SO-ARM100 6-Axis Robot Arm Controller for Athera
 *
 * Controls a 6-axis robot arm using Feetech STS3215 servo motors
 * over a half-duplex UART bus (daisy-chained servos, IDs 1-6).
 *
 * Features:
 * - Move individual joints or all joints simultaneously (sync write)
 * - Read joint positions, temperature, voltage
 * - Torque enable/disable (per joint or all)
 * - In-memory motion recording and playback
 * - Save/load recordings to ESP32 flash (NVS, 10 slots)
 *
 * Protocol: Feetech STS3215 half-duplex UART at 1Mbps
 * Position range: 0-4095 (maps to ~0-240 degrees)
 * Center position: 2048
 *
 * Athera Connector:
 * - Line A (signal): TX pin → 330Ω resistor → servo data line
 * - Line B (signal_b): RX pin → servo data line (direct)
 * - GND: shared with servo power supply
 * - Servo power: external 6-7.4V supply (NOT from Athera board)
 */

#ifndef ROBOT_ARM_MODULE_H
#define ROBOT_ARM_MODULE_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include <Preferences.h>

// Feetech STS3215 register addresses
namespace STS {
    constexpr uint8_t REG_ID              = 5;
    constexpr uint8_t REG_BAUD            = 6;
    constexpr uint8_t REG_MODE            = 33;
    constexpr uint8_t REG_TORQUE_ENABLE   = 40;
    constexpr uint8_t REG_GOAL_POSITION   = 42;
    constexpr uint8_t REG_GOAL_TIME       = 44;
    constexpr uint8_t REG_GOAL_SPEED      = 46;
    constexpr uint8_t REG_PRESENT_POSITION = 56;
    constexpr uint8_t REG_PRESENT_SPEED   = 58;
    constexpr uint8_t REG_PRESENT_LOAD    = 60;
    constexpr uint8_t REG_PRESENT_VOLTAGE = 62;
    constexpr uint8_t REG_PRESENT_TEMP    = 63;

    constexpr uint8_t INST_PING       = 0x01;
    constexpr uint8_t INST_READ       = 0x02;
    constexpr uint8_t INST_WRITE      = 0x03;
    constexpr uint8_t INST_REG_WRITE  = 0x04;
    constexpr uint8_t INST_ACTION     = 0x05;
    constexpr uint8_t INST_SYNC_WRITE = 0x83;

    constexpr uint8_t BROADCAST_ID = 0xFE;

    constexpr uint16_t POS_MIN = 0;
    constexpr uint16_t POS_MAX = 4095;
    constexpr uint16_t POS_CENTER = 2048;
}

// Motion recording frame
struct ArmFrame {
    uint16_t positions[6];
};

class RobotArm {
public:
    static constexpr int NUM_JOINTS = 6;
    static constexpr int MAX_FRAMES = 2000;
    static constexpr int MAX_SLOTS = 10;

    /**
     * @brief Construct robot arm controller
     * @param txPin GPIO for TX (connect via 330Ω resistor to servo data line)
     * @param rxPin GPIO for RX (connect directly to servo data line)
     * @param serialPort Which HardwareSerial to use (1 or 2)
     */
    RobotArm(int txPin, int rxPin, int serialPort = 1);

    /**
     * @brief Initialize UART bus and scan for servos
     * @param baud Servo bus baud rate (default 1000000)
     * @return true if at least one servo found
     */
    bool begin(uint32_t baud = 1000000);

    // ========== Joint Control ==========

    /** Move a single joint to position (0-4095), optional transition time in ms */
    bool moveJoint(uint8_t jointId, uint16_t position, uint16_t timeMs = 0);

    /** Move all 6 joints simultaneously (sync write) */
    bool moveAll(const uint16_t positions[6], uint16_t timeMs = 0);

    /** Read current position of a joint (0-4095), returns -1 on error */
    int readJoint(uint8_t jointId);

    /** Read all 6 joint positions. Returns number of successful reads. */
    int readAll(uint16_t positions[6]);

    /** Center all joints (move to 2048) */
    bool centerAll(uint16_t timeMs = 1000);

    // ========== Torque ==========

    /** Enable or disable torque on a single joint */
    bool setTorque(uint8_t jointId, bool enable);

    /** Enable or disable torque on all joints */
    void setTorqueAll(bool enable);

    // ========== Telemetry ==========

    /** Read temperature of a joint in degrees C, returns -1 on error */
    int readTemperature(uint8_t jointId);

    /** Read voltage of a joint in 0.1V units, returns -1 on error */
    int readVoltage(uint8_t jointId);

    /** Check if a joint is responding */
    bool pingJoint(uint8_t jointId);

    /** Read a joint's status/alarm byte (Feetech error flags). Returns -1 if no response. */
    int readStatus(uint8_t jointId);

    /** Ping every joint and print decoded diagnostics (alarm flags, temp, voltage, load) to Serial. */
    void printDiagnostics();

    /** Scan bus and return number of servos found */
    int scanJoints();

    /** Get number of joints found during last scan */
    int getJointCount() { return _jointCount; }

    // ========== Motion Recording ==========

    /** Start recording joint positions at given interval (ms) */
    void startRecording(uint16_t intervalMs = 100);

    /** Call periodically during recording to capture a frame. Returns false when buffer full. */
    bool captureFrame();

    /** Stop recording */
    void stopRecording();

    /** Is recording in progress? */
    bool isRecording() { return _recording; }

    /** Get number of recorded frames */
    int getFrameCount() { return _frameCount; }

    /** Get recorded interval in ms */
    uint16_t getRecordingInterval() { return _recordIntervalMs; }

    /** Play back recording. speedPct=100 is normal speed, 50=half, 200=double */
    void playRecording(uint16_t speedPct = 100);

    /** Check if playback is in progress */
    bool isPlaying() { return _playing; }

    /** Stop playback */
    void stopPlayback() { _playing = false; }

    /** Get frame data (for streaming to gateway) */
    const ArmFrame* getFrames() { return _frames; }

    // ========== Flash Storage ==========

    /** Save current recording to flash slot (0-9) */
    bool saveToSlot(uint8_t slot);

    /** Load recording from flash slot (0-9) */
    bool loadFromSlot(uint8_t slot);

private:
    HardwareSerial* _serial;
    int _txPin;
    int _rxPin;
    int _serialPort;

    uint8_t _jointIds[NUM_JOINTS] = {1, 2, 3, 4, 5, 6};
    int _jointCount = 0;

    // Recording state
    ArmFrame* _frames = nullptr;
    int _frameCount = 0;
    uint16_t _recordIntervalMs = 100;
    bool _recording = false;
    bool _playing = false;
    unsigned long _lastFrameTime = 0;

    // Low-level servo bus methods
    void _sendPacket(uint8_t id, uint8_t instruction, const uint8_t* params, int paramLen);
    int _receivePacket(uint8_t* idOut, uint8_t* errOut, uint8_t* buf, int bufLen);
    bool _writeRegister(uint8_t id, uint8_t addr, const uint8_t* data, int len);
    int _readRegister(uint8_t id, uint8_t addr, uint8_t len, uint8_t* buf);
    void _flushInput();
    void _discardEcho(int bytesSent);
};

#endif // ROBOT_ARM_MODULE_H
