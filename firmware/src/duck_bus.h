// Feetech STS/SCS serial-bus servo driver for N daisy-chained servos (STS3215, STS3032).
//
// Physical layer is the Atech robot_arm module: Line A = TX through 330 R to DATA,
// Line B = RX straight to DATA, GND shared with the servo supply (6-7.4 V, external).
// Half duplex on one wire: everything we send is echoed back and discarded.
// Protocol = Feetech "SCS" framing: FF FF ID LEN INSTR PARAMS... CHK (~sum).
#pragma once
#include <Arduino.h>
#include <HardwareSerial.h>

namespace STSReg {
enum : uint8_t {
    ID = 5, BAUD = 6, MIN_ANGLE = 9, MAX_ANGLE = 11, MAX_TEMP = 13, MAX_VOLT = 14, MIN_VOLT = 15,
    MAX_TORQUE = 16, P_COEF = 21, D_COEF = 22, I_COEF = 23, MODE = 33, TORQUE_ENABLE = 40,
    ACC = 41, GOAL_POS = 42, GOAL_TIME = 44, GOAL_SPEED = 46, LOCK = 55,
    PRESENT_POS = 56, PRESENT_SPEED = 58, PRESENT_LOAD = 60, PRESENT_VOLT = 62,
    PRESENT_TEMP = 63, MOVING = 66, PRESENT_CURRENT = 69
};
}
namespace STSInst {
enum : uint8_t { PING = 0x01, READ = 0x02, WRITE = 0x03, REG_WRITE = 0x04, ACTION = 0x05, SYNC_READ = 0x82, SYNC_WRITE = 0x83 };
}

struct ServoState {
    uint16_t pos = 0;      // counts 0..4095
    int16_t  speed = 0;    // counts/s, signed
    int16_t  load = 0;     // signed, 0..1000 = 0..100 %
    uint8_t  volt = 0;     // 0.1 V
    uint8_t  temp = 0;     // C
    bool     ok = false;
};

class DuckBus {
public:
    DuckBus(HardwareSerial& serial, int txPin, int rxPin);
    void begin(uint32_t baud = 1000000);

    bool ping(uint8_t id);
    bool write8(uint8_t id, uint8_t reg, uint8_t v);
    bool write16(uint8_t id, uint8_t reg, uint16_t v);
    int  read8(uint8_t id, uint8_t reg);          // -1 on error
    int  read16(uint8_t id, uint8_t reg);         // -1 on error
    bool torque(uint8_t id, bool on);
    bool setKp(uint8_t id, uint8_t kp);
    bool setKd(uint8_t id, uint8_t kd);
    bool setAccel(uint8_t id, uint8_t acc);
    bool setPositionMode(uint8_t id);

    /** One SYNC WRITE of goal positions (counts) to n servos, optional time/speed limits. */
    bool syncWritePositions(const uint8_t* ids, int n, const uint16_t* pos, uint16_t timeMs = 0, uint16_t speed = 0);
    /** One SYNC READ of PRESENT_POS..PRESENT_TEMP (8 bytes) for n servos. Returns servos answered. */
    int  syncReadState(const uint8_t* ids, int n, ServoState* out);
    /** Slow path: read one servo's state with an ordinary READ. */
    bool readState(uint8_t id, ServoState& s);

    static int16_t decodeSigned(uint16_t raw) { return (raw & 0x8000) ? -(int16_t)(raw & 0x7FFF) : (int16_t)raw; }
    static uint16_t encodeSigned(int16_t v)   { return v < 0 ? (uint16_t)(0x8000 | (-v)) : (uint16_t)v; }

private:
    HardwareSerial& _s;
    int _tx, _rx;
    void sendPacket(uint8_t id, uint8_t instr, const uint8_t* params, int n);
    int  receivePacket(uint8_t* idOut, uint8_t* errOut, uint8_t* buf, int bufLen, uint32_t timeoutUs);
    void flushInput();
    void discardEcho(int bytes);
    static void unpackState(const uint8_t* d, ServoState& s);
};
