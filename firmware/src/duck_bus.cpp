#include "duck_bus.h"

static const uint32_t RESP_TIMEOUT_US = 4000;   // per packet at 1 Mbps a reply is ~150 us
static const uint32_t ECHO_TIMEOUT_US = 3000;

DuckBus::DuckBus(HardwareSerial& serial, int txPin, int rxPin) : _s(serial), _tx(txPin), _rx(rxPin) {}

void DuckBus::begin(uint32_t baud) {
    _s.begin(baud, SERIAL_8N1, _rx, _tx);
    _s.setRxBufferSize(512);
    delay(50);
    flushInput();
}

void DuckBus::flushInput() { while (_s.available()) _s.read(); }

void DuckBus::discardEcho(int bytes) {
    uint32_t t0 = micros(); int n = 0;
    while (n < bytes && (micros() - t0) < ECHO_TIMEOUT_US) {
        if (_s.available()) { _s.read(); n++; }
    }
}

void DuckBus::sendPacket(uint8_t id, uint8_t instr, const uint8_t* params, int n) {
    flushInput();
    uint8_t len = (uint8_t)(n + 2);
    uint8_t chk = id + len + instr;
    uint8_t buf[80]; int k = 0;
    buf[k++] = 0xFF; buf[k++] = 0xFF; buf[k++] = id; buf[k++] = len; buf[k++] = instr;
    for (int i = 0; i < n; i++) { buf[k++] = params[i]; chk += params[i]; }
    buf[k++] = (uint8_t)~chk;
    _s.write(buf, k);
    _s.flush();
    discardEcho(k);
}

int DuckBus::receivePacket(uint8_t* idOut, uint8_t* errOut, uint8_t* buf, int bufLen, uint32_t timeoutUs) {
    uint32_t t0 = micros();
    int hdr = 0;
    while (hdr < 2) {
        if ((micros() - t0) > timeoutUs) return -1;
        if (_s.available()) { hdr = (_s.read() == 0xFF) ? hdr + 1 : 0; }
    }
    auto next = [&](uint8_t& b) -> bool {
        while (!_s.available()) { if ((micros() - t0) > timeoutUs) return false; }
        b = _s.read(); return true;
    };
    uint8_t id, len;
    if (!next(id) || !next(len)) return -1;
    if (len < 2 || len > 64) return -1;
    uint8_t pkt[64];
    for (int i = 0; i < len; i++) if (!next(pkt[i])) return -1;
    uint8_t chk = id + len;
    for (int i = 0; i < len - 1; i++) chk += pkt[i];
    if ((uint8_t)~chk != pkt[len - 1]) return -1;
    if (idOut) *idOut = id;
    if (errOut) *errOut = pkt[0];
    int dataLen = len - 2; if (dataLen > bufLen) dataLen = bufLen;
    for (int i = 0; i < dataLen; i++) buf[i] = pkt[1 + i];
    return dataLen;
}

bool DuckBus::ping(uint8_t id) {
    sendPacket(id, STSInst::PING, nullptr, 0);
    uint8_t rid, err, b[4];
    return receivePacket(&rid, &err, b, 4, RESP_TIMEOUT_US) >= 0 && rid == id;
}

bool DuckBus::write8(uint8_t id, uint8_t reg, uint8_t v) {
    uint8_t p[2] = { reg, v };
    sendPacket(id, STSInst::WRITE, p, 2);
    if (id == 0xFE) return true;
    uint8_t rid, err, b[4];
    return receivePacket(&rid, &err, b, 4, RESP_TIMEOUT_US) >= 0 && err == 0;
}
bool DuckBus::write16(uint8_t id, uint8_t reg, uint16_t v) {
    uint8_t p[3] = { reg, (uint8_t)(v & 0xFF), (uint8_t)(v >> 8) };
    sendPacket(id, STSInst::WRITE, p, 3);
    if (id == 0xFE) return true;
    uint8_t rid, err, b[4];
    return receivePacket(&rid, &err, b, 4, RESP_TIMEOUT_US) >= 0 && err == 0;
}
int DuckBus::read8(uint8_t id, uint8_t reg) {
    uint8_t p[2] = { reg, 1 }; sendPacket(id, STSInst::READ, p, 2);
    uint8_t rid, err, b[2];
    if (receivePacket(&rid, &err, b, 2, RESP_TIMEOUT_US) < 1 || err) return -1;
    return b[0];
}
int DuckBus::read16(uint8_t id, uint8_t reg) {
    uint8_t p[2] = { reg, 2 }; sendPacket(id, STSInst::READ, p, 2);
    uint8_t rid, err, b[2];
    if (receivePacket(&rid, &err, b, 2, RESP_TIMEOUT_US) < 2 || err) return -1;
    return b[0] | (b[1] << 8);
}
bool DuckBus::torque(uint8_t id, bool on)      { return write8(id, STSReg::TORQUE_ENABLE, on ? 1 : 0); }
bool DuckBus::setKp(uint8_t id, uint8_t kp)    { return write8(id, STSReg::P_COEF, kp); }
bool DuckBus::setKd(uint8_t id, uint8_t kd)    { return write8(id, STSReg::D_COEF, kd); }
bool DuckBus::setAccel(uint8_t id, uint8_t a)  { return write8(id, STSReg::ACC, a); }
bool DuckBus::setPositionMode(uint8_t id)      { return write8(id, STSReg::MODE, 0); }

bool DuckBus::syncWritePositions(const uint8_t* ids, int n, const uint16_t* pos, uint16_t timeMs, uint16_t speed) {
    if (n <= 0 || n > 20) return false;
    uint8_t p[2 + 20 * 7]; int k = 0;
    p[k++] = STSReg::GOAL_POS; p[k++] = 6;
    for (int i = 0; i < n; i++) {
        uint16_t v = pos[i] > 4095 ? 4095 : pos[i];
        p[k++] = ids[i];
        p[k++] = v & 0xFF; p[k++] = v >> 8;
        p[k++] = timeMs & 0xFF; p[k++] = timeMs >> 8;
        p[k++] = speed & 0xFF; p[k++] = speed >> 8;
    }
    sendPacket(0xFE, STSInst::SYNC_WRITE, p, k);
    return true;
}

void DuckBus::unpackState(const uint8_t* d, ServoState& s) {
    s.pos   = d[0] | (d[1] << 8);
    s.speed = decodeSigned(d[2] | (d[3] << 8));
    s.load  = decodeSigned(d[4] | (d[5] << 8));
    s.volt  = d[6];
    s.temp  = d[7];
    s.ok    = true;
}

int DuckBus::syncReadState(const uint8_t* ids, int n, ServoState* out) {
    if (n <= 0 || n > 20) return 0;
    uint8_t p[2 + 20]; int k = 0;
    p[k++] = STSReg::PRESENT_POS; p[k++] = 8;
    for (int i = 0; i < n; i++) p[k++] = ids[i];
    for (int i = 0; i < n; i++) out[i].ok = false;
    sendPacket(0xFE, STSInst::SYNC_READ, p, k);
    int got = 0;
    for (int i = 0; i < n; i++) {
        uint8_t rid, err, b[8];
        int len = receivePacket(&rid, &err, b, 8, RESP_TIMEOUT_US);
        if (len < 8) break;                       // a silent servo ends the burst (the rest time out)
        for (int j = 0; j < n; j++) if (ids[j] == rid) { unpackState(b, out[j]); got++; break; }
    }
    return got;
}

bool DuckBus::readState(uint8_t id, ServoState& s) {
    uint8_t p[2] = { STSReg::PRESENT_POS, 8 }; sendPacket(id, STSInst::READ, p, 2);
    uint8_t rid, err, b[8];
    if (receivePacket(&rid, &err, b, 8, RESP_TIMEOUT_US) < 8) { s.ok = false; return false; }
    unpackState(b, s); return true;
}
