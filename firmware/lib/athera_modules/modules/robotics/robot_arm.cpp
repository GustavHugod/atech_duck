#include "robot_arm.h"

static constexpr uint32_t RESPONSE_TIMEOUT_MS = 50;
static constexpr uint32_t ECHO_TIMEOUT_MS = 10;

RobotArm::RobotArm(int txPin, int rxPin, int serialPort)
    : _txPin(txPin), _rxPin(rxPin), _serialPort(serialPort) {
    // Select hardware serial instance
    if (serialPort == 2) {
        _serial = &Serial2;
    } else {
        _serial = &Serial1;
    }
}

bool RobotArm::begin(uint32_t baud) {
    _serial->begin(baud, SERIAL_8N1, _rxPin, _txPin);
    _serial->setTimeout(RESPONSE_TIMEOUT_MS);
    delay(100);

    // Scan for servos
    _jointCount = scanJoints();
    Serial.printf("[RobotArm] Found %d joints on Serial%d (TX=%d, RX=%d)\n",
                  _jointCount, _serialPort, _txPin, _rxPin);

    // Per-joint health report (alarm flags, temp, voltage, load).
    // Runs even if no joints were found, so a silent bus is obvious.
    printDiagnostics();

    return _jointCount > 0;
}

// ========== Joint Control ==========

bool RobotArm::moveJoint(uint8_t jointId, uint16_t position, uint16_t timeMs) {
    if (position > STS::POS_MAX) position = STS::POS_MAX;
    uint8_t data[6] = {
        (uint8_t)(position & 0xFF), (uint8_t)(position >> 8),
        (uint8_t)(timeMs & 0xFF),   (uint8_t)(timeMs >> 8),
        0, 0  // speed = 0 (use time-based control)
    };
    return _writeRegister(jointId, STS::REG_GOAL_POSITION, data, 6);
}

bool RobotArm::moveAll(const uint16_t positions[6], uint16_t timeMs) {
    // Sync write to all 6 joints atomically
    constexpr int dataPerServo = 6;
    int paramLen = 2 + NUM_JOINTS * (1 + dataPerServo);
    uint8_t params[128];

    params[0] = STS::REG_GOAL_POSITION;
    params[1] = dataPerServo;

    int idx = 2;
    for (int i = 0; i < NUM_JOINTS; i++) {
        uint16_t pos = positions[i] > STS::POS_MAX ? STS::POS_MAX : positions[i];
        params[idx++] = _jointIds[i];
        params[idx++] = pos & 0xFF;
        params[idx++] = pos >> 8;
        params[idx++] = timeMs & 0xFF;
        params[idx++] = timeMs >> 8;
        params[idx++] = 0; // speed low
        params[idx++] = 0; // speed high
    }

    _sendPacket(STS::BROADCAST_ID, STS::INST_SYNC_WRITE, params, paramLen);
    return true;
}

int RobotArm::readJoint(uint8_t jointId) {
    uint8_t buf[2];
    if (_readRegister(jointId, STS::REG_PRESENT_POSITION, 2, buf) < 2) return -1;
    return buf[0] | (buf[1] << 8);
}

int RobotArm::readAll(uint16_t positions[6]) {
    int count = 0;
    for (int i = 0; i < NUM_JOINTS; i++) {
        int pos = readJoint(_jointIds[i]);
        if (pos >= 0) {
            positions[i] = (uint16_t)pos;
            count++;
        } else {
            positions[i] = 0xFFFF; // invalid marker
        }
    }
    return count;
}

bool RobotArm::centerAll(uint16_t timeMs) {
    uint16_t center[6] = {
        STS::POS_CENTER, STS::POS_CENTER, STS::POS_CENTER,
        STS::POS_CENTER, STS::POS_CENTER, STS::POS_CENTER
    };
    return moveAll(center, timeMs);
}

// ========== Torque ==========

bool RobotArm::setTorque(uint8_t jointId, bool enable) {
    uint8_t val = enable ? 1 : 0;
    return _writeRegister(jointId, STS::REG_TORQUE_ENABLE, &val, 1);
}

void RobotArm::setTorqueAll(bool enable) {
    for (int i = 0; i < NUM_JOINTS; i++) {
        setTorque(_jointIds[i], enable);
    }
}

// ========== Telemetry ==========

int RobotArm::readTemperature(uint8_t jointId) {
    uint8_t buf[1];
    if (_readRegister(jointId, STS::REG_PRESENT_TEMP, 1, buf) < 1) return -1;
    return buf[0];
}

int RobotArm::readVoltage(uint8_t jointId) {
    uint8_t buf[1];
    if (_readRegister(jointId, STS::REG_PRESENT_VOLTAGE, 1, buf) < 1) return -1;
    return buf[0];
}

bool RobotArm::pingJoint(uint8_t jointId) {
    _sendPacket(jointId, STS::INST_PING, nullptr, 0);
    uint8_t rid, err;
    uint8_t buf[4];
    return _receivePacket(&rid, &err, buf, sizeof(buf)) >= 0;
}

int RobotArm::scanJoints() {
    int count = 0;
    for (int i = 0; i < NUM_JOINTS; i++) {
        if (pingJoint(_jointIds[i])) {
            count++;
        }
    }
    return count;
}

int RobotArm::readStatus(uint8_t jointId) {
    _sendPacket(jointId, STS::INST_PING, nullptr, 0);
    uint8_t rid, err;
    uint8_t buf[4];
    if (_receivePacket(&rid, &err, buf, sizeof(buf)) < 0) return -1;
    return err;
}

void RobotArm::printDiagnostics() {
    Serial.println("[RobotArm] --- Joint diagnostics ---");
    for (int i = 0; i < NUM_JOINTS; i++) {
        uint8_t id = _jointIds[i];

        // Ping first to grab the alarm/error byte.
        int status = readStatus(id);
        if (status < 0) {
            Serial.printf("[RobotArm]   Joint %d (ID %d): NO RESPONSE\n", i + 1, id);
            continue;
        }
        uint8_t err = (uint8_t)status;

        // Raw reads that, unlike _readRegister(), do NOT bail out when the
        // alarm bit is set — that is exactly the state we want to inspect.
        int temp = -1, volt = -1, load = -1;
        uint8_t rid, e2;
        uint8_t rbuf[4];
        uint8_t p[2];

        p[0] = STS::REG_PRESENT_TEMP; p[1] = 1;
        _sendPacket(id, STS::INST_READ, p, 2);
        if (_receivePacket(&rid, &e2, rbuf, 1) >= 1) temp = rbuf[0];

        p[0] = STS::REG_PRESENT_VOLTAGE; p[1] = 1;
        _sendPacket(id, STS::INST_READ, p, 2);
        if (_receivePacket(&rid, &e2, rbuf, 1) >= 1) volt = rbuf[0];

        p[0] = STS::REG_PRESENT_LOAD; p[1] = 2;
        _sendPacket(id, STS::INST_READ, p, 2);
        if (_receivePacket(&rid, &e2, rbuf, 2) >= 2) load = rbuf[0] | (rbuf[1] << 8);

        // Decode the Feetech STS alarm flags (bit meanings per STS3215 manual;
        // raw byte is printed too in case of firmware variation).
        char flags[80];
        if (err == 0) {
            strcpy(flags, "OK");
        } else {
            flags[0] = '\0';
            if (err & 0x01) strcat(flags, "VOLTAGE ");
            if (err & 0x02) strcat(flags, "ANGLE ");
            if (err & 0x04) strcat(flags, "OVERHEAT ");
            if (err & 0x08) strcat(flags, "OVERCURRENT ");
            if (err & 0x20) strcat(flags, "OVERLOAD ");
            if (flags[0] == '\0') strcpy(flags, "UNKNOWN ");
        }

        Serial.printf("[RobotArm]   Joint %d (ID %d): %s(0x%02X) | temp=",
                      i + 1, id, flags, err);
        if (temp >= 0) Serial.printf("%dC", temp); else Serial.print("?");
        Serial.print(" volt=");
        if (volt >= 0) Serial.printf("%d.%dV", volt / 10, volt % 10); else Serial.print("?");
        Serial.print(" load=");
        if (load >= 0) Serial.printf("%d", load & 0x3FF); else Serial.print("?");
        Serial.println();
    }
    Serial.println("[RobotArm] -------------------------");
}

// ========== Motion Recording ==========

void RobotArm::startRecording(uint16_t intervalMs) {
    if (!_frames) {
        _frames = new ArmFrame[MAX_FRAMES];
    }
    _frameCount = 0;
    _recordIntervalMs = intervalMs;
    _recording = true;
    _lastFrameTime = millis();
}

bool RobotArm::captureFrame() {
    if (!_recording || !_frames) return false;
    if (_frameCount >= MAX_FRAMES) {
        _recording = false;
        return false;
    }

    unsigned long now = millis();
    if (now - _lastFrameTime < _recordIntervalMs) return true; // not time yet
    _lastFrameTime = now;

    readAll(_frames[_frameCount].positions);
    _frameCount++;
    return true;
}

void RobotArm::stopRecording() {
    _recording = false;
}

void RobotArm::playRecording(uint16_t speedPct) {
    if (!_frames || _frameCount == 0) return;
    if (speedPct == 0) speedPct = 100;

    _playing = true;
    setTorqueAll(true);

    uint16_t interval = (uint16_t)((uint32_t)_recordIntervalMs * 100 / speedPct);

    for (int i = 0; i < _frameCount && _playing; i++) {
        // Filter out invalid frames
        bool valid = true;
        for (int j = 0; j < NUM_JOINTS; j++) {
            if (_frames[i].positions[j] == 0xFFFF) { valid = false; break; }
        }
        if (!valid) continue;

        moveAll(_frames[i].positions, interval);
        delay(interval);
    }
    _playing = false;
}

// ========== Flash Storage ==========

bool RobotArm::saveToSlot(uint8_t slot) {
    if (slot >= MAX_SLOTS || !_frames || _frameCount == 0) return false;

    char ns[8];
    snprintf(ns, sizeof(ns), "rec%d", slot);

    Preferences prefs;
    prefs.begin(ns, false);
    prefs.putInt("frames", _frameCount);
    prefs.putInt("interval", _recordIntervalMs);

    // Save in chunks (200 frames per key = 2400 bytes)
    const int chunkSize = 200;
    for (int i = 0; i < _frameCount; i += chunkSize) {
        int count = min(chunkSize, _frameCount - i);
        char key[8];
        snprintf(key, sizeof(key), "d%d", i / chunkSize);
        prefs.putBytes(key, &_frames[i], count * sizeof(ArmFrame));
    }

    prefs.end();
    Serial.printf("[RobotArm] Saved %d frames to slot %d\n", _frameCount, slot);
    return true;
}

bool RobotArm::loadFromSlot(uint8_t slot) {
    if (slot >= MAX_SLOTS) return false;

    char ns[8];
    snprintf(ns, sizeof(ns), "rec%d", slot);

    Preferences prefs;
    prefs.begin(ns, true);
    int count = prefs.getInt("frames", 0);
    if (count == 0) { prefs.end(); return false; }

    if (!_frames) {
        _frames = new ArmFrame[MAX_FRAMES];
    }

    _frameCount = min(count, MAX_FRAMES);
    _recordIntervalMs = prefs.getInt("interval", 100);

    const int chunkSize = 200;
    for (int i = 0; i < _frameCount; i += chunkSize) {
        int toRead = min(chunkSize, _frameCount - i);
        char key[8];
        snprintf(key, sizeof(key), "d%d", i / chunkSize);
        prefs.getBytes(key, &_frames[i], toRead * sizeof(ArmFrame));
    }

    prefs.end();
    Serial.printf("[RobotArm] Loaded %d frames from slot %d\n", _frameCount, slot);
    return true;
}

// ========== Low-Level Servo Bus ==========

void RobotArm::_flushInput() {
    while (_serial->available()) _serial->read();
}

void RobotArm::_discardEcho(int bytesSent) {
    unsigned long start = millis();
    int discarded = 0;
    while (discarded < bytesSent && (millis() - start) < ECHO_TIMEOUT_MS) {
        if (_serial->available()) {
            _serial->read();
            discarded++;
        }
    }
}

void RobotArm::_sendPacket(uint8_t id, uint8_t instruction, const uint8_t* params, int paramLen) {
    _flushInput();

    uint8_t length = paramLen + 2;
    uint8_t checksum = id + length + instruction;
    for (int i = 0; i < paramLen; i++) checksum += params[i];
    checksum = ~checksum;

    int totalBytes = 6 + paramLen;

    _serial->write(0xFF);
    _serial->write(0xFF);
    _serial->write(id);
    _serial->write(length);
    _serial->write(instruction);
    for (int i = 0; i < paramLen; i++) {
        _serial->write(params[i]);
    }
    _serial->write(checksum);
    _serial->flush();

    _discardEcho(totalBytes);
}

int RobotArm::_receivePacket(uint8_t* idOut, uint8_t* errOut, uint8_t* buf, int bufLen) {
    unsigned long start = millis();

    int headerCount = 0;
    while (headerCount < 2 && (millis() - start) < RESPONSE_TIMEOUT_MS) {
        if (_serial->available()) {
            if (_serial->read() == 0xFF) headerCount++;
            else headerCount = 0;
        }
    }
    if (headerCount < 2) return -1;

    uint8_t id, length;
    while (!_serial->available() && (millis() - start) < RESPONSE_TIMEOUT_MS) {}
    id = _serial->read();
    while (!_serial->available() && (millis() - start) < RESPONSE_TIMEOUT_MS) {}
    length = _serial->read();

    if (length < 2 || length > 64) return -1;

    int remaining = length;
    uint8_t packet[64];
    int idx = 0;
    while (idx < remaining && (millis() - start) < RESPONSE_TIMEOUT_MS) {
        if (_serial->available()) {
            packet[idx++] = _serial->read();
        }
    }
    if (idx < remaining) return -1;

    uint8_t checksum = id + length;
    for (int i = 0; i < remaining - 1; i++) checksum += packet[i];
    checksum = ~checksum;
    if (checksum != packet[remaining - 1]) return -1;

    if (idOut) *idOut = id;
    if (errOut) *errOut = packet[0];

    int dataLen = length - 2;
    if (dataLen > bufLen) dataLen = bufLen;
    for (int i = 0; i < dataLen; i++) {
        buf[i] = packet[1 + i];
    }
    return dataLen;
}

bool RobotArm::_writeRegister(uint8_t id, uint8_t addr, const uint8_t* data, int len) {
    uint8_t params[32];
    params[0] = addr;
    for (int i = 0; i < len && i < 30; i++) params[1 + i] = data[i];

    _sendPacket(id, STS::INST_WRITE, params, 1 + len);

    if (id == STS::BROADCAST_ID) return true;

    uint8_t rid, err;
    uint8_t buf[4];
    int n = _receivePacket(&rid, &err, buf, sizeof(buf));
    return (n >= 0 && err == 0);
}

int RobotArm::_readRegister(uint8_t id, uint8_t addr, uint8_t len, uint8_t* buf) {
    uint8_t params[2] = { addr, len };
    _sendPacket(id, STS::INST_READ, params, 2);

    uint8_t rid, err;
    int n = _receivePacket(&rid, &err, buf, len);
    if (n < 0 || err != 0) return -1;
    return n;
}
