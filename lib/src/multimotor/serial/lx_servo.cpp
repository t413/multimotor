#include "lx_servo.h"
#include "serial_drive_manager.h"
#include "serial_interface.h"
#include "../debugprint.h"
#include <string.h>

// LX16A Command definitions
constexpr uint8_t LX16A_SERVO_MOVE_TIME_WRITE = 1;
constexpr uint8_t LX16A_SERVO_MOVE_TIME_READ = 2;
constexpr uint8_t LX16A_SERVO_MOVE_TIME_WAIT_WRITE = 7;
constexpr uint8_t LX16A_SERVO_MOVE_START = 11;
constexpr uint8_t LX16A_SERVO_MOVE_STOP = 12;
constexpr uint8_t LX16A_SERVO_ID_WRITE = 13;
constexpr uint8_t LX16A_SERVO_ID_READ = 14;
constexpr uint8_t LX16A_SERVO_POS_READ = 28;// 0x1C
constexpr uint8_t LX16A_SERVO_OR_MOTOR_MODE_WRITE = 29; //0x1D
constexpr uint8_t LX16A_SERVO_LOAD_OR_UNLOAD_WRITE = 31;
constexpr uint8_t LX16A_SERVO_TEMP_READ = 26;
constexpr uint8_t LX16A_SERVO_VIN_READ = 27; //0x1B
constexpr uint8_t LX16A_SERVO_ANGLE_LIMIT_WRITE = 20;
constexpr uint8_t LX16A_SERVO_ANGLE_LIMIT_READ = 21;


#define LX_MIN_PACKET_LENGTH 4


LXServo::LXServo(uint8_t id, SerialDriveManager* bus, const char* name) : MotorDrive(name), id_(id), bus_(bus) {
    bus_->addDrive(this);
}

int LXServo::buildPacket(uint8_t* txbuf, uint8_t cmd, const uint8_t* params, int param_cnt, uint8_t id) {
    if (param_cnt < 0 || param_cnt > 4) return -1;
    int buflen = 6 + param_cnt;
    txbuf[0] = 0x55; txbuf[1] = 0x55;
    txbuf[2] = id;
    txbuf[3] = buflen - 3;
    txbuf[4] = cmd;
    for (int i = 0; i < param_cnt; i++) txbuf[5 + i] = params[i];
    uint8_t cksum = 0;
    for (int i = 2; i < buflen - 1; i++) cksum += txbuf[i];
    txbuf[buflen - 1] = ~cksum;
    return buflen;
}

bool LXServo::sendCommand(uint8_t cmd, const uint8_t* params, int param_cnt, bool expectResponse, uint32_t timeout_us) {
    if (!bus_) return false;
    uint8_t txbuf[32];
    int buflen = buildPacket(txbuf, cmd, params, param_cnt, id_);
    if (buflen < 0) return false;

    if (!bus_->writeAndConsumeEcho(txbuf, buflen, timeout_us))
        return false;

    if (!expectResponse) return true;

    uint8_t const* replyData; uint8_t replyLen;
    if (!bus_->waitForReply(id_, timeout_us, &replyData, &replyLen))
        return false;

    return handleIncoming(id_, replyData, replyLen, millis());
}

bool LXServo::requestStatus() {
    uint32_t now = millis();
    bool ret = requestPosition();
    //TODO calc velocity in handle method
    if (!lastTempRead_ || (now - lastTempRead_) > 5000)
        requestTemp();
    return ret;
}

bool LXServo::requestPosition() {
    uint8_t data[3] = {0};
    return sendCommand(LX16A_SERVO_POS_READ, data, 2, true);
}

bool LXServo::requestTemp() {
    uint8_t data[1] = {0};
    return sendCommand(LX16A_SERVO_TEMP_READ, data, 1, true);
}

bool LXServo::enable(bool en) {
    uint8_t enableParams[] = {(uint8_t)(en? 1 : 0)};
    return sendCommand(LX16A_SERVO_LOAD_OR_UNLOAD_WRITE, enableParams, 1, false);
    enabled_ = en;
}

bool LXServo::setMode(MotorMode mode) {
    switch (mode) {
        case MotorMode::Position: {
            // Enable servo mode
            uint8_t servoParams[] = {0, 0, 0, 0};
            sendCommand(LX16A_SERVO_OR_MOTOR_MODE_WRITE, servoParams, 4, false);
            return enable();
        }
        case MotorMode::Speed: {
            lastStatus_.mode = MotorMode::Speed;
            enabled_ = true;
            return enable();
        }
        default: {
            return enable(false);
        }
    }
}

bool LXServo::setSetpoint(MotorMode mode, float value) {
    if (!enabled_) return false;
    bool ret = false;

    if (mode == MotorMode::Position) {
        // Clamp to limits
        float angle = value;
        if (angle < minAngleDeg_) angle = minAngleDeg_;
        if (angle > maxAngleDeg_) angle = maxAngleDeg_;

        uint16_t ticks = angleToTicks(angle);
        uint16_t time = 0; // Move as fast as possible
        ret = movePosTime(ticks, time);
    } else if (mode == MotorMode::Speed) {
        // Convert speed to motor mode command
        ret = moveSpeed((int16_t)(value * 100));
        lastStatus_.velocity = value;
    }
    return ret;
}

bool LXServo::movePosTime(int16_t ticks, int16_t time) {
    uint8_t params[4];
    memcpy(params, &ticks, 2);
    memcpy(params + 2, &time, 2);
    return sendCommand(LX16A_SERVO_MOVE_TIME_WRITE, params, 4, false);
}

bool LXServo::moveSpeed(int16_t speed) {
    speed = -constrain(speed, -1000, 1000);
    uint8_t params[4] = {1, 0};
    memcpy(params + 2, &speed, 2);
    return sendCommand(LX16A_SERVO_OR_MOTOR_MODE_WRITE, params, 4, false);
}

bool LXServo::fetchVBus() {
    uint8_t data[3] = {0};
    return sendCommand(LX16A_SERVO_VIN_READ, data, 2, true);
}

bool LXServo::stop() {
    uint8_t params[1];
    return sendCommand(LX16A_SERVO_MOVE_STOP, params, 1, false);
}

bool LXServo::setAngleLimits(float minDeg, float maxDeg) {
    minAngleDeg_ = minDeg;
    maxAngleDeg_ = maxDeg;

    uint16_t minTicks = angleToTicks(minDeg);
    uint16_t maxTicks = angleToTicks(maxDeg);

    uint8_t params[4];
    memcpy(params, &minTicks, 2);
    memcpy(params + 2, &maxTicks, 2);
    return sendCommand(LX16A_SERVO_ANGLE_LIMIT_WRITE, params, 4, false);
}

MotorDrive* LXServo::makeDuplicate(int16_t newId) const {
    if (newId < 0) newId = DEFAULT_ID;
    return new LXServo(newId, bus_, "dupe");
}

bool LXServo::writeNewId(uint8_t newId, bool sendToDrive) {
    if (sendToDrive) {
        uint8_t params[] = {newId};
        if (!sendCommand(LX16A_SERVO_ID_WRITE, params, 1, false))
            return false;
    }
    id_ = newId;
    return true;
}

bool LXServo::ping(int timeout_ms) {
    uint8_t dummy[2] = {0};
    return sendCommand(LX16A_SERVO_POS_READ, dummy, 2, true, timeout_ms); // any read cmd works
}

bool LXServo::handleIncoming(uint32_t, uint8_t const* data, uint8_t len, uint32_t now) {
    uint8_t cmd = data[4];
    uint8_t const* payload = &data[5];
    switch (cmd) {
        case LX16A_SERVO_POS_READ: {
            int16_t ticks; memcpy(&ticks, payload, 2);
            // auto oldp = lastStatus_.position;
            lastStatus_.position = ticksToAngle(ticks);
            // lastStatus_.velocity = (lastStatus_.position - oldp) / ((now - lastStatusTime_) / 1000.0f);
            lastStatusTime_ = now;
            return true;
        }
        case LX16A_SERVO_VIN_READ: {
            int16_t mv; memcpy(&mv, payload, 2);
            voltage_ = mv / 1000.0f;
            return true;
        }
        case LX16A_SERVO_TEMP_READ: {
            lastStatus_.temperature = payload[0];
            // Serial.printf("LXServo %d: temperature %.1fC\n", id_, lastStatus_.temperature);
            lastTempRead_ = now;
            return true;
        }
        default:
            DebugPrinter::log("LXServo %d: unhandled reply cmd %d\n", id_, cmd);
            return false;
    }
}

ParseResult LXServo::parsePacket(uint8_t const* data, uint8_t len) {
    uint8_t idx = 0;
    while (idx < len - LX_MIN_PACKET_LENGTH) {
        // Look for packet header
        if (data[idx] != 0x55 || data[idx + 1] != 0x55) {
            ++idx;
            continue;
        }
        const uint8_t pktLen = data[idx + 3];
        if (pktLen < 3 || pktLen > 7) { //check for valid length
            ++idx;
            continue;
        }
        const uint8_t totalLen = pktLen + 3;
        if (len - idx < totalLen) break; // Not enough data yet

        // Check checksum
        uint8_t cksum = 0;
        for (uint16_t i = idx + 2; i < idx + totalLen - 1; i++)
            cksum += data[i];
        if (data[idx + totalLen - 1] != (uint8_t)~cksum) {
            DebugPrinter::log("LXServo: bad chk idx:%d expected 0x%02X, got %02X\n", idx, (uint8_t)~cksum, data[idx + totalLen - 1]);
            ++idx;
            continue;
        }

        uint8_t id = data[idx + 2];
        // uint8_t cmd = data[idx + 4];
        // uint8_t* payload = &data[idx + 5];
        // uint8_t payloadLen = totalLen - 6;
        ParseResult result = {idx, totalLen, id}; //start, len, id
        return result;
        // Remove this packet from buffer
        // idx += totalLen;
    }
    return {0, 0, -1};
}
