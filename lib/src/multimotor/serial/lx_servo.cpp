#include "lx_servo.h"
#include "serial_drive_manager.h"
#include "serial_interface.h"
#include "../debugprint.h"
#include <string.h>

// LX16A Command definitions
#define LX16A_SERVO_MOVE_TIME_WRITE 1
#define LX16A_SERVO_MOVE_TIME_READ 2
#define LX16A_SERVO_MOVE_TIME_WAIT_WRITE 7
#define LX16A_SERVO_MOVE_START 11
#define LX16A_SERVO_MOVE_STOP 12
#define LX16A_SERVO_ID_WRITE 13
#define LX16A_SERVO_ID_READ 14
#define LX16A_SERVO_POS_READ 28
#define LX16A_SERVO_OR_MOTOR_MODE_WRITE 29
#define LX16A_SERVO_LOAD_OR_UNLOAD_WRITE 31
#define LX16A_SERVO_TEMP_READ 26
#define LX16A_SERVO_VIN_READ 27
#define LX16A_SERVO_ANGLE_LIMIT_WRITE 20
#define LX16A_SERVO_ANGLE_LIMIT_READ 21
#define LX16A_BROADCAST_ID 0xFE


#define LX_MIN_PACKET_LENGTH 4


LXServo::LXServo(uint8_t id, SerialDriveManager* bus)
    : id_(id), bus_(bus) {
    bus_->addDrive(this);
}

bool LXServo::send(uint8_t cmd, const uint8_t* params, int param_cnt, uint8_t id) {
    if (param_cnt < 0 || param_cnt > 4) return false;
    auto serial = (bus_? bus_->getInterface() : nullptr);
    if (!serial) return false;

    uint8_t txbuf[32] = {0};
    int buflen = 6 + param_cnt;
    txbuf[0] = 0x55;
    txbuf[1] = 0x55;
    txbuf[2] = id;
    txbuf[3] = buflen - 3;
    txbuf[4] = cmd;

    for (int i = 0; i < param_cnt; i++)
        txbuf[5 + i] = params[i];

    uint8_t cksum = 0;
    for (int i = 2; i < buflen - 1; i++)
        cksum += txbuf[i];
    txbuf[buflen - 1] = ~cksum;

    serial->write(txbuf, buflen);
    return true;
}

void LXServo::sendCommand(uint8_t cmd, const uint8_t* params, int param_cnt, bool expectResponse) {
    if (send(cmd, params, param_cnt, id_)) {
        if (expectResponse) {
            expectedCmd_ = cmd;
            waitingForResponse_ = true;
        }
    }
}

void LXServo::requestStatus() {
    sendCommand(LX16A_SERVO_POS_READ, nullptr, 0, true);
}

void LXServo::setMode(MotorMode mode) {
    switch (mode) {
        case MotorMode::Position: {
            // Enable servo mode
            uint8_t servoParams[] = {0, 0, 0, 0};
            sendCommand(LX16A_SERVO_OR_MOTOR_MODE_WRITE, servoParams, 4, false);

            // Enable torque
            uint8_t enableParams[] = {1};
            sendCommand(LX16A_SERVO_LOAD_OR_UNLOAD_WRITE, enableParams, 1, false);
            enabled_ = true;
            lastStatus_.mode = MotorMode::Position;
            break;
        }
        case MotorMode::Speed: {
            lastStatus_.mode = MotorMode::Speed;
            enabled_ = true;
            break;
        }
        default: {
            // Disable servo
            uint8_t disableParams[] = {0};
            sendCommand(LX16A_SERVO_LOAD_OR_UNLOAD_WRITE, disableParams, 1, false);
            enabled_ = false;
            lastStatus_.mode = MotorMode::Disabled;
            break;
        }
    }
}

void LXServo::setSetpoint(MotorMode mode, float value) {
    if (!enabled_) return;

    if (mode == MotorMode::Position) {
        // Clamp to limits
        float angle = value;
        if (angle < minAngleDeg_) angle = minAngleDeg_;
        if (angle > maxAngleDeg_) angle = maxAngleDeg_;

        uint16_t ticks = angleToTicks(angle);
        uint16_t time = 0; // Move as fast as possible
        movePosTime(ticks, time);
    } else if (mode == MotorMode::Speed) {
        // Convert speed to motor mode command
        moveSpeed((int16_t)value);
    }
}

void LXServo::movePosTime(int16_t ticks, int16_t time) {
    uint8_t params[4];
    memcpy(params, &ticks, 2);
    memcpy(params + 2, &time, 2);
    sendCommand(LX16A_SERVO_MOVE_TIME_WRITE, params, 4, false);
}

void LXServo::moveSpeed(int16_t speed) {
    uint8_t params[4] = {1, 0};
    memcpy(params + 2, &speed, 2);
    sendCommand(LX16A_SERVO_OR_MOTOR_MODE_WRITE, params, 4, false);
}

void LXServo::fetchVBus() {
    sendCommand(LX16A_SERVO_VIN_READ, nullptr, 0, true);
}

void LXServo::stop() {
    uint8_t params[1];
    sendCommand(LX16A_SERVO_MOVE_STOP, params, 1, false);
}

void LXServo::setAngleLimits(float minDeg, float maxDeg) {
    minAngleDeg_ = minDeg;
    maxAngleDeg_ = maxDeg;

    uint16_t minTicks = angleToTicks(minDeg);
    uint16_t maxTicks = angleToTicks(maxDeg);

    uint8_t params[4];
    memcpy(params, &minTicks, 2);
    memcpy(params + 2, &maxTicks, 2);
    sendCommand(LX16A_SERVO_ANGLE_LIMIT_WRITE, params, 4, false);
}

void LXServo::setId(uint8_t newId) {
    uint8_t params[] = {newId};
    sendCommand(LX16A_SERVO_ID_WRITE, params, 1, false);
    id_ = newId; // Optimistically update
}

bool LXServo::handleIncoming(uint32_t inid, uint8_t const* data, uint8_t len, uint32_t now) {
    if (inid != id_) return false;
    uint8_t id = data[2];
    uint8_t pktLen = data[3];
    uint8_t cmd = data[4];
    uint8_t const* payload = &data[5];
    uint8_t payloadLen = pktLen - 3;
    DebugPrinter::log("LXServo: Handling packet ID %d, cmd %d, payloadLen %d\n", id, cmd, payloadLen);
    return true;
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
