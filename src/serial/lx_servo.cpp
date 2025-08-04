#include "lx_servo.h"
#include "../debugprint.h"
#include <string.h>

// LXBus implementation
LXBus::LXBus(SerialInterface* port, uint32_t baud)
 : port_(port), baud_(baud), debug_(false), retryCount_(3) {
}

bool LXBus::write(uint8_t cmd, const uint8_t* params, int param_cnt, uint8_t id) {
    for (int i = 0; i < (retryCount_ == 0 ? 1 : retryCount_); i++) {
        if (writeNoRetry(cmd, params, param_cnt, id))
            return true;
    }
    return false;
}

bool LXBus::read(uint8_t cmd, uint8_t* params, int param_len, uint8_t id) {
    for (int i = 0; i < (retryCount_ == 0 ? 1 : retryCount_); i++) {
        if (readNoRetry(cmd, params, param_len, id))
            return true;
    }
    return false;
}
bool LXBus::writeNoRetry(uint8_t cmd, const uint8_t* params, int param_cnt, uint8_t id) {
    if (param_cnt < 0 || param_cnt > 4) return false;

    int buflen = 6 + param_cnt;
    uint8_t buf[buflen];
    buf[0] = 0x55;
    buf[1] = 0x55;
    buf[2] = id;
    buf[3] = buflen - 3;
    buf[4] = cmd;

    for (int i = 0; i < param_cnt; i++) {
        buf[5 + i] = params[i];
    }

    uint8_t cksum = 0;
    for (int i = 2; i < buflen - 1; i++) {
        cksum += buf[i];
    }
    buf[buflen - 1] = ~cksum;

    // Clear input buffer
    while (port_->available()) {
        port_->read();
    }

    // Send command
    port_->write(buf, buflen);
    port_->flush();

    return true;
}

bool LXBus::readNoRetry(uint8_t cmd, uint8_t* params, int param_len, uint8_t id) {
    if (!writeNoRetry(cmd, nullptr, 0, id)) return false;
    return receive(cmd, params, param_len, id);
}

bool LXBus::receive(uint8_t cmd, uint8_t* params, int param_len, uint8_t id) {
    uint32_t timeout = 50; // 50ms timeout
    uint32_t start = millis();
    int got = 0;
    uint8_t sum = 0;
    int len = 7; // minimum length

    while (got < len && (millis() - start) < timeout) {
        if (port_->available()) {
            uint8_t ch = port_->read();

            switch (got) {
                case 0:
                case 1:
                    if (ch != 0x55) return false;
                    break;
                case 2:
                    if (ch != id && id != 0xFE) return false;
                    break;
                case 3:
                    if (ch < 3 || ch > 7) return false;
                    len = ch + 3;
                    if (len > param_len + 6) return false;
                    break;
                case 4:
                    if (ch != cmd) return false;
                    break;
                default:
                    if (got == len - 1) {
                        return (uint8_t)ch == (uint8_t)~sum;
                    }
                    if (got - 5 >= param_len) return false;
                    params[got - 5] = ch;
                    break;
            }
            if (got > 1) sum += ch;
            got++;
        }
    }
    return false;
}

// LXServo implementation
LXServo::LXServo(uint8_t id, LXBus* bus)
    : id_(id), bus_(bus), lastStatusTime_(0), enabled_(false),
      minAngleDeg_(0), maxAngleDeg_(240), temperature_(0), voltage_(0) {
    lastStatus_.mode = MotorMode::Position;
    lastStatus_.position = 0;
    lastStatus_.velocity = 0;
    lastStatus_.torque = 0;
    lastStatus_.temperature = 0;
}

void LXServo::requestStatus() {
    uint8_t params[2];
    if (bus_->read(LX16A_SERVO_POS_READ, params, 2, id_)) {
        uint16_t pos_ticks = params[0] | (params[1] << 8);
        lastStatus_.position = ticksToAngle(pos_ticks);
        lastStatusTime_ = millis();
        isCommandOk_ = true;
    } else {
        isCommandOk_ = false;
    }
}

void LXServo::setMode(MotorMode mode) {
    switch (mode) {
        case MotorMode::Position:
            // Enable servo mode
            uint8_t servoParams[] = {0, 0, 0, 0};
            bus_->write(LX16A_SERVO_OR_MOTOR_MODE_WRITE, servoParams, 4, id_);

            // Enable torque
            uint8_t enableParams[] = {1};
            bus_->write(LX16A_SERVO_LOAD_OR_UNLOAD_WRITE, enableParams, 1, id_);
            enabled_ = true;
            lastStatus_.mode = MotorMode::Position;
            break;

        case MotorMode::Speed:
            // Set to motor mode with speed
            // Note: LX16A speed control is limited, this is a basic implementation
            lastStatus_.mode = MotorMode::Speed;
            enabled_ = true;
            break;

        default:
            // Disable servo
            uint8_t disableParams[] = {0};
            bus_->write(LX16A_SERVO_LOAD_OR_UNLOAD_WRITE, disableParams, 1, id_);
            enabled_ = false;
            lastStatus_.mode = MotorMode::Disabled;
            break;
    }
}

void LXServo::setSetpoint(MotorMode mode, float value) {
    if (!enabled_) return;

    switch (mode) {
        case MotorMode::Position: {
            // Clamp to limits
            float angle = value;
            if (angle < minAngleDeg_) angle = minAngleDeg_;
            if (angle > maxAngleDeg_) angle = maxAngleDeg_;

            uint16_t ticks = angleToTicks(angle);
            uint16_t time = 0; // Move as fast as possible

            uint8_t params[] = {
                (uint8_t)(ticks & 0xFF),
                (uint8_t)(ticks >> 8),
                (uint8_t)(time & 0xFF),
                (uint8_t)(time >> 8)
            };
            bus_->write(LX16A_SERVO_MOVE_TIME_WRITE, params, 4, id_);
            break;
        }
        case MotorMode::Speed: {
            // Convert speed to motor mode command
            int16_t speed = (int16_t)value;
            uint8_t params[] = {1, 0, (uint8_t)(speed & 0xFF), (uint8_t)(speed >> 8)};
            bus_->write(LX16A_SERVO_OR_MOTOR_MODE_WRITE, params, 4, id_);
            break;
        }
        default:
            break;
    }
}

void LXServo::fetchVBus() {
    uint8_t params[2];
    if (bus_->read(LX16A_SERVO_VIN_READ, params, 2, id_)) {
        voltage_ = (params[0] | (params[1] << 8)) / 1000.0f; // Convert mV to V
    }

    // Also fetch temperature
    uint8_t tempParam[1];
    if (bus_->read(LX16A_SERVO_TEMP_READ, tempParam, 1, id_)) {
        temperature_ = tempParam[0];
        lastStatus_.temperature = temperature_;
    }
}

void LXServo::setAngleLimits(float minDeg, float maxDeg) {
    minAngleDeg_ = minDeg;
    maxAngleDeg_ = maxDeg;

    uint16_t minTicks = angleToTicks(minDeg);
    uint16_t maxTicks = angleToTicks(maxDeg);

    uint8_t params[] = {
        (uint8_t)(minTicks & 0xFF),
        (uint8_t)(minTicks >> 8),
        (uint8_t)(maxTicks & 0xFF),
        (uint8_t)(maxTicks >> 8)
    };
    bus_->write(LX16A_SERVO_ANGLE_LIMIT_WRITE, params, 4, id_);
}

void LXServo::setId(uint8_t newId) {
    uint8_t params[] = {newId};
    if (bus_->write(LX16A_SERVO_ID_WRITE, params, 1, LX16A_BROADCAST_ID)) {
        id_ = newId;
    }
}

uint8_t LXServo::getId() {
    uint8_t params[1];
    if (bus_->read(LX16A_SERVO_ID_READ, params, 1, id_)) {
        return params[0];
    }
    return 0;
}

void LXServo::stop() {
    uint8_t params[1];
    bus_->write(LX16A_SERVO_MOVE_STOP, params, 1, id_);
}

