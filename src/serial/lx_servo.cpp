#include "lx_servo.h"
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

// LXBus implementation
LXBus::LXBus(SerialInterface* port)
    : port_(port), debug_(false) {
}

bool LXBus::send(uint8_t cmd, const uint8_t* params, int param_cnt, uint8_t id) {
    if (param_cnt < 0 || param_cnt > 4) return false;

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

    port_->write(txbuf, buflen);
    return true;
}

// LXServo implementation
LXServo::LXServo(uint8_t id, LXBus* bus)
    : id_(id), bus_(bus), lastStatusTime_(0), lastFaults_(0), enabled_(false),
      minAngleDeg_(0), maxAngleDeg_(240), temperature_(0), voltage_(0),
      expectedCmd_(0), expectedId_(0), lastRequestTime_(0), waitingForResponse_(false),
      parseState_(0), packetIndex_(0), packetLength_(0), checksum_(0) {
    lastStatus_.mode = MotorMode::Position;
    lastStatus_.position = 0;
    lastStatus_.velocity = 0;
    lastStatus_.torque = 0;
    lastStatus_.temperature = 0;
    resetParser();
}

void LXServo::resetParser() {
    parseState_ = 0;
    packetIndex_ = 0;
    packetLength_ = 0;
    checksum_ = 0;
}

void LXServo::sendCommand(uint8_t cmd, const uint8_t* params, int param_cnt, bool expectResponse) {
    if (bus_->send(cmd, params, param_cnt, id_)) {
        if (expectResponse) {
            expectedCmd_ = cmd;
            expectedId_ = id_;
            waitingForResponse_ = true;
            lastRequestTime_ = millis();
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
            sendCommand(LX16A_SERVO_MOVE_TIME_WRITE, params, 4, false);
            break;
        }
        case MotorMode::Speed: {
            // Convert speed to motor mode command
            int16_t speed = (int16_t)value;
            uint8_t params[] = {1, 0, (uint8_t)(speed & 0xFF), (uint8_t)(speed >> 8)};
            sendCommand(LX16A_SERVO_OR_MOTOR_MODE_WRITE, params, 4, false);
            break;
        }
        default:
            break;
    }
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

    uint8_t params[] = {
        (uint8_t)(minTicks & 0xFF),
        (uint8_t)(minTicks >> 8),
        (uint8_t)(maxTicks & 0xFF),
        (uint8_t)(maxTicks >> 8)
    };
    sendCommand(LX16A_SERVO_ANGLE_LIMIT_WRITE, params, 4, false);
}

void LXServo::setId(uint8_t newId) {
    uint8_t params[] = {newId};
    sendCommand(LX16A_SERVO_ID_WRITE, params, 1, false);
    id_ = newId; // Optimistically update
}

bool LXServo::processPacket() {
    if (packetIndex_ < 6) return false; // Minimum packet size

    // Verify checksum
    uint8_t calculatedChecksum = 0;
    for (int i = 2; i < packetIndex_ - 1; i++) {
        calculatedChecksum += packetBuffer_[i];
    }
    if (packetBuffer_[packetIndex_ - 1] != (uint8_t)~calculatedChecksum) {
        return false;
    }

    uint8_t id = packetBuffer_[2];
    uint8_t cmd = packetBuffer_[4];
    uint8_t* params = &packetBuffer_[5];
    int param_len = packetIndex_ - 6;

    // Only process if it's for us
    if (id != id_) return false;

    auto* debug = DebugPrinter::getPrinter();

    switch (cmd) {
        case LX16A_SERVO_POS_READ:
            if (param_len >= 2) {
                uint16_t pos_ticks = params[0] | (params[1] << 8);
                lastStatus_.position = ticksToAngle(pos_ticks);
                lastStatusTime_ = millis();
                if (debug && debug->availableForWrite()) {
                    debug->printf("LX servo %d: position = %.2f deg\n", id_, lastStatus_.position);
                }
            }
            break;

        case LX16A_SERVO_VIN_READ:
            if (param_len >= 2) {
                voltage_ = (params[0] | (params[1] << 8)) / 1000.0f;
                if (debug && debug->availableForWrite()) {
                    debug->printf("LX servo %d: voltage = %.2f V\n", id_, voltage_);
                }
            }
            break;

        case LX16A_SERVO_TEMP_READ:
            if (param_len >= 1) {
                temperature_ = params[0];
                lastStatus_.temperature = temperature_;
                if (debug && debug->availableForWrite()) {
                    debug->printf("LX servo %d: temperature = %.1f C\n", id_, temperature_);
                }
            }
            break;

        default:
            if (debug && debug->availableForWrite()) {
                debug->printf("LX servo %d: unknown response cmd %d\n", id_, cmd);
            }
            break;
    }

    return true;
}

bool LXServo::handleIncoming(uint32_t id, uint8_t* data, uint8_t len, uint32_t now) {
    // Process all available bytes from the serial port
    while (bus_->available()) {
        uint8_t byte = bus_->read();

        switch (parseState_) {
            case 0: // Wait for first header byte
                if (byte == 0x55) {
                    packetBuffer_[0] = byte;
                    parseState_ = 1;
                    packetIndex_ = 1;
                }
                break;

            case 1: // Wait for second header byte
                if (byte == 0x55) {
                    packetBuffer_[1] = byte;
                    parseState_ = 2;
                    packetIndex_ = 2;
                } else {
                    resetParser();
                }
                break;

            case 2: // Get ID
                packetBuffer_[packetIndex_++] = byte;
                parseState_ = 3;
                break;

            case 3: // Get length
                if (byte < 3 || byte > 7) {
                    resetParser();
                } else {
                    packetBuffer_[packetIndex_++] = byte;
                    packetLength_ = byte + 3; // Total packet length
                    parseState_ = 4;
                }
                break;

            case 4: // Collect remaining bytes
                packetBuffer_[packetIndex_++] = byte;
                if (packetIndex_ >= packetLength_) {
                    // Complete packet received
                    bool processed = processPacket();
                    resetParser();
                    if (processed) {
                        waitingForResponse_ = false;
                        return true;
                    }
                }
                if (packetIndex_ >= sizeof(packetBuffer_)) {
                    resetParser(); // Buffer overflow protection
                }
                break;
        }
    }

    // Check for timeout on expected responses
    if (waitingForResponse_ && (now - lastRequestTime_) > 100) { // 100ms timeout
        waitingForResponse_ = false;
        lastFaults_ |= 0x01; // Set communication timeout fault
        auto* debug = DebugPrinter::getPrinter();
        if (debug && debug->availableForWrite()) {
            debug->printf("LX servo %d: response timeout\n", id_);
        }
    }

    return false;
}

