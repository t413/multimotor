#include "robstride.h"
#include "can_interface.h"
#include <string.h>
#include "debugprint.h"

union RobStridePayload {
    uint8_t bytes[8] = {0};
    uint16_t words[4];
    uint32_t dwords[2];
    float floats[2];
};

RobStrideDriver::RobStrideDriver(uint8_t id, CanInterface* can) : id_(id), can_(can) {}

void RobStrideDriver::send(RobStrideCmdType cmd, uint8_t* data, uint8_t len) {
    uint32_t canId = (uint32_t(cmd) << 24) | (0x1F << 8) | id_;  // cmd | master_id | motor_id
    if (can_) can_->send(canId, data, len, true, false, false);
}

uint16_t RobStrideDriver::floatToUint(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    if (x > x_max) x = x_max;
    else if (x < x_min) x = x_min;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

float RobStrideDriver::uintToFloat(uint16_t x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

void RobStrideDriver::requestStatus() {
    uint8_t data[8] = {0};
    send(RobStrideCmdType::MotorRequest, data, 8);
}

void RobStrideDriver::fetchVBus() {
    // RobStride doesn't have separate VBus command, handled in status
}

void RobStrideDriver::setRobStrideMode(RobStrideCtrlMode mode) {
    uint8_t data[8] = {0};
    data[0] = (uint8_t)mode;
    send(RobStrideCmdType::ControlMode, data, 8);
}

void RobStrideDriver::enableMotor() {
    uint8_t data[8] = {0};
    send(RobStrideCmdType::MotorEnable, data, 8);
    enabled_ = true;
}

void RobStrideDriver::disableMotor() {
    uint8_t data[8] = {0};
    send(RobStrideCmdType::MotorStop, data, 8);
    enabled_ = false;
}

void RobStrideDriver::setMode(MotorMode mode) {
    if (mode == MotorMode::Disabled) {
        disableMotor();
        lastSentMode_ = MotorMode::Disabled;
    } else {
        lastSentMode_ = mode;
        RobStrideCtrlMode robMode = (mode == MotorMode::Speed) ? RobStrideCtrlMode::Speed :
                                   (mode == MotorMode::Current) ? RobStrideCtrlMode::Current :
                                   (mode == MotorMode::Position) ? RobStrideCtrlMode::Position :
                                   RobStrideCtrlMode::MotionControl;
        setRobStrideMode(robMode);
        enableMotor();
    }
}

void RobStrideDriver::motionControl(float position, float velocity, float kp, float kd, float torque) {
    uint8_t data[8];
    uint16_t pos_int = floatToUint(position, ROBSTRIDE_P_MIN, ROBSTRIDE_P_MAX, 16);
    uint16_t vel_int = floatToUint(velocity, ROBSTRIDE_V_MIN, ROBSTRIDE_V_MAX, 12);
    uint16_t kp_int = floatToUint(kp, 0, 500, 12);
    uint16_t kd_int = floatToUint(kd, 0, 5, 12);
    uint16_t torque_int = floatToUint(torque, ROBSTRIDE_T_MIN, ROBSTRIDE_T_MAX, 12);

    data[0] = pos_int >> 8;
    data[1] = pos_int & 0xFF;
    data[2] = vel_int >> 4;
    data[3] = ((vel_int & 0xF) << 4) | (kp_int >> 8);
    data[4] = kp_int & 0xFF;
    data[5] = kd_int >> 4;
    data[6] = ((kd_int & 0xF) << 4) | (torque_int >> 8);
    data[7] = torque_int & 0xFF;

    send(RobStrideCmdType::MotionControl, data, 8);
}

void RobStrideDriver::setSetpoint(MotorMode mode, float value) {
    if (mode == MotorMode::Position) {
        motionControl(value, 0, 50, 1, 0);  // Position with default gains
    } else if (mode == MotorMode::Speed) {
        motionControl(0, value, 0, 1, 0);  // Velocity control
    } else if (mode == MotorMode::Current) {
        motionControl(0, 0, 0, 0, value);  // Torque control
    }
}

void RobStrideDriver::setZeroPosition() {
    uint8_t data[8] = {0};
    send(RobStrideCmdType::SetPosZero, data, 8);
}

bool RobStrideDriver::handleIncoming(uint32_t id, uint8_t* data, uint8_t len, uint32_t now) {
    uint8_t motorId = id & 0xFF;
    if (motorId != id_) return false;

    uint8_t masterId = (id >> 8) & 0xFF;
    RobStrideCmdType cmd = (RobStrideCmdType)((id >> 24) & 0xFF);

    if (cmd == RobStrideCmdType::MotorRequest && len >= 8) {
        // Parse motor status response
        uint16_t pos_int = (data[1] << 8) | data[2];
        uint16_t vel_int = (data[3] << 4) | (data[4] >> 4);
        uint16_t torque_int = ((data[4] & 0xF) << 8) | data[5];
        uint8_t temp_int = data[6];
        uint8_t error = data[7];

        lastStatus_.position = uintToFloat(pos_int, ROBSTRIDE_P_MIN, ROBSTRIDE_P_MAX, 16);
        lastStatus_.velocity = uintToFloat(vel_int, ROBSTRIDE_V_MIN, ROBSTRIDE_V_MAX, 12);
        lastStatus_.torque = uintToFloat(torque_int, ROBSTRIDE_T_MIN, ROBSTRIDE_T_MAX, 12);
        lastStatus_.temperature = (float)temp_int;
        lastStatus_.mode = enabled_ ? lastSentMode_ : MotorMode::Disabled;
        lastFaults_ = error;
        lastStatusTime_ = now;

    } else if (cmd == RobStrideCmdType::ErrorFeedback) {
        lastFaults_ = data[0];
    }

    return true;
}
