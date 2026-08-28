#include "robstride.h"
#include "can_drive_manager.h"
#include <string.h>
#include "../debugprint.h"

union RobStridePayload {
    uint8_t bytes[8] = {0};
    uint16_t words[4];
    uint32_t dwords[2];
    float floats[2];
};

RobStrideDriver::RobStrideDriver(uint8_t id, CanDriveManager* bus, const char* n) : MotorDrive(n), id_(id), bus_(bus) {}

bool RobStrideDriver::send(RobStrideCmdType cmd, const uint8_t* data, uint8_t len, uint16_t extradata, CanSS ss, CanReq rtr) {
    uint32_t canId = (uint32_t(cmd) << 24) | (extradata << 8) | id_;  // cmd | master_id | motor_id
    return bus_? bus_->send(canId, data, len, CanFrame::Extended, ss, rtr) : false;
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

bool RobStrideDriver::requestStatus() {
    uint8_t data[8] = {0};
    return send(RobStrideCmdType::MotorRequest, data, 8, DEFAULT_HOST_ID, CanSS::Retry, CanReq::RequestReply);
}

bool RobStrideDriver::fetchVBus() {
    return true; // RobStride doesn't have separate VBus command, handled in status
}

bool RobStrideDriver::setRobStrideMode(RobStrideCtrlMode mode) {
    uint8_t data[8] = {0};
    data[0] = (uint8_t)mode;
    return send(RobStrideCmdType::ControlMode, data, 8, DEFAULT_HOST_ID, CanSS::Retry, CanReq::Command);
}

bool RobStrideDriver::enable(bool en) {
    uint8_t data[8] = {0};
    auto type = en? RobStrideCmdType::MotorEnable : RobStrideCmdType::MotorStop;
    if (!send(type, data, 8, DEFAULT_HOST_ID, CanSS::Retry, CanReq::Command)) return false;
    enabled_ = en;
    return true;
}

bool RobStrideDriver::setMode(MotorMode mode) {
    bool ret = false;
    if (mode == MotorMode::Disabled) {
        ret = enable(false);
        lastSentMode_ = MotorMode::Disabled;
    } else {
        lastSentMode_ = mode;
        RobStrideCtrlMode robMode = (mode == MotorMode::Speed) ? RobStrideCtrlMode::Speed :
                                   (mode == MotorMode::Current) ? RobStrideCtrlMode::Current :
                                   (mode == MotorMode::Position) ? RobStrideCtrlMode::Position :
                                   RobStrideCtrlMode::MotionControl;
        setRobStrideMode(robMode);
        ret = enable(true);
    }
    return ret;
}

bool RobStrideDriver::motionControl(float position, float velocity, float kp, float kd, float torque) {
    uint8_t data[8];
    uint16_t pos_int = floatToUint(position, ROBSTRIDE_P_MIN, ROBSTRIDE_P_MAX, 16);
    uint16_t vel_int = floatToUint(velocity, ROBSTRIDE_V_MIN, ROBSTRIDE_V_MAX, 12);
    uint16_t kp_int = floatToUint(kp, 0, 500, 12);
    uint16_t kd_int = floatToUint(kd, 0, 5, 12);
    uint16_t torque_int = floatToUint(torque, ROBSTRIDE_T_MIN, ROBSTRIDE_T_MAX, 12);

    data[0] = pos_int >> 8;
    data[1] = pos_int & 0xFF;
    data[2] = vel_int >> 8;
    data[3] = vel_int & 0xFF;
    data[4] = kp_int >> 8;
    data[5] = kp_int & 0xFF;
    data[6] = kd_int >> 8;
    data[7] = kd_int & 0xFF;

    return send(RobStrideCmdType::MotionControl, data, 8, torque_int, CanSS::Singleshot, CanReq::Command);
}

bool RobStrideDriver::setSetpoint(MotorMode mode, float value) {
    if (mode == MotorMode::Position) {
        return motionControl(value, 0, 50, 1, 0);  // Position with default gains
    } else if (mode == MotorMode::Speed) {
        return motionControl(0, value, 0, 1, 0);  // Velocity control
    } else if (mode == MotorMode::Current) {
        return motionControl(0, 0, 0, 0, value);  // Torque control
    }
    return false;
}

bool RobStrideDriver::setZeroPosition() {
    uint8_t data[8] = {0};
    return send(RobStrideCmdType::SetPosZero, data, 8, DEFAULT_HOST_ID, CanSS::Retry, CanReq::Command);
}

bool RobStrideDriver::ping(int timeout_ms) {
    if (!requestStatus() || !bus_) return false;
    CanMessage msg;
    return bus_->waitForReply(msg, timeout_ms * 1000, id_, 0x000000FFu);
}

bool RobStrideDriver::validID(int id) const {
    return id >= 0 && id < 254;
}

MotorDrive* RobStrideDriver::makeDuplicate(int16_t newId) const {
    if (newId < 0) newId = id_;
    return new RobStrideDriver((uint8_t)newId, bus_, "dupe");
}

bool RobStrideDriver::writeNewId(uint8_t newId, bool sendToDrive) {
    bool ret = true;
    if (sendToDrive) {
        ret = send(RobStrideCmdType::SetCanID, nullptr, 0, newId, CanSS::Retry, CanReq::Command);
    }
    id_ = newId;
    return ret;
}

bool RobStrideDriver::handleIncoming(uint32_t id, uint8_t const* data, uint8_t len, uint32_t now) {
    uint8_t motorId = id & 0xFF;
    if (motorId != id_) return false;

    uint16_t extraData = (id >> 8) & 0xFFFF;
    RobStrideCmdType cmd = (RobStrideCmdType)((id >> 24) & 0xFF);

    if (cmd == RobStrideCmdType::MotorRequest && len >= 8) {
        // Parse motor status response
        uint16_t pos_int = (data[0] << 8) | data[1];
        uint16_t vel_int = (data[2] << 8) | data[3];
        uint16_t torque_int = (data[4] << 8) | data[5];
        uint8_t temp_int = data[6];
        uint8_t error = data[7];

        lastStatus_.position = uintToFloat(pos_int, ROBSTRIDE_P_MIN, ROBSTRIDE_P_MAX, 16);
        lastStatus_.velocity = uintToFloat(vel_int, ROBSTRIDE_V_MIN, ROBSTRIDE_V_MAX, 16);
        lastStatus_.torque = uintToFloat(torque_int, ROBSTRIDE_T_MIN, ROBSTRIDE_T_MAX, 16);
        lastStatus_.temperature = (float)temp_int;
        lastStatus_.mode = enabled_ ? lastSentMode_ : MotorMode::Disabled;
        lastFaults_ = error;
        lastStatusTime_ = now;

    } else if (cmd == RobStrideCmdType::ErrorFeedback) {
        lastFaults_ = data[0];
    }

    return true;
}
