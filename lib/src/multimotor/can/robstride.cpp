#include "robstride.h"
#include "can_drive_manager.h"
#include <string.h>
#include <math.h>
#include "../debugprint.h"

union RobStridePayload {
    uint8_t bytes[8] = {0};
    uint16_t words[4];
    uint32_t dwords[2];
    float floats[2];
};

constexpr float ROBSTRIDE_P_MAX =  4 * M_PI;
constexpr float ROBSTRIDE_P_MIN = -ROBSTRIDE_P_MAX;
constexpr float ROBSTRIDE_V_MAX = 50.0f; //rad/s
constexpr float ROBSTRIDE_V_MIN = -ROBSTRIDE_V_MAX;
constexpr float ROBSTRIDE_T_MAX = 6.0f;
constexpr float ROBSTRIDE_T_MIN = -ROBSTRIDE_T_MAX;

enum class RobStrideParams : uint16_t {
    VBUS = 0x701C,
};

RobStrideDriver::RobStrideDriver(uint8_t id, CanDriveManager* bus, const char* n) : MotorDrive(n), id_(id % (MAX_ID + 1)), bus_(bus) {}

bool RobStrideDriver::send(RobStrideCmdType cmd, const uint8_t* data, uint8_t len, uint16_t extradata, CanSS ss, CanReq rtr) {
    uint32_t canId = (uint32_t(cmd) << 24) | (extradata << 8) | id_;  // cmd | master_id | motor_id
    if (lastCommsTime_ == 0) ss = CanSS::Singleshot; //force no-retry until we've heard anything back
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
    if (lastCommsTime_ == 0)
        return send(RobStrideCmdType::GetID, data, 8, DEFAULT_HOST_ID, CanSS::Singleshot);
    else return send(RobStrideCmdType::MotorRequest, data, 8, DEFAULT_HOST_ID, CanSS::Retry);
}

bool RobStrideDriver::fetchVBus() {
    return reqParam((uint16_t)RobStrideParams::VBUS);
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
        RobStrideCtrlMode robMode = (mode == MotorMode::Speed) ? RobStrideCtrlMode::Speed :
                                   (mode == MotorMode::Current) ? RobStrideCtrlMode::Current :
                                   (mode == MotorMode::Position) ? RobStrideCtrlMode::Position :
                                   RobStrideCtrlMode::MotionControl;
        setRobStrideMode(robMode);
        ret = enable(true);
        if (ret) lastSentMode_ = mode;
    }
    return ret;
}

bool RobStrideDriver::motionControl(float position, float velocity, float kp, float kd, float torque) {
    uint16_t pos_int = floatToUint(position, ROBSTRIDE_P_MIN, ROBSTRIDE_P_MAX, 16);
    uint16_t vel_int = floatToUint(velocity, ROBSTRIDE_V_MIN, ROBSTRIDE_V_MAX, 16);
    uint16_t kp_int = floatToUint(kp, 0, 500, 16);
    uint16_t kd_int = floatToUint(kd, 0, 5, 16);
    uint16_t torque_int = floatToUint(torque, ROBSTRIDE_T_MIN, ROBSTRIDE_T_MAX, 16);

    uint8_t data[8] = {
        static_cast<uint8_t>(pos_int >> 8), static_cast<uint8_t>(pos_int & 0xFF),
        static_cast<uint8_t>(vel_int >> 8), static_cast<uint8_t>(vel_int & 0xFF),
        static_cast<uint8_t>(kp_int >> 8),  static_cast<uint8_t>(kp_int & 0xFF),
        static_cast<uint8_t>(kd_int >> 8),  static_cast<uint8_t>(kd_int & 0xFF),
    };

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
    return bus_->waitForReply(msg, timeout_ms * 1000, id_ << 8, 0x0000FF00u);
}

bool RobStrideDriver::validID(int id) const {
    return id >= 0 && id <= MAX_ID;
}

MotorDrive* RobStrideDriver::makeDuplicate(int16_t newId) const {
    if (newId < 0) newId = DEFAULT_ID;
    return new RobStrideDriver((uint8_t)newId, bus_, "dupe");
}

bool RobStrideDriver::writeNewId(uint8_t newId, bool sendToDrive) {
    bool ret = true;
    newId = newId % (MAX_ID + 1);
    if (sendToDrive) {
        const uint8_t zeros[8] = {0};
        uint16_t extra = (newId << 8) | DEFAULT_HOST_ID;
        ret = send(RobStrideCmdType::SetCanID, zeros, 8, extra, CanSS::Singleshot);
    }
    id_ = newId;
    return ret;
}

bool RobStrideDriver::setZero() {
    uint8_t data[8] = {1, 0};
    return send(RobStrideCmdType::SetPosZero, data, 8);
}

bool RobStrideDriver::saveSettings() {
    uint8_t data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    return send(RobStrideCmdType::SaveData, data, 8);
}

bool RobStrideDriver::reqParam(uint16_t paramId) {
    uint8_t data[8] = { (uint8_t)(paramId & 0xFF), (uint8_t)((paramId >> 8) & 0xFF) };
    return send(RobStrideCmdType::GetSingleParam, data, 8, DEFAULT_HOST_ID, CanSS::Retry);
}

bool RobStrideDriver::handleIncoming(uint32_t id, uint8_t const* data, uint8_t len, uint32_t now) {
    uint8_t hostid = id & 0xFF;
    uint8_t driveid = (id >> 8) & 0xFF;
    uint8_t extra = (id >> 16) & 0xFF;
    RobStrideCmdType cmd = (RobStrideCmdType)((id >> 24) & 0xFF);
    if (driveid != id_) return false;

    if (cmd == RobStrideCmdType::MotorRequest && len >= 8) {
        // Parse motor status response
        uint16_t pos_int = (data[0] << 8) | data[1];
        uint16_t vel_int = (data[2] << 8) | data[3];
        uint16_t torque_int = (data[4] << 8) | data[5];
        uint8_t temp_int = (data[6] << 8) | data[7];
        lastFaults_ = (id >> 16) & 0x3F; //bits 16~21
        uint8_t runmode = (id >> 22) & 0x3; //bits 22~23

        lastStatus_.position = uintToFloat(pos_int, ROBSTRIDE_P_MIN, ROBSTRIDE_P_MAX, 16);
        lastStatus_.velocity = uintToFloat(vel_int, ROBSTRIDE_V_MIN, ROBSTRIDE_V_MAX, 16);
        lastStatus_.torque = uintToFloat(torque_int, ROBSTRIDE_T_MIN, ROBSTRIDE_T_MAX, 16);
        lastStatus_.temperature = (float)temp_int / 10.0f;
        lastStatus_.mode = runmode ? lastSentMode_ : MotorMode::Disabled;
        lastStatusTime_ = now;

    } else if (cmd == RobStrideCmdType::ErrorFeedback) {
        lastFaults_ = data[0];
    } else if (cmd == RobStrideCmdType::GetID) {
        // update serial_ with payload
        memcpy(serial_, data, len < 8 ? len : 8);
        // Serial.printf("RS ID %x: Serial: %02X%02X%02X%02X%02X%02X%02X%02X\n", id_, serial_[0], serial_[1], serial_[2], serial_[3], serial_[4], serial_[5], serial_[6], serial_[7]);
    } else if (cmd == RobStrideCmdType::GetSingleParam) {
        // uint16_t paramid = (data[1] << 8) | data[0];
        auto paramid = reinterpret_cast<const uint16_t&>(data[0]);
        auto ival = reinterpret_cast<const uint32_t&>(data[4]);
        auto fval = reinterpret_cast<const float&>(data[4]);
        // Serial.printf("RS ID %x: Param 0x%04X = 0x%08X (%0.1f)\n", id_, paramid, ival, fval);
        if (paramid == (uint16_t)RobStrideParams::VBUS) {
            lastVBus_ = fval;
        }
    }

    lastCommsTime_ = now;
    return true;
}
