#include "robstride.h"
#include "can_drive_manager.h"
#include <string.h>
#include "../debugprint.h"

enum class RSCmd : uint8_t {
    GetID = 0x00,
    MotionControl = 0x01,
    MotorRequest = 0x02,
    MotorEnable = 0x03,
    MotorStop = 0x04,
    SetPosZero = 0x06,
    SetCanID = 0x07,
    WriteParamLower =  0x8,  // params 0x0000 - 0x302F
    ReadParamLower  =  0x9,  // params 0x0000 - 0x302F
    GetSingleParam = 0x11,
    SetSingleParam = 0x12,
    ErrorFeedback = 0x15,
    SaveData = 0x16,
};

enum class RSParams : uint16_t {
    AddrVBUSmv   = 0x3007, //uint16_t, millivolts
    AddrVBUSfloat= 0x302B,
    PARAM_UPPER_ADDR = 0x7000,  // Threshold for upper/lower commands
};

RobStrideDriver::RobStrideDriver(uint8_t id, CanDriveManager* bus, const char* n) : MotorDrive(n, bus), id_(id % (MAX_ID + 1)), bus_(bus) {}

void RobStrideDriver::setScales(float pmax, float vmax, float tmax) {
    scalePMax_ = pmax;
    scaleVMax_ = vmax;
    scaleTMax_ = tmax;
}

bool RobStrideDriver::send(RSCmd cmd, const uint8_t* data, uint8_t len, uint16_t extradata, CanSS ss, CanReq rtr) {
    uint32_t canId = (uint32_t(cmd) << 24) | (extradata << 8) | id_;  // cmd | master_id | motor_id
    if (lastCommsTime_ == 0) ss = CanSS::Singleshot; //force no-retry until we've heard anything back
    return bus_? bus_->send(canId, data, len, CanFrame::Extended, ss, rtr) : false;
}

bool RobStrideDriver::requestStatus() {
    uint8_t data[8] = {0};
    if (lastCommsTime_ == 0)
        return send(RSCmd::GetID, data, 8, DEFAULT_HOST_ID, CanSS::Singleshot);
    else return send(RSCmd::MotorRequest, data, 8, DEFAULT_HOST_ID, CanSS::Retry);
}

bool RobStrideDriver::fetchVBus() {
    return reqParam((uint16_t)RSParams::AddrVBUSfloat);
}

bool RobStrideDriver::enable(bool en) {
    uint8_t data[8] = {0};
    auto type = en? RSCmd::MotorEnable : RSCmd::MotorStop;
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
        ret = enable(true);
        if (ret) lastSentMode_ = mode;
    }
    return ret;
}

bool RobStrideDriver::mitTarget(float position, float velocity, float kp, float kd, float torque) {
    uint16_t pos_int = floatToUint(position, -scalePMax_, scalePMax_, 16);
    uint16_t vel_int = floatToUint(velocity, -scaleVMax_, scaleVMax_, 16);
    uint16_t kp_int = floatToUint(kp, 0, 500, 16);
    uint16_t kd_int = floatToUint(kd, 0, 5, 16);
    uint16_t torque_int = floatToUint(torque, -scaleTMax_, scaleTMax_, 16);

    uint8_t data[8] = {
        static_cast<uint8_t>(pos_int >> 8), static_cast<uint8_t>(pos_int & 0xFF),
        static_cast<uint8_t>(vel_int >> 8), static_cast<uint8_t>(vel_int & 0xFF),
        static_cast<uint8_t>(kp_int >> 8),  static_cast<uint8_t>(kp_int & 0xFF),
        static_cast<uint8_t>(kd_int >> 8),  static_cast<uint8_t>(kd_int & 0xFF),
    };

    return send(RSCmd::MotionControl, data, 8, torque_int, CanSS::Singleshot, CanReq::Command);
}

bool RobStrideDriver::setSetpoint(MotorMode mode, float value) {
    if (mode == MotorMode::Position) {
        return mitTarget(value, 0, 50, 1, 0);  // Position with default gains
    } else if (mode == MotorMode::Speed) {
        return mitTarget(0, value, 0, 1, 0);  // Velocity control
    } else if (mode == MotorMode::Current) {
        return mitTarget(0, 0, 0, 0, value);  // Torque control
    }
    return false;
}

bool RobStrideDriver::setZeroPosition() {
    uint8_t data[8] = {0};
    return send(RSCmd::SetPosZero, data, 8, DEFAULT_HOST_ID, CanSS::Retry, CanReq::Command);
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
        ret = send(RSCmd::SetCanID, zeros, 8, extra, CanSS::Singleshot);
    }
    id_ = newId;
    return ret;
}

bool RobStrideDriver::setZero() {
    uint8_t data[8] = {1, 0};
    return send(RSCmd::SetPosZero, data, 8);
}

bool RobStrideDriver::saveSettings() {
    uint8_t data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    return send(RSCmd::SaveData, data, 8);
}

bool RobStrideDriver::reqParam(uint16_t paramId) {
    uint8_t data[8] = { (uint8_t)(paramId & 0xFF), (uint8_t)((paramId >> 8) & 0xFF) };
    auto cmd = paramId < (uint16_t)RSParams::PARAM_UPPER_ADDR ? RSCmd::ReadParamLower : RSCmd::GetSingleParam;
    return send(cmd, data, 8, DEFAULT_HOST_ID, CanSS::Retry);
}

bool RobStrideDriver::handleIncoming(uint32_t id, uint8_t const* data, uint8_t len, uint32_t now) {
    uint8_t hostid = id & 0xFF;
    uint8_t driveid = (id >> 8) & 0xFF;
    uint8_t extra = (id >> 16) & 0xFF;
    RSCmd cmd = (RSCmd)((id >> 24) & 0xFF);
    if (driveid != id_) return false;

    if (cmd == RSCmd::MotorRequest && len >= 8) {
        // Parse motor status response
        uint16_t pos_int = (data[0] << 8) | data[1];
        uint16_t vel_int = (data[2] << 8) | data[3];
        uint16_t torque_int = (data[4] << 8) | data[5];
        uint8_t temp_int = (data[6] << 8) | data[7];
        lastFaults_ = (id >> 16) & 0x3F; //bits 16~21 = [Uncalibrated, hall, magsense, overtemp, overcurrent, undervolt]
        uint8_t runmode = (id >> 22) & 0x3; //bits 22~23, 0 disabled, 1 position, 2 speed, 3 current

        lastStatus_.position = uintToFloat(pos_int, -scalePMax_, scalePMax_, 16);
        lastStatus_.velocity = uintToFloat(vel_int, -scaleVMax_, scaleVMax_, 16);
        lastStatus_.torque = uintToFloat(torque_int, -scaleTMax_, scaleTMax_, 16);
        lastStatus_.temperature = (float)temp_int / 10.0f;
        lastStatus_.mode = (runmode == 0) ? MotorMode::Disabled : (runmode == 1) ? MotorMode::Position : (runmode == 2) ? MotorMode::Speed : (runmode == 3) ? MotorMode::Current : MotorMode::Unknown;
        lastStatusTime_ = now;

    } else if (cmd == RSCmd::ErrorFeedback) {
        lastFaults_ = data[0];
    } else if (cmd == RSCmd::GetID) {
        // update serial_ with payload
        memcpy(serial_, data, len < 8 ? len : 8);
        // Serial.printf("RS ID %x: Serial: %02X%02X%02X%02X%02X%02X%02X%02X\n", id_, serial_[0], serial_[1], serial_[2], serial_[3], serial_[4], serial_[5], serial_[6], serial_[7]);
    } else if (cmd == RSCmd::GetSingleParam || cmd == RSCmd::ReadParamLower) {
        // uint16_t paramid = (data[1] << 8) | data[0];
        auto paramid = reinterpret_cast<const uint16_t&>(data[0]);
        auto fval = reinterpret_cast<const float&>(data[4]);
        // Serial.printf("RS ID %x: Param 0x%04X = 0x%08X (%0.1f)\n", id_, paramid, ival, fval);
        if (paramid == (uint16_t)RSParams::AddrVBUSfloat) {
            lastVBus_ = fval;
        } else if (paramid == (uint16_t)RSParams::AddrVBUSmv) {
            lastVBus_ = (data[4] | (data[5] << 8)) / 1000.0f; // Convert millivolts to volts
        }
    }

    lastCommsTime_ = now;
    return true;
}
