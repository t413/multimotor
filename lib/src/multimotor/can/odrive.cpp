#include "odrive.h"
#include "can_drive_manager.h"
#include "../debugprint.h"
#include <string.h>

enum class CmdIDs : uint8_t {
    GetVersion            = 0x00,
    Heartbeat             = 0x01,
    Estop                 = 0x02,
    GetError              = 0x03,
    RxSdo                 = 0x04,
    TxSdo                 = 0x05,
    Address               = 0x06,
    SetAxisState          = 0x07,
    GetEncoderEstimates   = 0x09,
    SetControllerMode     = 0x0b,
    SetInputPos           = 0x0c,
    SetInputVel           = 0x0d,
    SetInputTorque        = 0x0e,
    SetLimits             = 0x0f,
    SetTrajVelLimit       = 0x11,
    SetTrajAccelLimits    = 0x12,
    SetTrajInertia        = 0x13,
    GetIq                 = 0x14,
    GetTemperature        = 0x15,
    Reboot                = 0x16,
    GetBusVoltageCurrent  = 0x17,
    ClearErrors           = 0x18,
    SetAbsolutePosition   = 0x19,
    SetPosGain            = 0x1a,
    SetVelGains           = 0x1b,
    GetTorques            = 0x1c,
    GetPowers             = 0x1d,
};

ODriveDriver::ODriveDriver(uint8_t id, CanDriveManager* bus, const char* n) : MotorDrive(n), id_(id), bus_(bus) { }

uint16_t mkID(uint8_t id, CmdIDs cmd) {
    return (id << 5) | (uint16_t) cmd;
}

bool ODriveDriver::send(CmdIDs cmd, uint8_t* data, uint8_t len, CanSS ss, CanReq rtr) {
    return bus_? bus_->send(mkID(id_, (CmdIDs) cmd), data, len, CanFrame::Standard, ss, rtr) : false;
}

//get different combinations of payload
union Payload {
    uint8_t  bytes[8] = {0};
    uint16_t words[4];
    uint32_t dwords[2];
    float    floats[2];
};

bool ODriveDriver::requestStatus() {
    return send(CmdIDs::GetEncoderEstimates, NULL, 0, CanSS::Singleshot, CanReq::RequestReply); //single shot, request-to-receive
}

bool ODriveDriver::fetchVBus() {
    return send(CmdIDs::GetBusVoltageCurrent, NULL, 0, CanSS::Singleshot, CanReq::RequestReply);
}

bool ODriveDriver::ping(int timeout_ms) {
    if (! requestStatus()) return false;
    CanMessage msg;
    return bus_->waitForReply(msg, timeout_ms * 1000, id_);
}

bool ODriveDriver::setOdriveMode(OdriveCtrlMode mode) {
    Payload p;
    p.dwords[0] = (uint32_t) mode;
    p.dwords[1] = 1; //passtrough input mode
    return send(CmdIDs::SetControllerMode, p.bytes, 8, CanSS::Retry); //no single shot (enables retries)
}

bool ODriveDriver::clearErrors() {
    Payload p;
    p.dwords[0] = 1; // Clear all errors
    return send(CmdIDs::ClearErrors, p.bytes, 8, CanSS::Retry); //no single shot (enables retries)
}

bool ODriveDriver::setOdriveEnable(bool enable) {
    if (lastFaults_) {
        DebugPrinter::log("ODrive %d faults %d detected, clearing first\n", id_, lastFaults_);
        clearErrors(); //clear errors before enabling
    }
    Payload p;
    p.dwords[0] = enable ? 8 : 1; //Closed loop control
    return send(CmdIDs::SetAxisState, p.bytes, 8, CanSS::Retry); //no single shot (enables retries)
}

bool ODriveDriver::setMode(MotorMode mode) {
    bool ret = false;
    if (mode == MotorMode::Disabled) {
        ret = setOdriveEnable(false); //disable before setting mode
        lastSentMode_ = MotorMode::Disabled;
    } else {
        lastSentMode_ = mode;
        OdriveCtrlMode odriveMode = (mode == MotorMode::Speed) ? OdriveCtrlMode::Velocity :
                       (mode == MotorMode::Current) ? OdriveCtrlMode::Torque :
                       OdriveCtrlMode::Position;
        ret = setOdriveMode(odriveMode);
        ret &= setOdriveEnable(true);
    }
    return ret;
}

bool ODriveDriver::setSetpoint(MotorMode mode, float value) {
    Payload p;
    p.floats[0] = value;
    CmdIDs cmd = (mode == MotorMode::Position) ? CmdIDs::SetInputPos :
                 (mode == MotorMode::Speed)    ? CmdIDs::SetInputVel : CmdIDs::SetInputTorque;
    return send(cmd, p.bytes);
}

bool ODriveDriver::handleIncoming(uint32_t id, uint8_t const* data, uint8_t len, uint32_t now) {
    uint8_t inCanId = id >> 5;
    if (inCanId != id_) return false;
    CmdIDs cmd = (CmdIDs) (id & 0x1F);
    Payload p;
    memcpy(p.bytes, data, len);
    if (cmd == CmdIDs::GetEncoderEstimates) { //default ever 10ms
        lastStatus_.position = p.floats[0];
        lastStatus_.velocity = p.floats[1];
        lastStatusTime_ = now;
    } else if (cmd == CmdIDs::GetBusVoltageCurrent) {
        lastVolt_ = p.floats[0];
        lastCurr_ = p.floats[1];
        lastBusVoltTime_ = now;
    } else if (cmd == CmdIDs::Heartbeat) {
        lastFaults_ = p.dwords[0];
        lastAxisState_ = p.bytes[4];
        lastHeartbeatTime_ = now;
        auto state = (OdriveAxisState)lastAxisState_;
        if (state == OdriveAxisState::Idle) {
            lastStatus_.mode = MotorMode::Disabled;
        } else if (state == OdriveAxisState::ClosedLoopControl) {
            lastStatus_.mode = lastSentMode_;
        } else {
            lastStatus_.mode = MotorMode::Unknown;
        }
    } else {
        auto* debug = DebugPrinter::getPrinter();
        if (debug && debug->availableForWrite()) {
            debug->printf(" > o-rx cmd %x len %d: {", (uint8_t) cmd, len);
            for (int i = 0; i < len; i++)
                debug->printf("0x%02x, ", data[i]);
            debug->println("}");
        }
    }
    return true;
}

MotorDrive* ODriveDriver::makeDuplicate(int16_t newId) const {
    if (newId < 0) newId = DEFAULT_ID;
    return new ODriveDriver(newId, bus_, "dupe");
}

bool ODriveDriver::writeNewId(uint8_t newid, bool sendToDrive) {
    if (newid > DEFAULT_ID) {
        return false;
    }
    bool ret = false;
    if (sendToDrive) {
        ret = send(CmdIDs::Address, &newid, 1, CanSS::Retry, CanReq::RequestReply);
    } else ret = true;
    id_ = newid;
    return ret;
}

