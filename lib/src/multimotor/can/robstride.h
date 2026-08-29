#pragma once
#include "../motordrive.h"
#include "../can/can_interface.h"

class CanDriveManager;

enum class RobStrideCtrlMode : uint8_t {
    MotionControl = 0,
    Position = 1,
    Speed = 2,
    Current = 3,
    SetZero = 4
};

enum class RobStrideCmdType : uint8_t {
    GetID = 0x00,
    MotionControl = 0x01,
    MotorRequest = 0x02,
    MotorEnable = 0x03,
    MotorStop = 0x04,
    SetPosZero = 0x06,
    SetCanID = 0x07,
    ControlMode = 0x12,
    GetSingleParam = 0x11,
    SetSingleParam = 0x12,
    ErrorFeedback = 0x15
};

class RobStrideDriver : public MotorDrive {
    uint8_t id_ = 0;
    CanDriveManager* bus_ = nullptr;
    uint32_t lastFaults_ = 0;
    uint32_t lastStatusTime_ = 0;
    uint32_t lastCommsTime_ = 0;
    float lastVBus_ = 0.0f;
    MotorState lastStatus_;
    MotorMode lastSentMode_ = MotorMode::Disabled;
    bool enabled_ = false;
    uint8_t serial_[8] = {0};

public:
    RobStrideDriver(uint8_t id, CanDriveManager* bus, const char* name);
    static constexpr uint8_t DEFAULT_ID = 0x7D;
    static constexpr uint8_t DEFAULT_HOST_ID = 0xFE;

    // MotorDrive interface implementation
    uint32_t getId() const override { return id_; }
    bool requestStatus() override;
    bool setMode(MotorMode mode) override;
    bool setSetpoint(MotorMode mode, float value) override;
    bool handleIncoming(uint32_t id, uint8_t const* data, uint8_t len, uint32_t now) override;
    uint32_t getLastStatusTime() const override { return lastStatusTime_; }
    uint32_t getLastFaults() const override { return lastFaults_; }
    MotorState getMotorState() const override { return lastStatus_; }
    bool fetchVBus() override;
    float getVBus() const override { return lastVBus_; }
    bool ping(int timeout_ms = 100) override;
    bool validID(int id) const override;
    MotorDrive* makeDuplicate(int16_t id = -1) const override;
    bool writeNewId(uint8_t newId, bool sendToDrive = true) override;

    // RobStride specific methods
    bool setRobStrideMode(RobStrideCtrlMode mode);
    bool enable(bool en = true);
    bool setZeroPosition();
    bool motionControl(float position, float velocity, float kp, float kd, float torque);
    bool reqParam(uint16_t paramId);

private:
    bool send(RobStrideCmdType cmd, const uint8_t* data, uint8_t len, uint16_t extradata = DEFAULT_HOST_ID, CanSS ss = CanSS::Singleshot, CanReq rtr = CanReq::Command);
    static uint16_t floatToUint(float x, float x_min, float x_max, int bits);
    static float uintToFloat(uint16_t x_int, float x_min, float x_max, int bits);
};
