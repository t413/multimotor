#pragma once
#include "motordrive.h"

class CanInterface;

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

// Limits for RobStride motor
#define ROBSTRIDE_P_MIN -12.5f
#define ROBSTRIDE_P_MAX 12.5f
#define ROBSTRIDE_V_MIN -44.0f
#define ROBSTRIDE_V_MAX 44.0f
#define ROBSTRIDE_T_MIN -17.0f
#define ROBSTRIDE_T_MAX 17.0f

class RobStrideDriver : public MotorDrive {
    uint8_t id_ = 0;
    CanInterface* can_ = nullptr;
    uint32_t lastFaults_ = 0;
    uint32_t lastStatusTime_ = 0;
    uint32_t lastBusVoltTime_ = 0;
    float lastVBus_ = 0.0f;
    MotorState lastStatus_;
    MotorMode lastSentMode_ = MotorMode::Disabled;
    bool enabled_ = false;

public:
    RobStrideDriver(uint8_t id, CanInterface* can);

    // MotorDrive interface implementation
    void requestStatus() override;
    void setMode(MotorMode mode) override;
    void setSetpoint(MotorMode mode, float value) override;
    bool handleIncoming(uint32_t id, uint8_t* data, uint8_t len, uint32_t now) override;
    uint32_t getLastStatusTime() const override { return lastStatusTime_; }
    uint32_t getLastFaults() const override { return lastFaults_; }
    MotorState getMotorState() const override { return lastStatus_; }
    void fetchVBus() override;
    float getVBus() const override { return lastVBus_; }

    // RobStride specific methods
    void setRobStrideMode(RobStrideCtrlMode mode);
    void enableMotor();
    void disableMotor();
    void setZeroPosition();
    void motionControl(float position, float velocity, float kp, float kd, float torque);

private:
    void send(RobStrideCmdType cmd, uint8_t* data, uint8_t len);
    uint16_t floatToUint(float x, float x_min, float x_max, int bits);
    float uintToFloat(uint16_t x_int, float x_min, float x_max, int bits);
};
