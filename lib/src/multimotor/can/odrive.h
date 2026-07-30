#pragma once
#include "../motordrive.h"
#include "../can/can_interface.h"

class CanInterface;
enum class OdriveAxisState : uint8_t {
    Undefined = 0,
    Idle = 1,
    StartupSequence = 2,
    FullCalibrationSequence = 3,
    MotorCalibration = 4,
    EncoderIndexSearch = 5,
    EncoderOffsetCalibration = 6,
    ClosedLoopControl = 7,
    LockinSpin = 8,
    EncoderDirFind = 9,
    Homing = 10,
    EncoderHallPolarityCalibration = 11,
    EncoderHallPhaseCalibration = 12,
};
enum class OdriveCtrlMode : int8_t {
    Unknown   = -1,
    Voltage   = 0,
    Torque    = 1,
    Velocity  = 2,
    Position  = 3,
};
enum class CmdIDs : uint8_t;

class ODriveDriver : public MotorDrive {
    uint8_t id_ = 0;
    CanInterface* can_ = nullptr;
    uint32_t lastFaults_ = 0;
    uint8_t lastAxisState_ = 0;
    uint32_t lastHeartbeatTime_ = 0;
    uint32_t lastStatusTime_ = 0;
    float lastVolt_ = 0, lastCurr_ = 0; //encoder estimates packet
    uint32_t lastBusVoltTime_ = 0;
    MotorState lastStatus_; // Holds the state of the motor
    MotorMode lastSentMode_ = MotorMode::Unknown;
public:
    ODriveDriver(uint8_t id, CanInterface* can, const char* name);
    static constexpr uint8_t DEFAULT_ID = 0x3f;

    //contract
    uint32_t getId() const override { return id_; }
    bool requestStatus() override;
    bool setMode(MotorMode) override;
    bool setSetpoint(MotorMode, float) override;
    bool handleIncoming(uint32_t id, uint8_t const* data, uint8_t len, uint32_t now) override;
    uint32_t getLastStatusTime() const override { return lastStatusTime_; }
    uint32_t getLastFaults() const override { return lastFaults_; }

    MotorState getMotorState() const override { return lastStatus_; }
    bool setOdriveMode(OdriveCtrlMode);
    bool setOdriveEnable(bool enable);
    bool clearErrors();
    bool send(CmdIDs cmd, uint8_t* data, uint8_t len = 8, CanSS ss = CanSS::Singleshot, CanReq rtr = CanReq::Command);

    bool fetchVBus() override;
    float getVBus() const override { return lastVolt_; }
    bool ping(int timeout_ms = 100) override;
    bool validID(int id) const override { return id >= 0 && id < DEFAULT_ID; }
    MotorDrive* makeDuplicate(int16_t id = -1) const override;
    bool writeNewId(uint8_t newId, bool sendToDrive = true) override;
};

