#pragma once
#include "../motordrive.h"
#include "../can/can_interface.h"

enum class CGCmds : uint8_t;

class CyberGearDriver : public MotorDrive {
    uint8_t id_ = 0;
    CanInterface* can_ = nullptr;
    uint8_t lastFaults_ = 0;
    uint32_t lastStatusTime_ = 0;
    uint32_t lastCommsTime_ = 0;
    bool enabled_ = false;
    MotorState lastStatus_; // Holds the state of the motor
    float vbus_ = 0.0f;  // Cached VBUS value

public:
    CyberGearDriver(uint8_t id, CanInterface* can, const char* name);
    static constexpr uint8_t DEFAULT_ID = 0x7D;

    //contract
    uint32_t getId() const override { return id_; }
    bool requestStatus() override;
    bool setMode(MotorMode) override;
    bool setSetpoint(MotorMode, float) override;
    bool handleIncoming(uint32_t id, uint8_t const* data, uint8_t len, uint32_t now) override;
    uint32_t getLastStatusTime() const override { return lastStatusTime_; }
    uint32_t getLastFaults() const override { return lastFaults_; }

    bool send(CGCmds cmd, uint8_t* data, uint8_t len, CanSS ss = CanSS::Singleshot);
    MotorState getMotorState() const override { return lastStatus_; }
    bool setCyberMode(uint8_t mode);
    bool setEnable(bool enable);

    bool fetchVBus() override;
    float getVBus() const override { return vbus_; }
    bool ping(int timeout_ms = 100) override;
    MotorDrive* makeDuplicate(int16_t id = -1) const override;
    bool writeNewId(uint8_t newId, bool sendToDrive = true) override;
};

