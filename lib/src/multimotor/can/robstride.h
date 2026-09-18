#pragma once
#include <math.h>
#include "../motordrive.h"
#include "../can/can_interface.h"

class CanDriveManager;

enum class RSCmd : uint8_t;

class RobStrideDriver : public MotorDrive {
protected:
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
    float scalePMax_ = 4 * M_PI;
    float scaleVMax_ = 50.0f; //rad/s, value for EL05
    float scaleTMax_ = 6.0f; //EL05

public:
    RobStrideDriver(uint8_t id, CanDriveManager* bus, const char* name);
    void setScales(float pmax, float vmax, float tmax);
    static constexpr uint8_t MAX_ID = 0x7F;
    static constexpr uint8_t DEFAULT_ID = MAX_ID;
    static constexpr uint8_t DEFAULT_HOST_ID = 0x66;

    // MotorDrive interface implementation
    static constexpr const char* SHORTNAME = "RS";
    const char* typeName() const override { return SHORTNAME; }
    uint32_t getId() const override { return id_; }
    bool requestStatus() override;
    bool setMode(MotorMode mode) override;
    bool setSetpoint(MotorMode mode, float value) override;
    bool mitTarget(float position, float velocity, float kp, float kd, float torqueFF) override;
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
    bool setZero() override;
    bool saveSettings() override;

    bool enable(bool en = true);
    bool setZeroPosition();
    bool reqParam(uint16_t paramId);

private:
    bool send(RSCmd cmd, const uint8_t* data, uint8_t len, uint16_t extradata = DEFAULT_HOST_ID, CanSS ss = CanSS::Singleshot, CanReq rtr = CanReq::Command);
};

class CGDrive : public RobStrideDriver {
public:
    CGDrive(uint8_t id, CanDriveManager* bus, const char* name) : RobStrideDriver(id,bus,name) {
        setScales(scalePMax_, 30.0f, 12.0f);
    }
    static constexpr const char* SHORTNAME = "CG";
    const char* typeName() const override { return SHORTNAME; }
};
