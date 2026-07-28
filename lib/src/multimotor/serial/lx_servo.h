#pragma once
#include "../motordrive.h"

class SerialDriveManager;

struct ParseResult {
    uint8_t start;
    uint8_t len;
    int16_t id;
};

constexpr uint8_t LX16A_BROADCAST_ID = 0xFE;

class LXServo : public MotorDrive {
public:
    LXServo(uint8_t id, SerialDriveManager* bus, const char* name);

    // MotorDrive interface
    bool requestStatus() override;
    bool setMode(MotorMode mode) override;
    bool setSetpoint(MotorMode mode, float value) override;
    bool handleIncoming(uint32_t id, uint8_t const* data, uint8_t len, uint32_t now) override; // Handles payload for this servo
    uint32_t getLastStatusTime() const override { return lastStatusTime_; }
    uint32_t getLastFaults() const override { return lastFaults_; }
    MotorState getMotorState() const override { return lastStatus_; }
    bool fetchVBus() override;
    float getVBus() const override { return voltage_; }

    // LX16A specific methods
    bool requestPosition();
    bool requestTemp();
    bool movePosTime(int16_t ticks, int16_t time);
    bool moveSpeed(int16_t speed);
    bool setAngleLimits(float minDeg, float maxDeg);
    uint32_t getId() const override { return id_; }
    bool stop();
    static ParseResult parsePacket(uint8_t const* data, uint8_t len);

    MotorDrive* makeDuplicate(uint8_t id) const override;
    bool writeNewId(uint8_t newId, bool sendToDrive = true) override;
    bool ping(int timeout_ms = 100) override;

protected:
    float ticksToAngle(int32_t ticks) { return ticks * 0.24f; }
    uint16_t angleToTicks(float angle) { return (uint16_t)(angle / 0.24f); }

    bool enable(bool en = true);
    int buildPacket(uint8_t* txbuf, uint8_t cmd, const uint8_t* params, int param_cnt, uint8_t id);
    bool sendCommand(uint8_t cmd, const uint8_t* params, int param_cnt, bool expectResponse = false, uint32_t timeout_us = 5000);

protected:
    uint8_t id_ = 0;
    SerialDriveManager* bus_ = nullptr;
    MotorState lastStatus_;
    uint32_t lastStatusTime_ = 0;
    uint32_t lastFaults_ = 0;
    bool enabled_ = false;
    float minAngleDeg_ = 0.0f, maxAngleDeg_ = 240.0f;
    float voltage_ = 0.0f;
    uint32_t lastTempRead_ = 0;
};
