#pragma once
#include "../motordrive.h"
#include "serial_interface.h"

class LXServo;


class LXBus {
    SerialInterface* port_;
    bool debug_;
    uint8_t inBuffer_[32] = {0}; // Buffer for reading incoming data
    uint8_t inBufferIndex_ = 0;
    LXServo* servos_[16] = {nullptr};

public:
    LXBus(SerialInterface* port);
    void setDebug(bool debug) { debug_ = debug; }

    bool send(uint8_t cmd, const uint8_t* params, int param_cnt, uint8_t id);
    void iterate(); // Process incoming data and update servo states
};

class LXServo : public MotorDrive {
    uint8_t id_;
    LXBus* bus_;
    MotorState lastStatus_;
    uint32_t lastStatusTime_;
    uint32_t lastFaults_;
    bool enabled_;
    float minAngleDeg_, maxAngleDeg_;
    float temperature_;
    float voltage_;

    // Async state management
    uint8_t expectedCmd_;
    uint8_t expectedId_;
    uint32_t lastRequestTime_;
    bool waitingForResponse_;

    // Packet parsing state
    uint8_t parseState_;
    uint8_t packetBuffer_[16];
    uint8_t packetIndex_;
    uint8_t packetLength_;
    uint8_t checksum_;

public:
    LXServo(uint8_t id, LXBus* bus);

    // MotorDrive interface
    void requestStatus() override;
    void setMode(MotorMode mode) override;
    void setSetpoint(MotorMode mode, float value) override;
    bool handleIncoming(uint32_t id, uint8_t* data, uint8_t len, uint32_t now) override;
    uint32_t getLastStatusTime() const override { return lastStatusTime_; }
    uint32_t getLastFaults() const override { return lastFaults_; }
    MotorState getMotorState() const override { return lastStatus_; }
    void fetchVBus() override;
    float getVBus() const override { return voltage_; }

    // LX16A specific methods
    void setAngleLimits(float minDeg, float maxDeg);
    void setId(uint8_t newId);
    void stop();
    float getTemperature() const { return temperature_; }

private:
    float ticksToAngle(uint16_t ticks) { return ticks * 0.24f; }
    uint16_t angleToTicks(float angle) { return (uint16_t)(angle / 0.24f); }

    void sendCommand(uint8_t cmd, const uint8_t* params, int param_cnt, bool expectResponse = false);
    bool processPacket();
    void resetParser();
};
