#pragma once
#include "../motordrive.h"
#include "serial_interface.h"

// LX16A Command definitions
#define LX16A_SERVO_MOVE_TIME_WRITE 1
#define LX16A_SERVO_MOVE_TIME_READ 2
#define LX16A_SERVO_MOVE_TIME_WAIT_WRITE 7
#define LX16A_SERVO_MOVE_START 11
#define LX16A_SERVO_MOVE_STOP 12
#define LX16A_SERVO_ID_WRITE 13
#define LX16A_SERVO_ID_READ 14
#define LX16A_SERVO_POS_READ 28
#define LX16A_SERVO_OR_MOTOR_MODE_WRITE 29
#define LX16A_SERVO_LOAD_OR_UNLOAD_WRITE 31
#define LX16A_SERVO_TEMP_READ 26
#define LX16A_SERVO_VIN_READ 27
#define LX16A_SERVO_ANGLE_LIMIT_WRITE 20
#define LX16A_SERVO_ANGLE_LIMIT_READ 21
#define LX16A_BROADCAST_ID 0xFE

class LXBus {
    SerialInterface* port_;
    uint32_t baud_;
    bool debug_;
    int retryCount_;

public:
    LXBus(SerialInterface* port, uint32_t baud = 115200);
    void setDebug(bool debug) { debug_ = debug; }
    void setRetryCount(int count) { retryCount_ = count; }

    bool write(uint8_t cmd, const uint8_t* params, int param_cnt, uint8_t id);
    bool read(uint8_t cmd, uint8_t* params, int param_len, uint8_t id);

private:
    bool writeNoRetry(uint8_t cmd, const uint8_t* params, int param_cnt, uint8_t id);
    bool readNoRetry(uint8_t cmd, uint8_t* params, int param_len, uint8_t id);
    bool receive(uint8_t cmd, uint8_t* params, int param_len, uint8_t id);
    uint32_t timeUs(uint32_t bytes) { return bytes * 10 * 1000000 / baud_; }
};

class LXServo : public MotorDrive {
    uint8_t id_;
    LXBus* bus_;
    MotorState lastStatus_;
    uint32_t lastStatusTime_;
    bool enabled_;
    float minAngleDeg_, maxAngleDeg_;
    float temperature_;
    float voltage_;

public:
    LXServo(uint8_t id, LXBus* bus);

    // MotorDrive interface
    void requestStatus() override;
    void setMode(MotorMode mode) override;
    void setSetpoint(MotorMode mode, float value) override;
    bool handleIncoming(uint32_t id, uint8_t* data, uint8_t len, uint32_t now) override { return false; } // Not used for serial
    uint32_t getLastStatusTime() const override { return lastStatusTime_; }
    uint32_t getLastFaults() const override { return 0; } // TODO: implement fault detection
    MotorState getMotorState() const override { return lastStatus_; }
    void fetchVBus() override;
    float getVBus() const override { return voltage_; }

    // LX16A specific methods
    void setAngleLimits(float minDeg, float maxDeg);
    void setId(uint8_t newId);
    uint8_t getId();
    void stop();
    float getTemperature() const { return temperature_; }

private:
    float ticksToAngle(uint16_t ticks) { return ticks * 0.24f; }
    uint16_t angleToTicks(float angle) { return (uint16_t)(angle / 0.24f); }
    bool isCommandOk_;
};
