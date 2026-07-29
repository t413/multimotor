#pragma once
#include <stdint.h>

enum class MotorMode {
    Disabled = 0,
    Current = 1,
    Speed = 2,
    Position = 3,
    Unknown = 4,
};

struct MotorState {
    float temperature = 0.0f; // in degrees Celsius
    float position = 0.0f; // in radians
    float velocity = 0.0f; // in radians per second
    float torque = 0.0f; // in Newton-meters
    MotorMode mode = MotorMode::Disabled;
};
class MotorDrive {
public:
    MotorDrive(const char* name) : name_(name) { }
    virtual uint32_t getId() const = 0;
    virtual const char* getName() const { return name_; }
    virtual bool requestStatus() = 0;
    virtual bool setMode(MotorMode mode) = 0;
    virtual bool supportsMode(MotorMode) const { return true; }
    virtual bool setSetpoint(MotorMode, float) = 0;
    virtual bool handleIncoming(uint32_t id, uint8_t const* data, uint8_t len, uint32_t now) = 0;
    virtual uint32_t getLastStatusTime() const = 0;
    virtual uint32_t getLastFaults() const = 0;

    virtual MotorState getMotorState() const = 0;
    virtual bool fetchVBus() = 0;
    virtual float getVBus() const = 0;

    virtual MotorDrive* makeDuplicate(uint8_t id) const = 0;
    virtual bool writeNewId(uint8_t newId, bool sendToDrive = true) = 0;  // sends ID-write command targeting current id_
    virtual bool ping(int timeout_ms = 100) = 0; // sends a read cmd, returns true if it replies

protected:
    const char* name_ = nullptr;
};
