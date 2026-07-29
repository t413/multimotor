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
    virtual ~MotorDrive() { }
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

    virtual bool writeNewId(uint8_t newId, bool sendToDrive = true) = 0;  // sends ID-write command targeting current id_
    virtual MotorDrive* makeDuplicate(int16_t id = -1) const = 0; //make new instance, -1 is default address
    virtual bool ping(int timeout_ms = 100) = 0; // sends a read cmd, returns true if it replies
    virtual bool pingId(uint8_t id, uint32_t timeout = 1000);
    virtual bool validID(int id) const = 0;
    virtual int16_t discoverNext(bool updateThisID = true, uint32_t pingTimeout = 100, uint32_t totalTimeout = 1000); //search and find another instance on the bus, optionally updating id_

protected:
    const char* name_ = nullptr;
};
