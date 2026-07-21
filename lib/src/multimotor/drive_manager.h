#pragma once
#include <stdint.h>
class MotorDrive;

class DriveManager {
public:
    virtual void addDrive(MotorDrive* drive) = 0;
    virtual MotorDrive* getDrive(uint8_t id) = 0;
    virtual MotorDrive* const* getDrives() const = 0;
    virtual uint8_t getCount() const = 0;

    virtual bool handleIncoming(uint32_t id, uint8_t const* data, uint8_t len, uint32_t now) = 0; //parses incoming data, updates drives. returns true if handled
    virtual bool readOnce(uint32_t now, uint32_t timeout_us) = 0;
    virtual uint8_t iterate(uint32_t now, uint32_t timeout_ms) = 0; //reads port_ data if available, calls handleIncoming, returns new packet count
};
