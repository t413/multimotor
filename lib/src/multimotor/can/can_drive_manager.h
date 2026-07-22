#pragma once
#include "../drive_manager.h"
class CanInterface;

class CanDriveManager : public DriveManager {
public:
    CanDriveManager(CanInterface* canInterface) : interface_(canInterface) {}

    virtual void addDrive(MotorDrive* drive) override;
    virtual MotorDrive* getDrive(uint8_t id) override;
    virtual MotorDrive* const* getDrives() const override { return drives_; }
    virtual uint8_t getCount() const override;

    virtual bool handleIncoming(uint32_t id, uint8_t const* data, uint8_t len, uint32_t now) override;
    virtual bool readOnce(uint32_t now, uint32_t timeout_us) override;
    virtual uint8_t iterate(uint32_t now, uint32_t timeout_ms) override;

protected:
    static constexpr uint8_t MAX_DRIVES = 16;
    MotorDrive* drives_[MAX_DRIVES] = {nullptr};
    uint8_t driveCount_ = 0;
    CanInterface* interface_ = nullptr;
};

#ifndef ARDUINO
uint32_t micros();
void delayMicroseconds(uint32_t);
#endif
