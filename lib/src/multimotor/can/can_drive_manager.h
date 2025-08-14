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

    virtual void handleIncoming(uint32_t id, uint8_t const* data, uint8_t len, uint32_t now) override;
    virtual void iterate(uint32_t now) override;

protected:
    static constexpr uint8_t MAX_DRIVES = 16;
    MotorDrive* drives_[MAX_DRIVES] = {nullptr};
    uint8_t driveCount_ = 0;
    CanInterface* interface_ = nullptr;
};
