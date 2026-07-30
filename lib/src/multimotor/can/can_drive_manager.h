#pragma once
#include "../drive_manager.h"
#include "can_interface.h"

class CanDriveManager : public DriveManager {
public:
    CanDriveManager(CanInterface* canInterface) : interface_(canInterface) {}

    virtual void addDrive(MotorDrive* drive) override;
    virtual MotorDrive* getDrive(uint8_t id) override;
    virtual MotorDrive* const* getDrives() const override { return drives_; }
    virtual uint8_t getCount() const override;
    MotorDrive* first() const override { return driveCount_ > 0 ? drives_[0] : nullptr; }

    virtual bool handleIncoming(uint32_t id, uint8_t const* data, uint8_t len, uint32_t now) override;
    virtual bool readOnce(uint32_t now, uint32_t timeout_us) override;
    virtual uint8_t iterate(uint32_t now, uint32_t timeout_ms) override;

    virtual bool send(uint32_t id, uint8_t* data, uint8_t len, CanFrame extended, CanSS ss = CanSS::Singleshot, CanReq rtr = CanReq::Command);
    bool waitForReply(CanMessage&, uint32_t timeout_us, uint32_t id = 0, uint32_t idmask = 0xffffffff);

protected:
    static constexpr uint8_t MAX_DRIVES = 16;
    MotorDrive* drives_[MAX_DRIVES] = {nullptr};
    uint8_t driveCount_ = 0;
    CanInterface* interface_ = nullptr;
};

