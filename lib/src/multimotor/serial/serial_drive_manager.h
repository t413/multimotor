#pragma once
#include "../drive_manager.h"
#include "serial_interface.h"

class SerialDriveManager : public DriveManager {
public:
    SerialDriveManager(SerialInterface* serialInterface) : interface_(serialInterface) {}
    SerialInterface* getInterface() const { return interface_; }

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
    SerialInterface* interface_ = nullptr;

    static constexpr uint8_t INBUF_LEN = 32;
    uint8_t inBuffer_[INBUF_LEN] = {0}; // Buffer for reading incoming data
    uint8_t inBufferIndex_ = 0;
};
