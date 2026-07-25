#pragma once
#include "../drive_manager.h"
#include "serial_interface.h"

class SerialDriveManager : public DriveManager {
public:
    SerialDriveManager() { }
    // SerialInterface* getInterface() const { return interface_; }
    void beginSinglePin(SerialInterface* port, int txRxPin);
    void beginDualPins(SerialInterface* port, int txPin, int rxPin);

    virtual void addDrive(MotorDrive* drive) override;
    virtual MotorDrive* getDrive(uint8_t id) override;
    virtual MotorDrive* const* getDrives() const override { return drives_; }
    virtual uint8_t getCount() const override;

    bool writeAndConsumeEcho(uint8_t const* data, uint8_t len, uint32_t timeout_us);
    bool waitForReply(int16_t id, uint32_t timeout_us, uint8_t const** outData, uint8_t* outLen);

    virtual bool handleIncoming(uint32_t id, uint8_t const* data, uint8_t len, uint32_t now) override;
    virtual bool readOnce(uint32_t now, uint32_t timeout_us) override;
    virtual uint8_t iterate(uint32_t now, uint32_t timeout_ms) override;

    void write(uint8_t const* data, uint8_t len);

protected:
    static constexpr uint8_t MAX_DRIVES = 16;
    MotorDrive* drives_[MAX_DRIVES] = {nullptr};
    uint8_t driveCount_ = 0;
    int uartSinglePin_ = -1;
    SerialInterface* interface_ = nullptr;

    static constexpr uint8_t INBUF_LEN = 32;
    uint8_t inBuffer_[INBUF_LEN] = {0}; // Buffer for reading incoming data
    uint8_t inBufferIndex_ = 0;
};
