#include "serial_drive_manager.h"
#include "../motordrive.h"
#include "../debugprint.h"
#include "lx_servo.h"

void SerialDriveManager::addDrive(MotorDrive* drive) {
    if (driveCount_ < MAX_DRIVES) {
        drives_[driveCount_++] = drive;
    }
}

MotorDrive* SerialDriveManager::getDrive(uint8_t id) {
    for (uint8_t i = 0; i < driveCount_; ++i) {
        if (drives_[i] && drives_[i]->getId() == id) {
            return drives_[i];
        }
    }
    return nullptr;
}

uint8_t SerialDriveManager::getCount() const {
    return driveCount_;
}

void SerialDriveManager::handleIncoming(uint32_t id, uint8_t const* indata, uint8_t inlen, uint32_t now) {
    auto dbg = DebugPrinter::getPrinter();

    uint8_t loop = 0; //limited looping
    while ((inlen > 0) && loop++ < 4) {
        //copy as much data as we can into inBuffer_
        int16_t copyLen = std::min((int16_t)inlen, (int16_t)(INBUF_LEN - inBufferIndex_));
        memcpy(inBuffer_ + inBufferIndex_, indata, copyLen);
        inBufferIndex_ += copyLen;
        indata += copyLen;
        inlen -= copyLen;
        //try and parse an LX packet
        ParseResult result = LXServo::parsePacket(inBuffer_, inBufferIndex_);
        if (result.id >= 0 && result.len > 0) {
            if (auto servo = getDrive(result.id)) {
                bool handled = servo->handleIncoming(result.id, inBuffer_ + result.start, result.len, now);
                if (!handled) DebugPrinter::log("LXServo: Unhandled packet for ID %d\n", result.id);
            } else DebugPrinter::log("LXServo: No servo registered with ID %d\n", result.id);
            //remove parsed data from inBuffer_
            memmove(inBuffer_, inBuffer_ + result.start + result.len, inBufferIndex_ - (result.start + result.len));
            inBufferIndex_ -= (result.start + result.len);
        }
    }
}

void SerialDriveManager::iterate(uint32_t now) {
    uint8_t buf[INBUF_LEN];
    int len = 0;
    while (interface_->available() && (len < INBUF_LEN)) {
        buf[len++] = interface_->read();
    }
    if (len > 0) {
        handleIncoming(0, buf, len, now);
    }
}
