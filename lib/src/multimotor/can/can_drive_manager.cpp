#include "can_drive_manager.h"
#include "can_interface.h"
#include "../motordrive.h"
#include "../debugprint.h"
#ifdef ARDUINO_ARCH_ESP32
#include "can_esp32_twai.h"
#endif

void CanDriveManager::addDrive(MotorDrive* drive) {
    if (driveCount_ < MAX_DRIVES) {
        drives_[driveCount_++] = drive;
    }
}

MotorDrive* CanDriveManager::getDrive(uint8_t id) {
    for (uint8_t i = 0; i < driveCount_; ++i) {
        if (drives_[i] && drives_[i]->getId() == id) {
            return drives_[i];
        }
    }
    return nullptr;
}

uint8_t CanDriveManager::getCount() const {
    return driveCount_;
}

bool CanDriveManager::send(uint32_t id, uint8_t* data, uint8_t len, CanFrame extended, CanSS ss, CanReq rtr) {
    return interface_ ? interface_->send(id, data, len, extended, ss, rtr) : false;
}

bool CanDriveManager::waitForReply(CanMessage& msg, uint32_t timeout_us, uint32_t id, uint32_t idmask) {
    uint32_t deadline = micros() + timeout_us;
    while ((int32_t)(deadline - micros()) > 0) {
        if (interface_->readOne(msg, (int32_t)(deadline - micros()) / 1000)) {
            handleIncoming(msg.id, msg.data, msg.len, millis());
            if ((msg.id & idmask) == (id & idmask))
                return true;
        }
        delayMicroseconds(10); //small pause to avoid busy loop
    }
    return false;
}

bool CanDriveManager::handleIncoming(uint32_t id, uint8_t const* data, uint8_t len, uint32_t now) {
    uint8_t handled = 0;
    for (uint8_t i = 0; i < driveCount_; ++i) {
        handled += drives_[i]->handleIncoming(id, data, len, now);
    }
    auto dbg = DebugPrinter::getPrinter();
    if (!handled && dbg) {
        dbg->printf("CanDriveManager: No drive handled incoming data for ID %u, len %u:", id, len);
        dbg->printhex(data, len, true);
    }
    return handled > 0;
}

bool CanDriveManager::readOnce(uint32_t now, uint32_t timeout_us) {
    CanMessage msg;
    return waitForReply(msg, timeout_us, now);
}

uint8_t CanDriveManager::iterate(uint32_t now, uint32_t timeout_ms) {
    uint8_t ret = 0;
    CanMessage msg;
    while (interface_ && interface_->readOne(msg, ret == 0 ? timeout_ms : 0)) {
        handleIncoming(msg.id, msg.data, msg.len, now);
        ++ret;
    }
    return ret;
}
