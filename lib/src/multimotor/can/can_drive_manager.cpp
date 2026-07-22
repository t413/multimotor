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
    if (timeout_us == 0) {
        return iterate(now, 0) > 0;
    }
    uint32_t start = micros();
    uint32_t deadline = start + timeout_us;
    uint32_t timeout_ms = timeout_us / 1000;

    while ((int32_t)(micros() - deadline) <= 0) {
        if (iterate(now, timeout_ms) > 0) return true;
        delayMicroseconds(10); //small pause to avoid busy loop
    }
    return false;
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

#ifndef ARDUINO
#include <sys/time.h>
#include <unistd.h>
uint32_t micros() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint32_t)(tv.tv_sec * 1000000 + tv.tv_usec);
}
void delayMicroseconds(uint32_t microseconds) {
    usleep(microseconds);
}
#endif
