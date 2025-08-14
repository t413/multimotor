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

void CanDriveManager::handleIncoming(uint32_t id, uint8_t const* data, uint8_t len, uint32_t now) {
    uint8_t handled = 0;
    for (uint8_t i = 0; i < driveCount_; ++i) {
        handled += drives_[i]->handleIncoming(id, data, len, now);
    }
    auto dbg = DebugPrinter::getPrinter();
    if (!handled && dbg) {
        dbg->printf("CanDriveManager: No drive handled incoming data for ID %u, len %u:", id, len);
        dbg->printhex(data, len, true);
    }
}

void CanDriveManager::iterate(uint32_t now) {
    #ifdef ARDUINO_ARCH_ESP32
    auto can = (CanEsp32Twai*) interface_;
    while (can && can->available()) {
        CanMessage message = can->readOne();
        handleIncoming(message.id, message.data, message.len, now);
    }
    #endif
}
