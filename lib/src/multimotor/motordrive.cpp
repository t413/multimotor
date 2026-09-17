#include "motordrive.h"
#include "debugprint.h"
#include "drive_manager.h"


MotorDrive::MotorDrive(const char* name, DriveManager* mgr) : name_(name), mgr_(mgr) {
    if (mgr) mgr->addDrive(this);
}

MotorDrive::~MotorDrive() {
    if (mgr_) mgr_->remove(this);
}

uint16_t MotorDrive::floatToUint(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    if (x > x_max) x = x_max;
    else if (x < x_min) x = x_min;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

float MotorDrive::uintToFloat(uint16_t x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

bool MotorDrive::pingId(uint8_t id, uint32_t timeout) {
    if (!validID(id)) return false;
    uint8_t originalId = getId();
    writeNewId(id, false);
    bool ret = ping(timeout);
    writeNewId(originalId, false); //restore original
    return ret;
}

int16_t MotorDrive::discoverNext(bool updateThisID, uint32_t pingTimeout, uint32_t totalTimeout) {
    uint32_t start = millis();
    uint8_t candidate = getId();
    for (uint16_t i = 0; i < 253; ++i) {
        candidate++;
        if (candidate == 0 || candidate >= 0xFE)
            candidate = 1;
        if (pingId(candidate, pingTimeout)) {
            if (updateThisID) writeNewId(candidate, false);
            return candidate;
        }
        if ((millis() - start) > totalTimeout) break;
    }
    return -1;
}

bool MotorDrive::waitForReply(uint32_t now_ms, uint32_t timeout_us) {
    auto bus = getManager();
    return bus? bus->readOnce(now_ms, timeout_us) : false;
}
