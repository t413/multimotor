#include "motordrive.h"
#include "debugprint.h"

bool MotorDrive::pingId(uint8_t id, uint32_t timeout) {
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
