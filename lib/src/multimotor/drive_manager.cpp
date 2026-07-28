#include "drive_manager.h"
#include "motordrive.h"

MotorDrive* DriveManager::findAtAddr(uint8_t addr) {
    auto count = getCount();
    auto drives = getDrives();
    if (count == 0) return nullptr;
    uint8_t savedId = drives[0]->getId();
    drives[0]->writeNewId(addr, false); //temporarily set addr of drives[0]
    MotorDrive* ret = nullptr;
    if (drives[0]->ping()) {
        ret = drives[0]->makeDuplicate(addr);
    }
    drives[0]->writeNewId(savedId, false); //restore original ID
    return ret;
}
