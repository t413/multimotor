#include "drive_manager.h"
#include "motordrive.h"

bool DriveManager::remove(MotorDrive* drive) {
    MotorDrive** drives = getDrives();
    auto count = getCount();
    for (int i = 0; i < count; i++) {
        if (drives[i] == drive) {
            // Backfill the array
            for (int j = i; j < count - 1; j++) {
                drives[j] = drives[j + 1];
            }
            setCount(count - 1);
            return true;
        }
    }
    return false;
}

MotorDrive* DriveManager::getDriveByID(uint8_t id) {
    auto count = getCount();
    for (uint8_t i = 0; i < count; ++i) {
        auto drive = getDrive(i);
        if (drive && drive->getId() == id) {
            return drive;
        }
    }
    return nullptr;
}
