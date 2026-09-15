#include "drive_manager.h"

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
