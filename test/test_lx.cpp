#include <gtest/gtest.h>
#include <multimotor/debugprint.h>
#include <multimotor/serial/serial_drive_manager.h>
#include <multimotor/serial/lx_servo.h>
#include "test_utils.h"


TEST(LxSerialDriver, constructor) {
    MockSerial comms;
    SerialDriveManager bus(&comms);
    LXServo servo(16, &bus);
}

TEST(LxSerialDriver, WriteCommand) {
    MockSerial comms;
    SerialDriveManager bus(&comms);
    LXServo servo(16, &bus);

    // Test packet for SERVO_MOVE_TIME_WRITE command
    // ID: 16, Position: 500, Time: 1000
    uint8_t expected[] = {
        0x55, 0x55,           // Header
        0x10,                 // ID (16)
        0x07,                 // Length
        0x01,                 // Command (SERVO_MOVE_TIME_WRITE)
        0xF4, 0x01,          // Position (500)
        0xE8, 0x03,          // Time (1000)
        0x07                  // Checksum
    };

    servo.movePosTime(500, 1000);  // 500 ticks = 12000 centidegrees
    ASSERT_EQ(comms.sentFrames.size(), 1);
    auto tx = comms.sentFrames.back();
    ASSERT_EQ(tx.size(), sizeof(expected));
    for (size_t i = 0; i < sizeof(expected); i++) {
        EXPECT_EQ((uint8_t)tx.at(i), expected[i]) << "Byte " << i << " mismatch";
    }
}
