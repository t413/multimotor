#include <gtest/gtest.h>
#include <multimotor/debugprint.h>
#include <multimotor/serial/lx_servo.h>
#include "test_utils.h"


TEST(LxSerialDriver, constructor) {
    MockSerial comms;
    SerialBus bus(&comms);
    LXServo servo(16, &bus);
}

TEST(LxSerialDriver, WriteCommand) {
    MockSerial comms;
    SerialBus bus(&comms);
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

/*
TEST(LxSerialDriver, ReadPosition) {
    MockSerial comms;
    SerialBus bus(&comms);
    LXServo servo(16, &bus);

    // Test write packet for position read command
    uint8_t expected_write[] = {
        0x55, 0x55,           // Header
        0x10,                 // ID (16)
        0x03,                 // Length
        0x1C,                 // Command (SERVO_POS_READ)
        0xD3                  // Checksum
    };

    // Test response packet from servo
    uint8_t mock_response[] = {
        0x55, 0x55,           // Header
        0x10,                 // ID (16)
        0x05,                 // Length
        0x1C,                 // Command (SERVO_POS_READ)
        0xF4, 0x01,          // Position (500)
        0xDB                  // Checksum
    };

    comms.expectWrite(expected_write, sizeof(expected_write));
    comms.queueRead(mock_response, sizeof(mock_response));

    int32_t pos = servo.pos_read();
    EXPECT_EQ(pos, 12000);  // 500 ticks = 12000 centidegrees
    EXPECT_TRUE(servo.isCommandOk());
}

TEST(LxSerialDriver, InvalidResponse) {
    MockSerial comms;
    SerialBus bus(&comms);
    LXServo servo(16, &bus);

    // Test write packet for position read command
    uint8_t expected_write[] = {
        0x55, 0x55,           // Header
        0x10,                 // ID (16)
        0x03,                 // Length
        0x1C,                 // Command (SERVO_POS_READ)
        0xD3                  // Checksum
    };

    // Test invalid response packet (wrong header)
    uint8_t mock_response[] = {
        0x55, 0x54,           // Invalid header
        0x10,                 // ID (16)
        0x05,                 // Length
        0x1C,                 // Command (SERVO_POS_READ)
        0xF4, 0x01,          // Position (500)
        0xDB                  // Checksum
    };

    comms.expectWrite(expected_write, sizeof(expected_write));
    comms.queueRead(mock_response, sizeof(mock_response));

    servo.pos_read();
    EXPECT_FALSE(servo.isCommandOk());
}

TEST(LxSerialDriver, ChecksumValidation) {
    MockSerial comms;
    SerialBus bus(&comms);
    LXServo servo(16, &bus);

    // Test write packet for position read command
    uint8_t expected_write[] = {
        0x55, 0x55,           // Header
        0x10,                 // ID (16)
        0x03,                 // Length
        0x1C,                 // Command (SERVO_POS_READ)
        0xD3                  // Checksum
    };

    // Test response packet with invalid checksum
    uint8_t mock_response[] = {
        0x55, 0x55,           // Header
        0x10,                 // ID (16)
        0x05,                 // Length
        0x1C,                 // Command (SERVO_POS_READ)
        0xF4, 0x01,          // Position (500)
        0xFF                  // Invalid Checksum
    };

    comms.expectWrite(expected_write, sizeof(expected_write));
    comms.queueRead(mock_response, sizeof(mock_response));

    servo.pos_read();
    EXPECT_FALSE(servo.isCommandOk());
}
*/
