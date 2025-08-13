#include <gtest/gtest.h>
#include <multimotor/can/cybergear.h>
#include "test_utils.h"


TEST(CyberGear, constructor) {
    MockCanInterface mockCan;
    CyberGearDriver driver(1, &mockCan);
}

TEST(CyberGear, RequestStatus) {
    MockCanInterface mockCan;
    CyberGearDriver driver(16, &mockCan);
    printf("Requesting status...\n");
    fflush(stdout);
    driver.requestStatus();
    ASSERT_FALSE(mockCan.sentFrames.empty());
    auto& frame = mockCan.sentFrames.back();
    EXPECT_EQ(frame.id, 0x15000010);
    EXPECT_EQ(frame.len, 8);
}

TEST(CyberGear, SetEnable) {
    MockCanInterface mockCan;
    CyberGearDriver driver(16, &mockCan);
    driver.setEnable(true);
    ASSERT_FALSE(mockCan.sentFrames.empty());
    auto& frame = mockCan.sentFrames.back();
    EXPECT_EQ(frame.id, 0x03000010);
    for (int i = 0; i < 8; ++i) {
        EXPECT_EQ(frame.data[i], 0x00);
    }
}

TEST(CyberGear, HandleIncoming) {
    MockCanInterface mockCan;
    CyberGearDriver driver(16, &mockCan);


    // Prepare a realistic status frame
    // Compose CAN ID:
    uint32_t cmd = 0x2 << 24; // bits 24-31: CmdRequest (0x2)
    uint32_t mode = 0x1 << 22; // bits 22-23: Mode (0x1 = Position)
    uint32_t faults = 0x05 << 16; // bits 16-21: Faults (0x05)
    uint32_t driveid = 16 << 8; // bits 8-15: DriveID (16)
    uint32_t can_id = cmd | mode | faults | driveid;

    // pos,vel,torq,temp: 1234,5678,9101,250
    uint8_t data[8] = {0x04, 0xD2, 0x16, 0x2E, 0x23, 0x8D, 0x00, 0xFA};

    printf("Handling incoming frame with ID: %08X: ", can_id);
    for (int i = 0; i < 8; ++i) {
        printf("%02X ", data[i]);
    }
    printf("\n");

    uint32_t now = 123456;
    bool handled = driver.handleIncoming(can_id, data, 8, now);
    EXPECT_TRUE(handled);
    EXPECT_EQ(driver.getLastFaults(), 0x05);
    EXPECT_EQ(driver.getLastStatusTime(), now);
    auto status = driver.getMotorState();
    printf("Motor State: Position: %.2f, Velocity: %.2f, Torque: %.2f, Temperature: %.2f, Mode: %d\n",
           status.position, status.velocity, status.torque, status.temperature, static_cast<int>(status.mode));
    EXPECT_NEAR(status.position, -12.03f, 0.01f);
    EXPECT_NEAR(status.velocity, -24.80f, 0.01f);
    EXPECT_NEAR(status.torque, -8.67f, 0.01f);
    EXPECT_NEAR(status.temperature, 25.0f, 0.01f);
    EXPECT_EQ(status.mode, MotorMode::Position);
}
