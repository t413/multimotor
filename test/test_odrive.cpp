#include <gtest/gtest.h>
#include <multimotor/debugprint.h>
#include <multimotor/can/odrive.h>
#include "test_utils.h"


TEST(ODriveDriver, constructor) {
    MockCanInterface mockCan;
    ODriveDriver driver(1, &mockCan);
}

TEST(ODriveDriver, RequestStatus) {
    MockCanInterface mockCan;
    ODriveDriver driver(16, &mockCan);
    printf("Requesting status...\n");
    fflush(stdout);
    driver.requestStatus();
    ASSERT_FALSE(mockCan.sentFrames.empty());
    auto& frame = mockCan.sentFrames.back();
    EXPECT_EQ(frame.id, 0x209);
    EXPECT_EQ(frame.len, 0);
}

TEST(ODriveDriver, SetEnable) {
    MockCanInterface mockCan;
    ODriveDriver driver(16, &mockCan);
    driver.setMode(MotorMode::Position);
    ASSERT_FALSE(mockCan.sentFrames.empty());
    auto& frame = mockCan.sentFrames.back();
    EXPECT_EQ(frame.id, 0x207);
    EXPECT_EQ(frame.data[0], 0x08);
    for (int i = 1; i < 8; ++i)
        EXPECT_EQ(frame.data[i], 0x00);
}

TEST(ODriveDriver, HandleIncoming_heartbeat) {
    MockCanInterface mockCan;
    ODriveDriver driver(16, &mockCan);

    // Prepare a realistic heartbeat frame
    uint32_t can_id = (0x01) + (16 << 5);  // (heartbeat type) + (node_id 16)
    uint8_t data[8] = { 0x05, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00 }; // fault 5, closed-loop mode

    uint32_t now = 123456;
    EXPECT_TRUE(driver.handleIncoming(can_id, data, 8, now));
    EXPECT_EQ(driver.getLastFaults(), 5);
    EXPECT_EQ(driver.getMotorState().mode, MotorMode::Unknown); // unknown until we've sent a mode
}
