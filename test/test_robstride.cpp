#include <gtest/gtest.h>
#include <multimotor/debugprint.h>
#include <multimotor/can/can_drive_manager.h>
#include <multimotor/can/robstride.h>
#include "test_utils.h"


TEST(RobStrideDriver, RequestStatus) {
    MockCanInterface mockCan;
    CanDriveManager mgr(&mockCan);
    RobStrideDriver driver(RobStrideDriver::DEFAULT_ID, &mgr, "test");
    printf("Requesting status...\n");
    fflush(stdout);
    EXPECT_EQ(driver.getId(), 0x7F);
    driver.requestStatus();
    ASSERT_FALSE(mockCan.sentFrames.empty());
    auto& frame = mockCan.sentFrames.back();
    EXPECT_EQ(frame.id, 0x0000667F);
    EXPECT_EQ(frame.len, 8);
}

TEST(RobStrideDriver, SetMotion) {
    MockCanInterface mockCan;
    CanDriveManager mgr(&mockCan);
    RobStrideDriver driver(RobStrideDriver::DEFAULT_ID, &mgr, "test");
    EXPECT_EQ(driver.getId(), 0x7F);
    driver.setSetpoint(MotorMode::Current, 0.0f);
    ASSERT_FALSE(mockCan.sentFrames.empty());
    auto& frame = mockCan.sentFrames.back();
    EXPECT_EQ(frame.id, 0x017FFF7F); //torque is extra in ID
    EXPECT_EQ(frame.len, 8);
}
