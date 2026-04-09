#include "IMUOrientation.h"
#include "MotorControl.h"
#include "mock_Arduino.h"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <string>

using ::testing::_;
using ::testing::AtLeast;
using ::testing::NiceMock;

namespace
{
std::string writeImuCsv(const std::string &name, const std::string &content)
{
    const std::string path = std::string("/tmp/") + name;
    std::ofstream file(path);
    file << content;
    file.close();
    return path;
}

class RealMotorControlTest : public ::testing::Test {
protected:
    NiceMock<MockArduino> mockArduino;
};

class RealIMUOrientationTest : public ::testing::Test {
protected:
    NiceMock<MockArduino> mockArduino;
    unsigned long nowMicros = 0;

    void SetUp() override
    {
        ON_CALL(mockArduino, micros()).WillByDefault([this]() { return nowMicros; });
        ON_CALL(mockArduino, delay(_)).WillByDefault([this](int ms) {
            nowMicros += static_cast<unsigned long>(ms) * 1000UL;
        });
    }

    void TearDown() override
    {
        unsetenv("IMU_CSV");
        unsetenv("IMU_LOOP");
        unsetenv("IMU_BEGIN_FAIL");
    }
};
}

TEST_F(RealMotorControlTest, InitSetSpeedAndStopExercisesPwmPaths)
{
    MotorControl motor(1, 2, 3, 4);

    EXPECT_CALL(mockArduino, pinMode(_, _)).Times(4);
    EXPECT_CALL(mockArduino, hw_timer_pause()).Times(2);
    EXPECT_CALL(mockArduino, setPWM(_, _, _, _)).Times(4);
    EXPECT_CALL(mockArduino, setCaptureCompare(_, _, _)).Times(AtLeast(16));

    motor.init(20000);
    motor.setMotorSpeeds(65, 40, true, false);
    motor.setMotorSpeeds(20, 80, false, true);
    motor.stopBothMotors();
}

TEST_F(RealIMUOrientationTest, CalibrateAndUpdateProduceFiniteAngles)
{
    const std::string csvPath = writeImuCsv("imu_cov_nominal.csv",
                                            "0.0,0.0,9.80665,0.0,0.0,0.01,25.0\n"
                                            "0.2,0.1,9.70,0.02,-0.01,0.03,24.9\n");

    setenv("IMU_CSV", csvPath.c_str(), 1);
    setenv("IMU_LOOP", "1", 1);

    IMUOrientation imu;
    ASSERT_TRUE(imu.begin());

    imu.calibrate(false);

    nowMicros += 20000;
    imu.update();

    EXPECT_TRUE(std::isfinite(imu.roll()));
    EXPECT_TRUE(std::isfinite(imu.pitch()));
    EXPECT_TRUE(std::isfinite(imu.yaw()));
}

TEST_F(RealIMUOrientationTest, CalibratePrintPathAndNegativeYawClampAreCovered)
{
    const std::string csvPath = writeImuCsv("imu_cov_negative_wrap.csv",
                                            "0.0,0.0,9.80665,0.0,0.0,-8.0,25.0\n"
                                            "0.0,0.0,9.80665,0.0,0.0,-8.0,25.0\n");

    setenv("IMU_CSV", csvPath.c_str(), 1);
    setenv("IMU_LOOP", "1", 1);

    IMUOrientation imu;
    ASSERT_TRUE(imu.begin());

    /* Cover printing branch in calibrate(). */
    imu.calibrate(true);

    /* Separate instance without calibration so raw negative gyro reaches clamp path. */
    nowMicros = 3000000;
    IMUOrientation unclampedOffsetImu;
    ASSERT_TRUE(unclampedOffsetImu.begin());

    nowMicros = 4000000;
    unclampedOffsetImu.update();
    nowMicros = 5000000;
    unclampedOffsetImu.update();

    /* rawGz ~= -458 deg/s, clamped to -400 and integrated twice (2s) -> wrapped around -80. */
    EXPECT_NEAR(unclampedOffsetImu.gyroZDeg(), -400.0f, 1.5f);
    EXPECT_NEAR(unclampedOffsetImu.yaw(), -80.0f, 3.0f);
}

TEST_F(RealIMUOrientationTest, BeginCanFailWhenBackendReportsFailure)
{
    setenv("IMU_BEGIN_FAIL", "1", 1);

    IMUOrientation imu;
    EXPECT_FALSE(imu.begin());
}

TEST_F(RealIMUOrientationTest, SpikeRejectAndClampPathsAreApplied)
{
    const std::string csvPath = writeImuCsv("imu_cov_spike.csv",
                                            "0.0,0.0,9.80665,0.0,0.0,20.0,25.0\n" /* spike > 800
                                                                                     deg/s */
                                            "0.0,0.0,9.80665,0.0,0.0,10.0,25.0\n"); /* clamp to 400
                                                                                       deg/s */

    setenv("IMU_CSV", csvPath.c_str(), 1);
    setenv("IMU_LOOP", "1", 1);

    IMUOrientation imu;
    ASSERT_TRUE(imu.begin());

    nowMicros = 1000000;
    imu.update();
    EXPECT_NEAR(imu.gyroZDeg(), 0.0f, 1.0f);

    nowMicros = 2000000;
    imu.update();

    EXPECT_NEAR(imu.gyroZDeg(), 400.0f, 1.0f);
    EXPECT_NEAR(imu.yaw(), 40.0f, 2.5f);
}
