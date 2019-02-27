/* Copyright (C)
 * 2019 - Bhargav Dandamudi
 *
 * MIT License
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the 'Software'), to deal in the Software without
 * restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit
 * persons to whom the Software is furnished to do so,subject to
 * the following conditions:
 * The above copyright notice and this permission notice shall
 * be included in all copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED ''AS IS'', WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR
 * ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM,OUT OF OR IN CONNECTION WITH
 * THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */
/**
 * @file test_pose_estimation.cpp
 * @brief  To test all the functions in Pose estimator class
 * @author Bhargav Dandamudi
 * @version 1
 * @date 2019-02-27
 */
#include <gtest/gtest.h>
#include "../include/PoseEstimator.hpp"
#include "../include/readData.hpp"
double wheel_rad{0.2}, distance_between_wheels{1}, gain{0.4};
int ticks_per_rev{512};

/* --------------------------------------------------------------------------*/
/**
 * @brief  Creating object to test each functionality of the program
 *
 * @Param wheel_rad
 * @Param distance_between_wheels
 * @Param ticks_per_rev
 * @Param gain
 *
 * @Returns   object initialised
 */
/* --------------------------------------------------------------------------*/
PoseEstimator pose_test(wheel_rad, distance_between_wheels, ticks_per_rev,
                        gain);

TEST(testPoseEstimatorClass, setterTest) {
    EXPECT_EQ(pose_test.setXEncoder(0.2), 0.2);
    EXPECT_EQ(pose_test.setYEncoder(0.2), 0.2);
    EXPECT_EQ(pose_test.setThetaEncoder(0.2), 0.2);

    EXPECT_EQ(pose_test.setXEncoderPrev(0.2), 0.2);
    EXPECT_EQ(pose_test.setYEncoderPrev(0.2), 0.2);
    EXPECT_EQ(pose_test.setThetaEncoderPrev(0.2), 0.2);

    EXPECT_EQ(pose_test.setXEncoderDot(0.2), 0.2);
    EXPECT_EQ(pose_test.setYEncoderDot(0.2), 0.2);
    EXPECT_EQ(pose_test.setThetaEncoderDot(0.2), 0.2);

    EXPECT_EQ(pose_test.setXGyro(0.2), 0.2);
    EXPECT_EQ(pose_test.setYGyro(0.2), 0.2);
    EXPECT_EQ(pose_test.setThetaGyro(0.2), 0.2);

    EXPECT_EQ(pose_test.setXGyroPrev(0.2), 0.2);
    EXPECT_EQ(pose_test.setYGyroPrev(0.2), 0.2);
    EXPECT_EQ(pose_test.setThetaGyroPrev(0.2), 0.2);

    EXPECT_EQ(pose_test.setXGyroDot(0.2), 0.2);
    EXPECT_EQ(pose_test.setYGyroDot(0.2), 0.2);
    EXPECT_EQ(pose_test.setThetaGyroDot(0.2), 0.2);
}

/* --------------------------------------------------------------------------*/
/**
 * @brief  TO test all calculations done in PoseEstimation Class
 *
 * @Param testPoseEstimatorClass
 * @Param calculateFunctionsTest
 */
/* --------------------------------------------------------------------------*/
TEST(testPoseEstimatorClass, calculateFunctionsTest) {
    double a{0.2}, b{1}, d{0.4};
    int c{512};

    PoseEstimator calculateTest(a, b, c, d);

    double time_current = calculateTest.setTimeCurr(5);

    double time_prev = calculateTest.setTimePrev(4);
    double wheel_rad = calculateTest.setWheelRad(0.2);
    double dist_ = calculateTest.setDistaceBetWheels(1);
    double steering_angle = calculateTest.setSteeringAngle(0.3);
    int prev_enc_ticks = calculateTest.setEncoderTicksPrev(33);
    int curr_enc_ticks = calculateTest.setEncoderTicksCurr(34);
    double theta_dot = calculateTest.setAngularVelocity(2);
    double fgain = calculateTest.setFiltergain(0.5);
    double theta_gyro_dot = calculateTest.setThetaGyroDot(2);

    EXPECT_EQ(calculateTest.calculateDt(), 1);
    EXPECT_NEAR(calculateTest.calculateSteeringVelocity(), 0.00245436, 1e-4);
    EXPECT_NEAR(calculateTest.calculateThetaEncoderDot(), 7.253157111e-4, 1e-4);
    EXPECT_NEAR(calculateTest.integrateVariable(2, 7.253157111e-4), 2.000725316,
                1e-4);   // used theta_ecoderdot values to verify
    EXPECT_NEAR(calculateTest.calculateXEncoderDot(), 2.13357452e-3, 1e-2);
    EXPECT_NEAR(calculateTest.calculateYEncoderDot(), 8.185989854e-5, 1e-2);

    EXPECT_NEAR(calculateTest.calculateSteeringVelocityGyro(), 6.767726724,
                1e-4);
    EXPECT_EQ(calculateTest.integrateVariable(1.1, 2), 3.1);

    double theta_gyro = calculateTest.setThetaGyro(3.1);
    int counter = calculateTest.setCounter(3);
    EXPECT_NEAR(calculateTest.calculateXGyroDot(), -6.459908498, 1e-3);
    EXPECT_NEAR(calculateTest.calculateYGyroDot(), 0.26883978, 1e-3);
    std::vector<double> pose_temp{-1.34412, 2.93951, 1.000361};

    std::vector<double> pose_calc = calculateTest.estimate(
        time_current, steering_angle, curr_enc_ticks, theta_gyro_dot);
    EXPECT_NEAR(pose_calc[0], pose_temp[0], 1e-3);
    EXPECT_NEAR(pose_calc[1], pose_temp[1], 1e-3);
    EXPECT_NEAR(pose_calc[2], pose_temp[2], 1e-3);
}

/* --------------------------------------------------------------------------*/
/**
 * @brief  TO test complimentary filter output
 *
 * @Param testPoseEstimatorClass
 * @Param compliFilterTest
 */
/* --------------------------------------------------------------------------*/
TEST(testPoseEstimatorClass, compliFilterTest) {
    double a{0.2}, b{1}, d{0.4};
    int c{512};

    PoseEstimator compliFilterTest(a, b, c, d);

    EXPECT_EQ(compliFilterTest.complimentaryFilter(3, 4), 3.6);
}
