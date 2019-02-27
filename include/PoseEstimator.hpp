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
 * @file PoseEstimator.hpp
 * @brief  class to calculate pose values from the given data
 * @author Bhargav Dandamudi
 * @version 1
 * @date 2019-02-27
 */
#ifndef POSEESTIMATOR_HPP_
#define POSEESTIMATOR_HPP_
#include <math.h>
#include <iostream>
#include <vector>
using std::vector;

/* --------------------------------------------------------------------------*/
/**
 * @brief  Main Pose Estimator Class carrying out all
 * operations using complimentary filter
 */
/* --------------------------------------------------------------------------*/
class PoseEstimator {
 private:
    int counter;        // counter for checking first iteration
    double wheel_rad;   // Rear Wheel radius
    int ticks_per_revolution;
    double distance_between_front_back_wheels;
    double dt;   // dt = delta t for difference in time between each iteration
    double t_curr;   // current value of time
    double t_prev;   // previous iteration time value
    double steering_angle;
    int encoder_ticks_curr;   // current encoder tick value
    int encoder_ticks_prev;
    double x_encoder;   // distance travelled in x direction as calculated using
                        // encoder values
    double y_encoder;   // distance travelled in y direction as calculated using
                        // encoder values

    double theta_encoder;    // theta as measuring using encoder values
    double x_encoder_prev;   // previous value of x encoder
    double y_encoder_prev;   // previous value of y encoder
    double theta_encoder_prev;
    double x_encoder_dot;
    double y_encoder_dot;
    double theta_encoder_dot;
    double steering_velocity_encoder;
    double theta_gyro_dot;
    double theta_gyro;
    double steering_velocity_gyro;
    double x_gyro_dot;
    double y_gyro_dot;
    double x_gyro_prev;
    double y_gyro_prev;
    double theta_gyro_prev;
    double x_gyro;
    double y_gyro;
    double filter_gain;
    double steering_angle_limit;   // cannot rotate front wheel more than 90
                                   // Degree

 public:
    /* ---------------------------------------------------------------------*/
    /**
     * @brief  Constructor
     *
     * @Param double wheel radius
     * @Param double distance between front and back wheels
     * @Param int ticks per revolution
     * @Param double complimentary filter gain
     */
    /* ---------------------------------------------------------------------*/
    PoseEstimator(double&, double&, int&, double&);

    /* ---------------------------------------------------------------------*/
    /**
     * @brief  To process steering angle if its more than 90 degree
     *
     */
    /* ---------------------------------------------------------------------*/
    void preprocessSteeringAngle();

    /* ---------------------------------------------------------------------*/
    /**
     * @brief  To Estimate the poses of the tricycle using given data
     *
     * @Param double current time
     * @Param double steering angle
     * @Param int      encoder ticks
     * @Param double  angular velocity as measured by gyroscope
     *
     * @Returns   poses
     */
    /* ---------------------------------------------------------------------*/
    vector<double> estimate(double&, double&, int&, double&);

    /* ---------------------------------------------------------------------*/
    /**
     * @brief  To integrate velocity to get position ;angular velocity to get
     * theta
     *
     * @Param double previous value
     * @Param double  position/theta value to be integrated
     *
     * @Returns
     */
    /* ---------------------------------------------------------------------*/
    double integrateVariable(double, double);
    /* ---------------------------------------------------------------------*/
    /**
     * @brief  Complimentary Filter which combines encoder and gyroscope
     * measurements to get good predictions
     *
     * @Param double encoder estimated pose
     * @Param double gyroscope estimated pose
     *
     * @Returns   Predicted Pose
     */
    /* ---------------------------------------------------------------------*/
    double complimentaryFilter(double, double);

    // functions to calculate variables

    /* ---------------------------------------------------------------------*/
    /**
     * @brief  To calculate the values of private variables
     *
     * @Returns
     */
    /* ---------------------------------------------------------------------*/

    double calculateDt();
    double calculateSteeringVelocity();
    double calculateThetaEncoderDot();
    double calculateXEncoderDot();
    double calculateYEncoderDot();
    double calculateSteeringVelocityGyro();
    double calculateXGyroDot();
    double calculateYGyroDot();

    // functions to set values in variables

    /* ---------------------------------------------------------------------*/
    /**
     * @brief  to set values to class private variables
     *
     * @Param int/double
     *
     * @Returns   input value to be set
     */
    /* ---------------------------------------------------------------------*/

    int setCounter(int);
    double setXEncoder(double);
    double setYEncoder(double);
    double setThetaEncoder(double);
    double setXEncoderPrev(double);
    double setYEncoderPrev(double);
    double setThetaEncoderPrev(double);
    double setXEncoderDot(double);
    double setYEncoderDot(double);
    double setThetaEncoderDot(double);
    double setXGyro(double);
    double setYGyro(double);
    double setThetaGyro(double);
    double setXGyroPrev(double);
    double setThetaGyroPrev(double);
    double setYGyroPrev(double);
    double setXGyroDot(double);
    double setYGyroDot(double);
    double setThetaGyroDot(double);

    // initial parameters and remaining variable getters
    double setTimeCurr(double);
    double setTimePrev(double);
    double setWheelRad(double);
    double setDistaceBetWheels(double);
    double setSteeringAngle(double);
    int setEncoderTicksPrev(int);
    int setEncoderTicksCurr(int);
    double setAngularVelocity(double);
    double setFiltergain(double);
};
#endif   // POSEESTIMATOR_HPP_
