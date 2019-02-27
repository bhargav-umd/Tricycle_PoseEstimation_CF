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
 * @file readData.hpp
 * @brief class to read data from give datasets
 * @author Bhargav Dandamudi
 * @version 1
 * @date 2019-02-27
 */
#ifndef READDATA_HPP_
#define READDATA_HPP_

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
// This class reads a CSV file
class readData {
 private:
    std::string path;   // path where dataset is located in system

    // Declaring variables to store columns in csv file
    std::vector<double> time_stamp;
    std::vector<int> encoder_values;
    std::vector<double> gyro_z;
    std::vector<double> steering_angle;

 public:
    // Default constructor
    readData(std::string);
    // Default Destructor
    ~readData();
    /* ----------------------------------------------------------------------*/
    /**
     * @brief  To read data from dataset file in given path
     */
    /* -----------------------------------------------------------------------*/
    void getData();

    /* -----------------------------------------------------------------------*/
    /**
     * @brief  to store Time stamps in vector
     *
     * @Returns vector of double containing time stamps
     * --------------------------------------------------------------------*/
    std::vector<double> getTime();
    /* ---------------------------------------------------------------------*/
    /**
     * @brief  to store steering angle in vector
     *
     * @Returns  vector of double containing steering angles
     * ----------------------------------------------------------------------*/
    std::vector<double> getSteeringAngle();
    /* ------------------------------------------------------------------------*/
    /**
     * @brief to store Encoder values
     *
     * @Returns vector of int encoder readings

     */
    /* --------------------------------------------------------------------------*/
    std::vector<int> getEncoderTicks();

    /* --------------------------------------------------------------------------*/
    /**
     * @brief To store gyroscope Angular Velocities in vector
     *
     * @Returns   vector of int containing angular velocties
     */
    /* --------------------------------------------------------------------------*/
    std::vector<double> getAngularVelocity();
};

#endif   // READDATA_HPP_
