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
 * @file readData.cpp
 * @brief  reading data from csv file and storing them in vectors
 * @author Bhargav Dandamudi
 * @version 1
 * @date 2019-02-27
 */
#include "../include/readData.hpp"

readData::readData(std::string path) { readData::path = path; }

void readData::getData() {
    std::ifstream data(readData::path);
    if (!data.is_open()) {
        exit(EXIT_FAILURE);
    }
    std::string str;   // to store individual values;

    // Looping through each row of the file to read the values
    while (getline(data, str)) {
        std::istringstream iss(str);
        std::string token;

        // storing each value to its respective vector
        getline(iss, token, ',');
        this->time_stamp.push_back(atof(token.c_str()));

        getline(iss, token, ',');
        this->encoder_values.push_back(atof(token.c_str()));

        getline(iss, token, ',');
        this->gyro_z.push_back(atof(token.c_str()));

        getline(iss, token, ',');
        this->steering_angle.push_back(atof(token.c_str()));
    }
}

/* --------------------------------------------------------------------------*/
/**
 * @brief storing each value to its respective vector
 *
 * @Returns vector
 */
/* --------------------------------------------------------------------------*/

std::vector<double> readData::getTime() { return this->time_stamp; }

std::vector<double> readData::getSteeringAngle() {
    return this->steering_angle;
}

std::vector<int> readData::getEncoderTicks() { return this->encoder_values; }

std::vector<double> readData::getAngularVelocity() { return this->gyro_z; }
// Default Destructor;
readData::~readData() {}
