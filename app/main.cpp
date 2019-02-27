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
 * @file main.cpp
 * @brief  Main file to calculate poses using two measurement data
 * @author Bhargav Dandamudi
 * @version 1
 * @date 2019-02-27
 */
#include <iostream>
#include <vector>
#include "../include/PoseEstimator.hpp"
#include "../include/readData.hpp"

// Define dataset
std::string path = "/home/bob/Bob_old/Test/dataset0.csv";
// Change this path for difference files in system.

double wheel_rad{0.2};
double distance_between_front_back_wheels{1};
int ticks_per_revolution{35136};
double filter_gain{0.5};   // Complimentary filter gain
int main() {
    // Creating object of readData class to read from csv files
    readData r(path);
    r.getData();
    // Make vectors for time, steering_angle, encoder_ticks, and
    // angular_velocity
    auto all_time = r.getTime();
    auto all_steering_angle = r.getSteeringAngle();
    auto all_encoder_ticks = r.getEncoderTicks();
    auto all_angular_velocity = r.getAngularVelocity();

    // Define vehicle parameters for complimentary filter;
    PoseEstimator model(wheel_rad, distance_between_front_back_wheels,
                        ticks_per_revolution, filter_gain);

    // Initialising parameters for pose estimation
    double curr_time = 0;
    int encoder_ticks = 0;
    double steering_angle = 0;
    double angular_velocity = 0;

    std::ofstream predict_file;   // creating file to save output

    // Opening a file to save the prediced poses in it
    predict_file.open("/home/bob/Bob_old/Test/output_0.csv");

    // Main loop iterating over each measurement value
    for (int i = 1; i < int(all_time.size()); ++i) {
        curr_time = all_time[i];
        encoder_ticks = all_encoder_ticks[i];
        steering_angle = all_steering_angle[i];
        angular_velocity = all_angular_velocity[i];

        // storing predicted poses of the vehicle in vector
        auto estimated_pose = model.estimate(curr_time, steering_angle,
                                             encoder_ticks, angular_velocity);

        // Displaying the output for visualizing the output on console
        std::cout << "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx" << std::endl;
        std::cout << i << std::endl;
        std::cout << estimated_pose[0] << ": x position" << std::endl;
        std::cout << estimated_pose[1] << ": y position" << std::endl;
        std::cout << estimated_pose[2] << ": theta" << std::endl;

        // writing output in file
        predict_file << estimated_pose[0] << "," << estimated_pose[1] << ","
                     << estimated_pose[2] << std::endl;
    }
    predict_file.close();
    return 0;
}
