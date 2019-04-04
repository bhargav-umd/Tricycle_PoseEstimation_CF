# Tricycle_PoseEstimation_CF
[![Build Status](https://travis-ci.org/dpiet/cpp-boilerplate.svg?branch=master)](https://travis-ci.org/dpiet/cpp-boilerplate)
[![Coverage Status](https://coveralls.io/repos/github/dpiet/cpp-boilerplate/badge.svg?branch=master)](https://coveralls.io/github/dpiet/cpp-boilerplate?branch=master)
---

## Overview

Pose Estimation of a Tricyce based on encoder and gyroscope values given

## Approach
1. Calculate the position and orientation using basic kinematics on Encoder Values
2. Calculated the position and orientation using Gyroscope Measurement values
3. Use Complimentary filter to get Tricycle pose . 

TODO: Implement Kalman Filter for better prediction

## Standard install via command-line
```
unzip Bhargav_TriCycle_CF.zip
cd <path to repository>

vim app/main.cpp
```
change ```path``` variable to load the datasets
```
mkdir build
cd build
cmake ..
make
## To run the output and tests 
Run tests: ./test/cpp-test
Run program: ./app/shell-app
```

## Building for code coverage
```
sudo apt-get install lcov
cmake -D COVERAGE=ON -D CMAKE_BUILD_TYPE=Debug ../
make
make code_coverage
```

## Dataset
Download the dataset from the below link 
https://drive.google.com/open?id=1nkhp7D-udMCWT1J8LSaGG7aqDpJc1tIY

##Dependancies
No dependancies are needed to run the program
