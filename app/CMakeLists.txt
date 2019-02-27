add_executable(shell-app main.cpp PoseEstimator.cpp readData.cpp)
include_directories(
    ${CMAKE_SOURCE_DIR}/include
)
