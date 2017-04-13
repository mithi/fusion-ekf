project(FusionEKF)

cmake_minimum_required(VERSION 3.5)
add_definitions(-std=c++0x)

include_directories(headers)

set(SOURCES src/main.cpp 
            src/usagecheck.cpp
            src/datapoint.cpp
            src/kalmanfilter.cpp)

add_executable(ExtendedKF ${SOURCES})