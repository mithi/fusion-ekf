project(ExtendedKF)

cmake_minimum_required(VERSION 3.5)
add_definitions(-std=c++0x)

include_directories(headers)

set(SOURCES src/main.cpp 
            src/usagecheck.cpp
            src/tools.cpp
            src/datapoint.cpp
            src/kalmanfilter.cpp
            src/fusionekf.cpp)

add_executable(ExtendedKF ${SOURCES})