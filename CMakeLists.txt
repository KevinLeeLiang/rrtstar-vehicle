cmake_minimum_required(VERSION 3.21)
project(rrtstartsmart_vehicle)

set(CMAKE_CXX_STANDARD 14)

add_executable(rrtstartsmart_vehicle main.cpp rrttest.h rrt_star.h)
