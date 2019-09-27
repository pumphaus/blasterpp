cmake_minimum_required (VERSION 3.1)

project(BlasterPP)
set(CMAKE_CXX_STANDARD 17)
add_executable(blasterpp_example main.cpp mailbox.c mailbox.h blasterpp.h blasterpp.cpp)
