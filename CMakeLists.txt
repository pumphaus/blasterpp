cmake_minimum_required (VERSION 3.1)

project(BlasterPP)
set(CMAKE_CXX_STANDARD 17)

set(BLASTERPP_SRC
    mailbox.c mailbox.h blasterpp.h blasterpp.cpp
)

add_executable(blasterpp_example main.cpp ${BLASTERPP_SRC})
add_executable(read_rcw_0001 rcw_0001.cpp ${BLASTERPP_SRC})
