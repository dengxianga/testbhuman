cmake_minimum_required(VERSION 1.5)
project(bhuman_dcmII)
set(CMAKE_BUILD_TYPE Release)

include_directories(/home/nao/testbhuman/libbhuman)

link_directories(/home/nao/testbhuman/libbhuman)

add_library(dcmII SHARED /home/nao/testbhuman/libbhuman/dcmII.cpp)

set_target_properties(dcmII PROPERTIES LIBRARY_OUTPUT_DIRECTORY /home/nao/testbhuman/libbhuman)

TARGET_LINK_LIBRARIES(dcmII ${Boost_LIBRARIES} rt)
