cmake_minimum_required(VERSION 1.5)
project(bhuman_dcmII)
set(CMAKE_BUILD_TYPE Release)

include_directories(/home/nao/testbhuman/libbhuman
/home/nao/testbhuman/libbhuman/low
/home/nao/newnaoSDK/naoqi-sdk-2.1.4.13-linux32/include)

link_directories(/home/nao/newnaoSDK/naoqi-sdk-2.1.4.13-linux32/lib)

add_library(bhuman SHARED /home/nao/testbhuman/libbhuman/bhuman.cpp)

set_target_properties(bhuman PROPERTIES LIBRARY_OUTPUT_DIRECTORY /home/nao/testbhuman/libbhuman)
