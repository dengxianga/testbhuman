cmake_minimum_required(VERSION 1.5)
project(test_dcmII)
set(CMAKE_BUILD_TYPE Release)

include_directories(/home/nao/testbhuman/libbhuman)

link_directories(/home/nao/testbhuman/libbhuman)

add_executable(test_dcmII
/home/nao/testbhuman/libbhuman/test_dcmII/test_dcmII.cpp)
target_link_libraries(test_dcmII dcmII)
