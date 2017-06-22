cmake_minimum_required(VERSION 1.5)
project(test_dcmII)
set(CMAKE_BUILD_TYPE Release)

include_directories(/home/nao/testbhuman/libbhuman)

link_directories(/home/nao/testbhuman/libbhuman)

add_executable(print_positions
/home/nao/testbhuman/libbhuman/print_positions/print_positions.cpp)
target_link_libraries(print_positions dcmII)
