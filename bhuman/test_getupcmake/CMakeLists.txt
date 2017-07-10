cmake_minimum_required(VERSION 2.5)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

SET(CMAKE_C_COMPILER /home/nao/gcc-5.2.0/bin/gcc)
SET(CMAKE_CXX_COMPILER /home/nao/gcc-5.2.0/bin/g++)

include_directories(/home/nao/newnaoSDK/naoqi-sdk-2.1.4.13-linux32/include /home/nao/bhuman/Src/ /home/nao/bhuman/Src/Modules/MotionControl /usr/lib/eigen3 /home/nao/bhuman/Util/GameController/include)

link_directories(/home/nao/newnaoSDK/naoqi-sdk-2.1.4.13-linux32/lib /home/nao/bhuman/Src/ /home/nao/bhuman/getuplib)

add_executable(test_getup test_getup.cpp)
target_link_libraries(test_getup bhGetUp)
