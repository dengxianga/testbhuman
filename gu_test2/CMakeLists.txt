cmake_minimum_required(VERSION 3.0)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -m32 -nostdinc++")
set (CMAKE_INSTALL_RPATH "/home/nao/BHumanCodeRelease-master/Util/Buildchain/gcc/include/c++/5.2.0")
set (CMAKE_INSTALL_RPATH_USE_LINK_PATH true)
set(CPLUS_INCLUDE_PATH /home/nao/BHumanCodeRelease-master/Util/Buildchain/gcc/include/c++/5.2.0/)

include_directories(/home/nao/gup_standalone/ /home/nao/newnaoSDK/naoqi-sdk-2.1.4.13-linux32/include /usr/include/eigen3 /home/nao/BHumanCodeRelease-master/Util/Buildchain/gcc/include/c++/5.2.0/ /home/nao/BHumanCodeRelease-master/Util/Buildchain/gcc/include/ /home/nao/BHumanCodeRelease-master/Util/Buildchain/gcc/include/c++/5.2.0/i686-pc-linux-gnu/)

#include(CheckIncludeFiles)
#CHECK_INCLUDE_FILES(cmath CMATH_INCLUDE)
#IF (CMATH_INCLUDE)
#	MESSAGE("YAY")
#ELSE (CMATH_INCLUDE)
#	MESSAGE("no yay :(")
#ENDIF (CMATH_INCLUDE)

link_directories(/home/nao/newnaoSDK/naoqi-sdk-2.1.4.13-linux32/lib /home/nao/BHumanCodeRelease-master/Util/Buildchain/gcc/lib /home/nao/gup_standalone/build/)

add_executable(test_getup test.cpp)
target_link_libraries(test_getup standalone) 
