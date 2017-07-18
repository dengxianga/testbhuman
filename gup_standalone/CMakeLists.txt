cmake_minimum_required(VERSION 3.0)

#SET(CMAKE_CXX_COMPILER /home/nao/gcc-5.2.0/bin/g++)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -m32 -nostdinc++")
set (CMAKE_INSTALL_RPATH "/home/nao/BHumanCodeRelease-master/Util/Buildchain/gcc/include/c++/5.2.0")
set (CMAKE_INSTALL_RPATH_USE_LINK_PATH true)
set(CPLUS_INCLUDE_PATH /home/nao/BHumanCodeRelease-master/Util/Buildchain/gcc/include/c++/5.2.0/)
#set (CMAKE_CXX_LINK_EXECUTABLE /home/nao/BHumanCodeRelease-master/Buildchain/gcc/lib/ld-2.13.so)

include_directories(/home/nao/gup_standalone/ /home/nao/newnaoSDK/naoqi-sdk-2.1.4.13-linux32/include /usr/include/eigen3 /home/nao/BHumanCodeRelease-master/Util/Buildchain/gcc/include/c++/5.2.0/ /home/nao/BHumanCodeRelease-master/Util/Buildchain/gcc/include/ /home/nao/BHumanCodeRelease-master/Util/Buildchain/gcc/include/c++/5.2.0/i686-pc-linux-gnu/)

link_directories(/home/nao/newnaoSDK/naoqi-sdk-2.1.4.13-linux32/lib /home/nao/BHumanCodeRelease-master/Util/Buildchain/gcc/lib)

file(GLOB allSRC "*.cpp")

add_library(standalone SHARED ${allSRC})

find_package(Threads REQUIRED)
if(THREADS_HAVE_PTHREAD_ARG)
  set_property(TARGET standalone PROPERTY COMPILE_OPTIONS "-pthread")
  set_property(TARGET standalone PROPERTY INTERFACE_COMPILE_OPTIONS "-pthread")
endif()
if(CMAKE_THREAD_LIBS_INIT)
  target_link_libraries(standalone "${CMAKE_THREAD_LIBS_INIT}")
endif()
