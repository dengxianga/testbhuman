cmake_minimum_required(VERSION 1.5)
project(test_dcmII)

find_package(Boost COMPONENTS system program_options thread serialization regex unit_test_framework REQUIRED)

include_directories(/home/nao/testbhuman/libbhuman ${Boost_INCLUDE_DIRS} /home/nao/newnaoSDK/naoqi-sdk-2.1.4.13-linux32/include)
link_directories(/home/nao/testbhuman/libbhuman /usr/local/lib ${Boost_LIBRARY_DIRS} /home/nao/local/lib /home/nao/newnaoSDK/naoqi-sdk-2.1.4.13-linux32/lib )

add_executable(test_dcmII test_dcmII.cpp)

target_link_libraries(test_dcmII dcmII bhuman)

# cmake_minimum_required(VERSION 1.5)

# project(test_dcmII)

# find_package(Boost COMPONENTS system program_options thread serialization regex unit_test_framework REQUIRED)

# include_directories(/home/nao/testbhuman/libbhuman ${Boost_INCLUDE_DIRS} /home/nao/newnaoSDK/naoqi-sdk-2.1.4.13-linux32/include)
# link_directories(/home/nao/testbhuman/libbhuman /usr/local/lib ${Boost_LIBRARY_DIRS} /home/nao/local/lib /home/nao/newnaoSDK/naoqi-sdk-2.1.4.13-linux32/lib )

# add_executable(test_dcmII test_dcmII.cpp)
# target_link_libraries(test_dcmII dcmII bhuman alcommon alerror alproxies almemoryfastaccess) 

# MESSAGE("CMAKE_SOURCE_DIR: " ${CMAKE_SOURCE_DIR})
