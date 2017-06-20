cmake_minimum_required(VERSION 1.5)

SET(BH_SRCS
	bhuman.cpp)

SET(DCM_SRCS
	dcmII.cpp)

SET(LUADCM_SRCS
	lua_dcmII.cpp)

include_directories(/home/nao/newnaoSDK/naoqi-sdk-2.1.4.13-linux32/include)
link_directories(/usr/local/lib /home/nao/local/lib /home/nao/newnaoSDK/naoqi-sdk-2.1.4.13-linux32/lib)

find_package(Boost COMPONENTS system program_options thread serialization regex unit_test_framework REQUIRED)

ADD_LIBRARY(bhuman SHARED ${BH_SRCS})
target_link_libraries(bhuman alcommon alerror alproxies almemoryfastaccess ${Boost_LIBRARIES})

ADD_LIBRARY(dcmII SHARED ${DCM_SRCS})
ADD_LIBRARY(lua_dcmII SHARED ${LUADCM_SRCS})

SET_TARGET_PROPERTIES(bhuman PROPERTIES OUTPUT_NAME "bhuman")
SET_TARGET_PROPERTIES(bhuman PROPERTIES PREFIX "lib")
SET_TARGET_PROPERTIES(dcmII PROPERTIES OUTPUT_NAME "dcmII")
SET_TARGET_PROPERTIES(dcmII PROPERTIES PREFIX "lib")
SET_TARGET_PROPERTIES(lua_dcmII PROPERTIES OUTPUT_NAME "lua_dcmII")
SET_TARGET_PROPERTIES(lua_dcmII PROPERTIES PREFIX "lib")

SET_TARGET_PROPERTIES(bhuman PROPERTIES CLEAN_DIRECT_OUTPUT 1)
SET_TARGET_PROPERTIES(dcmII PROPERTIES CLEAN_DIRECT_OUTPUT 1)
SET_TARGET_PROPERTIES(lua_dcmII PROPERTIES CLEAN_DIRECT_OUTPUT 1)




# include_directories(/home/nao/testbhuman/libbhuman ${BOOST_INCLUDE_DIRS})
# link_directories(/home/nao/testbhuman/libbhuman/build /usr/local/lib ${BOOST_LIBRARY_DIRS} /home/nao/local/lib /home/nao/newnaoSDK/naoqi-sdk-2.1.4.13-linux32/lib)

# add_executable(test_dcmII test_dcmII.cpp)
# target_link_libraries(test_dcmII dcmII alcommon alerror alproxies almemoryfastaccess ${BOOST_LIBRARIES}) 

MESSAGE("CMAKE_SOURCE_DIR: " ${CMAKE_SOURCE_DIR})