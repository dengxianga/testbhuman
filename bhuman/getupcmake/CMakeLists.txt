cmake_minimum_required(VERSION 2.5)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

SET(CMAKE_C_COMPILER /home/nao/gcc-5.2.0/bin/gcc)
SET(CMAKE_CXX_COMPILER /home/nao/gcc-5.2.0/bin/g++)

include_directories(/home/nao/newnaoSDK/naoqi-sdk-2.1.4.13-linux32/include 
					/home/nao/bhuman/Src/ 
					/home/nao/bhuman/Util/ 
					/usr/lib/eigen3/
					/home/nao/bhuman/Src/Platform/ 
					/home/nao/bhuman/Util/Buildchain/gcc/include/ 
					/home/nao/bhuman/Config/Locations/Default/ 
					/home/nao/bhuman/Util/GameController/include/)

link_directories(/home/nao/newnaoSDK/naoqi-sdk-2.1.4.13-linux32/lib 
				 /home/nao/bhuman/Src/ 
				 /home/nao/bhuman/getuplib)

add_library(bhGetUp SHARED /home/nao/bhuman/Src/Modules/MotionControl/GetUpEngine.cpp
						   /home/nao/bhuman/Src/Modules/MotionControl/MotionCombinator.cpp
						   /home/nao/bhuman/Src/Tools/Streams/OutStreams.cpp
						   /home/nao/bhuman/Src/Tools/Module/Module.cpp
						   /home/nao/bhuman/Src/Tools/Debugging/TimingManager.cpp
						   /home/nao/bhuman/Src/Tools/Module/Blackboard.cpp
						   /home/nao/bhuman/Src/Tools/Streams/InStreams.cpp
						   /home/nao/bhuman/Src/Tools/Global.cpp
						   /home/nao/bhuman/Src/Tools/Debugging/DebugRequest.cpp
						   /home/nao/bhuman/Src/Tools/MessageQueue/InMessage.cpp
						   /home/nao/bhuman/Src/Platform/Linux/Time.cpp
						   /home/nao/bhuman/Src/Tools/Streams/Streamable.cpp
						   /home/nao/bhuman/Src/Platform/File.cpp
						   /home/nao/bhuman/Src/Tools/MessageQueue/MessageQueueBase.cpp
						   /home/nao/bhuman/Src/Platform/Linux/SystemCall.cpp
						   /home/nao/bhuman/Src/Tools/Streams/SimpleMap.cpp
						   /home/nao/bhuman/Src/Modules/MotionControl/KickEngine/KickEngineParameters.cpp
						   /home/nao/bhuman/Src/Tools/MessageQueue/OutMessage.cpp
						   /home/nao/bhuman/Src/Platform/Linux/SoundPlayer.cpp
						   /home/nao/bhuman/Src/Tools/Streams/StreamHandler.cpp
						   /home/nao/bhuman/Src/Representations/Infrastructure/CameraInfo.cpp
						   /home/nao/bhuman/Src/Platform/Linux/BHAssert.cpp
						   /home/nao/bhuman/Src/Tools/Streams/Enum.cpp
						   /home/nao/bhuman/Src/Platform/Linux/Semaphore.cpp
						   /home/nao/bhuman/Src/Tools/Streams/InOut.cpp
						   /home/nao/bhuman/Src/Representations/Infrastructure/JointAngles.cpp
						   /home/nao/bhuman/Src/Platform/Linux/File.cpp
						   /home/nao/bhuman/Src/Platform/Linux/ThreadBase.cpp
						   /home/nao/bhuman/Src/Representations/Infrastructure/RobotInfo.cpp
						   /home/nao/bhuman/Src/Modules/MotionControl/ArmMotionCombinator.cpp
						   /home/nao/bhuman/Src/Modules/MotionControl/LegMotionCombinator.cpp
						   /home/nao/bhuman/Src/Modules/MotionControl/MotionSelector.cpp
						   /home/nao/bhuman/Src/Representations/MotionControl/LegMotionSelection.cpp
						   /home/nao/bhuman/Src/Representations/Sensing/FallDownState.cpp
						   /home/nao/bhuman/Src/Representations/Sensing/InertialData.cpp
						   /home/nao/bhuman/Src/Representations/MotionControl/SpecialActionRequest.cpp
						   /home/nao/bhuman/Src/Modules/MotionControl/SpecialActions.cpp
						   /home/nao/bhuman/Src/Tools/Debugging/ColorRGBA.cpp
						   /home/nao/bhuman/Src/Tools/Math/Random.cpp
						   /home/nao/bhuman/Src/Tools/Debugging/DebugDrawings.cpp
						   /home/nao/bhuman/Src/Tools/Motion/MofCompiler.cpp)

set_target_properties(bhGetUp PROPERTIES LIBRARY_OUTPUT_DIRECTORY /home/nao/bhuman/getuplib)

find_package(Threads REQUIRED)
if(THREADS_HAVE_PTHREAD_ARG)
  set_property(TARGET bhGetUp PROPERTY COMPILE_OPTIONS "-pthread")
  set_property(TARGET bhGetUp PROPERTY INTERFACE_COMPILE_OPTIONS "-pthread")
endif()
if(CMAKE_THREAD_LIBS_INIT)
  target_link_libraries(bhGetUp "${CMAKE_THREAD_LIBS_INIT}")
endif()

target_link_libraries(bhGetUp rt)