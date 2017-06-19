#include <sys/types.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <semaphore.h>
#include <csignal>
#include <sys/resource.h>
#include <ctime>
#include <cstring>
#include <sys/time.h>

#ifdef __clang__
#pragma clang diagnostic push

#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma clang diagnostic ignored "-Wconversion"
#pragma clang diagnostic ignored "-Wunused-variable"
#pragma clang diagnostic ignored "-Wunused-local-typedef"
#endif
#define BOOST_SIGNALS_NO_DEPRECATION_WARNING
#include <alcommon/albroker.h>
#include <alcommon/alproxy.h>
#include <alproxies/dcmproxy.h>
#include <alproxies/almemoryproxy.h>
#undef BOOST_SIGNALS_NO_DEPRECATION_WARNING
#ifdef __clang__
#pragma clang diagnostic pop
#endif

#include <iostream>
#include <lua.hpp>

#include "RoboCupGameControlData.h"
#include "bhuman.h"
using namespace std;
double speed = 0.01;
double current_time = 0;

enum jointNames { 
HeadYaw = 0, 
HeadPitch,
LShoulderPitch, 
LShoulderRoll,
LElbowYaw, 
LElbowRoll,
LHipYawPitch, 
LHipRoll, 
LHipPitch,
LKneePitch, 
LAnklePitch, 
LAnkleRoll,
RHipYawPitch,
RHipRoll, 
RHipPitch,
RKneePitch, 
RAnklePitch, 
RAnkleRoll,
RShoulderPitch, 
RShoulderRoll,
RElbowYaw, 
RElbowRoll, 
nJoint};

const int indexHead = HeadYaw;
const int nJointHead = 2;
const int indexLArm = LShoulderPitch;
const int nJointLArm = 4;
const int indexLLeg = LHipYawPitch;
const int nJointLLeg = 6;
const int indexRLeg = RHipYawPitch;
const int nJointRLeg = 6;
const int indexRArm = RShoulderPitch;
const int nJointRArm = 4;

// std::map<int, int> luaToBHumanPos;
// std::map<int,int>::iterator bHumanIndex;

// {HeadYaw, headYawPositionActuator},
// {HeadPitch, headPitchPositionActuator},
// {LShoulderPitch, lShoulderPitchPositionActuator},
// {LShoulderRoll, lShoulderRollPositionActuator},
// {LElbowYaw, lElbowYawPositionActuator},
// {LElbowRoll, lElbowRollPositionActuator},
// {LHipYawPitch, lHipYawPitchPositionActuator},
// {LHipRoll, lHipRollPositionActuator},
// {LHipPitch, lHipPitchPositionActuator},
// {LKneePitch, lKneePitchPositionActuator},
// {LAnklePitch, lAnklePitchPositionActuator},
// {LAnkleRoll, lAnkleRollPositionActuator},
// {RHipYawPitch, lHipYawPitchPositionActuator},
// {RHipRoll, rHipRollPositionActuator},
// {RHipPitch, rHipPitchPositionActuator},
// {RKneePitch, rKneePitchPositionActuator},
// {RAnklePitch, rAnklePitchPositionActuator},
// {RAnkleRoll, rAnkleRollPositionActuator},
// {RShoulderPitch, rShoulderPitchPositionActuator},
// {RShoulderRoll, rShoulderRollPositionActuator},
// {RElbowYaw, rElbowYawPositionActuator},
// {RElbowRoll, rElbowRollPositionActuator}

int luaToBHumanPos[nJoint] = {
headYawPositionActuator, 
headPitchPositionActuator,  
lShoulderPitchPositionActuator,
lShoulderRollPositionActuator,  
lElbowYawPositionActuator,  
lElbowRollPositionActuator,  
lHipYawPitchPositionActuator, 
lHipRollPositionActuator, 
lHipPitchPositionActuator,  
lKneePitchPositionActuator,  
lAnklePitchPositionActuator,  
lAnkleRollPositionActuator, 
lHipYawPitchPositionActuator, 
rHipRollPositionActuator, 
rHipPitchPositionActuator,  
rKneePitchPositionActuator, 
rAnklePitchPositionActuator,  
rAnkleRollPositionActuator, 
rShoulderPitchPositionActuator,  
rShoulderRollPositionActuator,  
rElbowYawPositionActuator,  
rElbowRollPositionActuator};

int fd;
//int memoryHandle = shm_open(LBH_MEM_NAME, O_CREAT | O_RDWR, S_IRUSR | S_IWUSR);
LBHData * data;
bool initialized = false;
int writingActuators = -1;

// static int luaBH_getdummy (lua_State *L) {
//   lua_pushnumber(L, 0);
//   // TODO try to manipulate the shared memory "data"
//   fd = shm_open(LBH_MEM_NAME, O_RDWR, S_IRUSR | S_IWUSR);
//   data = (LBHData*)mmap(NULL, sizeof(LBHData), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);

//   for (int i=0;i<10000;i++){
//     float * actuators = data->actuators[data->newestActuators];
//     std::cout<<"shm val actuators[0] " << actuators[0]<< " \n";
//   }

//   return 1;
// }

static void lua_initialize () {
  fd = shm_open(LBH_MEM_NAME, O_RDWR, S_IRUSR | S_IWUSR);
  data = (LBHData*)mmap(NULL, sizeof(LBHData), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  initialized = true;

  // luaToBHumanPos[HeadYaw] = headYawPositionActuator;
  // luaToBHumanPos[HeadPitch] = headPitchPositionActuator;
  // luaToBHumanPos[LShoulderPitch] = lShoulderPitchPositionActuator;
  // luaToBHumanPos[LShoulderRoll] = lShoulderRollPositionActuator;
  // luaToBHumanPos[LElbowYaw] = lElbowYawPositionActuator;
  // luaToBHumanPos[LElbowRoll] = lElbowRollPositionActuator;
  // luaToBHumanPos[LHipYawPitch] = lHipYawPitchPositionActuator;
  // luaToBHumanPos[LHipRoll] = lHipRollPositionActuator;
  // luaToBHumanPos[LHipPitch] = lHipPitchPositionActuator;
  // luaToBHumanPos[LKneePitch] = lKneePitchPositionActuator;
  // luaToBHumanPos[LAnklePitch] = lAnklePitchPositionActuator;
  // luaToBHumanPos[LAnkleRoll] = lAnkleRollPositionActuator;
  // luaToBHumanPos[RHipYawPitch] = lHipYawPitchPositionActuator;
  // luaToBHumanPos[RHipRoll] = rHipRollPositionActuator;
  // luaToBHumanPos[RHipPitch] = rHipPitchPositionActuator;
  // luaToBHumanPos[RKneePitch] = rKneePitchPositionActuator;
  // luaToBHumanPos[RAnklePitch] = rAnklePitchPositionActuator;
  // luaToBHumanPos[RAnkleRoll] = rAnkleRollPositionActuator;
  // luaToBHumanPos[RShoulderPitch] = rShoulderPitchPositionActuator;
  // luaToBHumanPos[RShoulderRoll] = rShoulderRollPositionActuator;
  // luaToBHumanPos[RElbowYaw] = rElbowYawPositionActuator;
  // luaToBHumanPos[RElbowRoll] = rElbowRollPositionActuator;
}

// static void lua_pushdouble_array(lua_State *L, double *v, int n) {
//   lua_createtable(L, n, 0);
//   for (int i = 0; i < n; i++) {
//     lua_pushnumber(L, v[i]);
//     lua_rawseti(L, -2, i+1);
//   }
// }

// static void lua_pushvector(lua_State *L, std::vector<double> &v) {
//   int n = v.size();
//   lua_createtable(L, n, 0);
//   for (int i = 0; i < n; i++) {
//     lua_pushnumber(L, v[i]);
//     lua_rawseti(L, -2, i+1);
//   }
// }

// static std::vector<double> lua_checkvector(lua_State *L, int narg) {
//   if (!lua_istable(L, narg))
//     luaL_typerror(L, narg, "vector");
//   int n = lua_objlen(L, narg);
//   std::vector<double> v(n);
//   for (int i = 0; i < n; i++) {
//     lua_rawgeti(L, narg, i+1);
//     v[i] = lua_tonumber(L, -1);
//     lua_pop(L, 1);
//   }
//   return v;
// }

// static void openActuators(float*& actuators)
// {
//   assert(writingActuators == -1);
//   writingActuators = 0;
//   if(writingActuators == data->newestActuators)
//     ++writingActuators;
//   if(writingActuators == data->readingActuators)
//     if(++writingActuators == data->newestActuators)
//       ++writingActuators;
//   assert(writingActuators != data->newestActuators);
//   assert(writingActuators != data->readingActuators);
//   actuators = data->actuators[writingActuators];
// }

// static void closeActuators()
// {
//   assert(writingActuators >= 0);
//   data->newestActuators = writingActuators;
//   writingActuators = -1;
// }

static void set_actuator_positions(std::vector<double> vs, std::vector<double> ids) {
  if (!initialized) {
    lua_initialize();
  }

  //////////// control with lua wrapper, no buffer
  // float * actuators;
  // openActuators(actuators);

  // for (unsigned int i = 0; i < vs.size(); i++) {
  //   actuators[(int)ids[i]-1] = vs[i];
  //   // std::cout<<" "<< vs[i];
  // }
  // // std::cout<<std::endl;
  // closeActuators();
  //////////

  //////// control with luawrapper, buffer
  for (unsigned int i = 0; i < vs.size(); i++) {
    int bHumanIndex = luaToBHumanPos[(int) ids[i] - 1];
    data->luaBuffer[bHumanIndex] = vs[i];
  }

  data->luaNewSet = true;
  ///////// 

  // std::cout<<data->luaNewSet<<std::endl;
}

static void set_actuator_hardnesses(std::vector<double> vs, std::vector<double> ids) {
  if (!initialized) {
    lua_initialize();
  }

  for (unsigned int i = 0; i < vs.size(); i++) {
    int bHumanIndex = luaToBHumanPos[(int) ids[i] - 1];
    data->luaBuffer[bHumanIndex + lbhNumOfPositionActuatorIds] = vs[i];
  }

  data->luaNewSet = true;

  // std::vector<double> hardness_values = lua_checkvector(L, 1);
  // int startIndex = luaL_checkint(L, 2) - 1;
  
  // for (int i = startIndex; i < startIndex + hardness_values.size(); i++) {
  //   int bHumanIndex = luaToBHumanPos[i];
  //   data->luaBuffer[bHumanIndex + lbhNumOfPositionActuatorIds] = hardness_values[i];
  // }

  // data->luaNewSet = true;

}

static void set_actuator_position(double x, int ind) {
  if (!initialized) {
    lua_initialize();
  }

  int index = index - 1; // convert to zero-indexed

  int bHumanIndex = luaToBHumanPos[index];
  data->luaBuffer[bHumanIndex] = x;

  data->luaNewSet = true;
}

static void set_actuator_hardness(double x, int ind) {
  if (!initialized) {
    lua_initialize();
  }
  
  int index = ind - 1; // convert to zero-indexed
  
  int bHumanIndex = luaToBHumanPos[index];
  data->luaBuffer[bHumanIndex + lbhNumOfPositionActuatorIds] = x;

  data->luaNewSet = true;
}

/**
 ** Returns actuator command for single position
 **/
static void get_actuator_position(float& value, int ind) {
  if (!initialized) {
    lua_initialize();
  }

  int index = ind - 1; // convert to zero-indexed
  data->readingActuators = data->newestActuators;
  float * actuators = data->actuators[data->readingActuators];

  int bHumanIndex = luaToBHumanPos[index];
  value = actuators[bHumanIndex];
}

/**
 ** Returns actuator command for single hardsness
 **/
static void get_actuator_hardness(float& value, int ind) {
  if (!initialized) {
    lua_initialize();
  }
  int index = ind - 1; // convert to zero-indexed
  data->readingActuators = data->newestActuators;
  float * actuators = data->actuators[data->readingActuators];

  int bHumanIndex = luaToBHumanPos[index];
  value = actuators[bHumanIndex + lbhNumOfPositionActuatorIds];
}

/**
 ** Sets entire list of actuator values starting with the initial index
 ** @param list of actuator positions, initial index
 **/
static void set_actuator_command(std::vector<double> joint_values, int startInd) {
  if (!initialized) {
    lua_initialize();
  }

  int startIndex = startInd - 1;
  
  for (int i = startIndex; i < startIndex + joint_values.size(); i++) {
    // std::cout << "i: " << i << "   joint_values[i]: " << joint_values[i-startIndex] << std::endl;
    int bHumanIndex = luaToBHumanPos[i];
    data->luaBuffer[bHumanIndex] = joint_values[i-startIndex];
  }

  data->luaNewSet = true;
}

/**
 ** Returns entire list of actuator values starting with the initial index
 ** @param starting index of head, l/r arm, or l/r leg
 ** @return list of actuator positions
 **/
static void get_actuator_command(float* in_out_buffer, float& size, int startInd) {
  if (!initialized) {
    lua_initialize();
  }

  int startIndex = startInd - 1;
  int numActuators;

  switch (startIndex) {
    case indexHead: numActuators = nJointHead; break;
    case indexLArm: numActuators = nJointLArm; break;
    case indexLLeg: numActuators = nJointLLeg; break;
    case indexRLeg: numActuators = nJointRLeg; break;
    case indexRArm: numActuators = nJointRArm; break;
  }

  data->readingActuators = data->newestActuators;
  float * actuators = data->actuators[data->newestActuators];
  float actuatorCommands[numActuators];
  for (int i = 0; i < numActuators; i++) {
    int bHumanIndex = luaToBHumanPos[i];
    actuatorCommands[i] = (double) actuators[bHumanIndex];
  }

  size = numActuators;
  memcpy(in_out_buffer, actuatorCommands, sizeof(actuatorCommands));
}

static void get_sensor_position(float& value, int ind) {
  if (!initialized) {
    lua_initialize();
  }

  int index = ind - 1; // convert to zero-indexed

  data->readingSensors = data->newestSensors;
  float * sensors = data->sensors[data->newestSensors];
  int bHumanIndex = luaToBHumanPos[index];

  value = sensors[3*bHumanIndex];

}

/**
 ** Returns all joint positions read by sensors
 **/
static void get_sensor_positions(float* in_out_buffer, float& size) {  
  if (!initialized) {
    lua_initialize();
  }
  data->readingSensors = data->newestSensors;
  float * sensors = data->sensors[data->newestSensors];
  float actuatorSensors[nJoint];
  for (int i = 0; i < nJoint; i++) {
    int bHumanIndex = luaToBHumanPos[i];
    actuatorSensors[i] = (double) sensors[3*bHumanIndex];
  }

  size = nJoint;
  memcpy(in_out_buffer, actuatorSensors, sizeof(actuatorSensors));
}

/**
 ** Returns all set joint hardness values
 **/
static void get_actuator_hardnesses(float* in_out_buffer, float& size) {
  if (!initialized) {
    lua_initialize();
  }
  data->readingActuators = data->newestActuators;
  float * actuators = data->actuators[data->readingActuators];
  double actuatorHardnesses[nJoint];
  for (int i = 0; i < nJoint; i++) {
    int bHumanIndex = luaToBHumanPos[i];
    actuatorHardnesses[i] = (double) actuators[bHumanIndex +lbhNumOfPositionActuatorIds];
  }
  size = nJoint;
  memcpy(in_out_buffer, actuatorHardnesses, sizeof(actuatorHardnesses));
}

/**
 ** Returns IMU angle readings 
 **/
static void get_imu_angle(float* in_out_buffer, float& size) {
  if (!initialized) {
    lua_initialize();
  }
  data->readingSensors = data->newestSensors;
  float * sensors = data->sensors[data->readingSensors];
  double ImuReadings[3] = {(double) sensors[angleXSensor], (double) sensors[angleYSensor], sensors[angleZSensor]};
  size = nJoint;
  memcpy(in_out_buffer, ImuReadings, sizeof(ImuReadings));
}

/**
 ** Returns IMU acc readings 
 **/
static void get_imu_acc(float* in_out_buffer, float& size) {
  if (!initialized) {
    lua_initialize();
  }
  data->readingSensors = data->newestSensors;
  float * sensors = data->sensors[data->readingSensors];
  double AccReadings[3] = {(double) sensors[accXSensor], (double) sensors[accYSensor], (double) sensors[accZSensor]};
  size = nJoint;
  memcpy(in_out_buffer, AccReadings, sizeof(AccReadings));
}

/**
 ** Returns IMU gyro readings 
 **/
static void get_imu_gyr(float* in_out_buffer, float& size) {
  if (!initialized) {
    lua_initialize();
  }
  data->readingSensors = data->newestSensors;
  float * sensors = data->sensors[data->readingSensors];
  double GyroReadings[3] = {(double) sensors[gyroXSensor], (double) sensors[gyroYSensor], (double) -sensors[gyroZSensor]};
  size = nJoint;
  memcpy(in_out_buffer, GyroReadings, sizeof(GyroReadings));
}

/**
 ** Returns whether the data->luaNewSet flag is on
 **/
static void get_flag(bool& flag) {
  flag = data->luaNewSet;
}

/**
 ** Use CPP to set motion
 **/
static void set_actuator_position_forever(std::vector<double> ids, std::vector<double> vs) {
  if (!initialized) {
    lua_initialize();
  }

  std::cout<<"i'm starting"<<std::endl;
  int cnt = 0.1;
  data->luaBuffer[headYawStiffnessActuator] = 0.5;
  while (1) {
    if (data->luaNewSet == false) {
      
      vs[0] = cos(speed*cnt);
      // std::vector<float> vs;
      // vs.push_back(cos(0.01*cnt));
      // std::vector<int> ids;
      // ids.push_back(1);
      //////// control with luawrapper, buffer
      for (unsigned int i = 0; i < vs.size(); i++) {
        data->luaBuffer[(int)ids[i]-1] = vs[i];
      }

      data->luaNewSet = true;
      std::cout<<"i'm running"<<std::endl;
      cnt++;
      
      
    }
    data->readingSensors = data->newestSensors;
    float * sensors = data->sensors[data->readingSensors];
    std::cout<<sensors[1] * 1000<< std::endl;
    std::cout<<data->luaNewSet<<std::endl;
  }
}

static void get_sensor_current(float* in_out_buffer, float& size) {
  if (!initialized) {
    lua_initialize();
  }
  // int index = luaL_checkint(L, 1) - 1; // convert to zero-indexed

  // int bHumanIndex = luaToBHumanPos[index];
  // data->readingSensors = data->newestSensors;
  // float * sensors = data->sensors[data->readingSensors];
  // lua_pushnumber(L, (double) 1000 * sensors[3*bHumanIndex+1]);
  // return 1;

  data->readingSensors = data->newestSensors;
  float * sensors = data->sensors[data->readingSensors];

  double sensorCurrents[nJoint];
  for (int i = 0; i < nJoint; i++) {
    int bHumanIndex = luaToBHumanPos[i];
    sensorCurrents[i] = (double) sensors[3 * bHumanIndex + 1];
  }
  size = nJoint;
  memcpy(in_out_buffer, sensorCurrents, sizeof(sensorCurrents));
}

static void set_actuator_velocity() {
  if (!initialized) {
    lua_initialize();
  }
}

static void get_actuator_velocity(float& value) {
  if (!initialized) {
    lua_initialize();
  }
  value = 0;
}

static void get_time(float& time) {
  if (!initialized) {
    lua_initialize();
  }
  // //while (!data->newTime) {} // wait for new time
  // lua_pushnumber(L, data->dcmTime);
  // // data->newTime = false;

  struct timeval tv;
  gettimeofday(&tv, NULL);
  double t = tv.tv_sec + 1E-6*tv.tv_usec;
  time = (float) t;
}

static void get_sensor_batteryCharge(float& value) {
  if (!initialized) {
    lua_initialize();
  }

  data->readingSensors = data->newestSensors;
  float * sensors = data->sensors[data->readingSensors];
  value = sensors[batteryChargeSensor];
}

static void get_sensor_button(float& value) {
  if (!initialized) {
    lua_initialize();
  }

  data->readingSensors = data->newestSensors;
  float * sensors = data->sensors[data->readingSensors];
  value = sensors[chestButtonSensor];
}

static void get_sensor_bumperLeft(float* in_out_buffer, float& size) {
  if (!initialized) {
    lua_initialize();
  }

  data->readingSensors = data->newestSensors;
  float * sensors = data->sensors[data->readingSensors];
  float bumperReadings[2] = {sensors[lBumperLeftSensor], sensors[lBumperRightSensor]};
  memcpy(in_out_buffer, bumperReadings, sizeof(bumperReadings));
}

static void get_sensor_bumperRight(float* in_out_buffer, float& size) {
  if (!initialized) {
    lua_initialize();
  }

  data->readingSensors = data->newestSensors;
  float * sensors = data->sensors[data->readingSensors];
  double bumperReadings[2] = {(double) sensors[rBumperLeftSensor], (double) sensors[rBumperRightSensor]};
  memcpy(in_out_buffer, bumperReadings, sizeof(bumperReadings));
}

static void get_sensor_sonarLeft(float* in_out_buffer, float& size) {
  if (!initialized) {
    lua_initialize();
  }

  data->readingSensors = data->newestSensors;
  float * sensors = data->sensors[data->readingSensors];

  double sonarSensors[10];
  for (int i=0; i<10; i++) {
    //std::cout << "lua_dcmII.cpp index: " << i+lUsSensor << std::endl;
    sonarSensors[i]=sensors[i+lUsSensor];
  }
  memcpy(in_out_buffer, sonarSensors, sizeof(sonarSensors));
}

static void get_sensor_sonarRight(float* in_out_buffer, float& size) {
  if (!initialized) {
    lua_initialize();
  }

  data->readingSensors = data->newestSensors;
  float * sensors = data->sensors[data->readingSensors];

  double sonarSensors[10];
  for (int i=0; i<10; i++) {
    sonarSensors[i]=sensors[i+rUsSensor];
  }
  memcpy(in_out_buffer, sonarSensors, sizeof(sonarSensors));
}

static void get_sensor_temperature(float* in_out_buffer, float& size) {
  if (!initialized) {
    lua_initialize();
  }

  data->readingSensors = data->newestSensors;
  float * sensors = data->sensors[data->readingSensors];

  double sensorTemperatures[nJoint];
  for (int i = 0; i < nJoint; i++) {
    int bHumanIndex = luaToBHumanPos[i];
    sensorTemperatures[i] = (double) sensors[3 * bHumanIndex + 2];
  }
  memcpy(in_out_buffer, sensorTemperatures, sizeof(sensorTemperatures));
}

static void set_actuator_ultraSonic(int command) {
  if (!initialized) {
    lua_initialize();
  }

  data->luaBuffer[usActuator] = command;
  data->luaNewSet = true;
}

static void get_sensor_fsrLeft(float* in_out_buffer, float& size) {
  if (!initialized) {
    lua_initialize();
  }

  data->readingSensors = data->newestSensors;
  float * sensors = data->sensors[data->readingSensors];
  double fsrSensors[4] = {(double) sensors[lFSRFrontLeftSensor], (double) sensors[lFSRRearLeftSensor], (double) sensors[lFSRFrontRightSensor], (double) sensors[lFSRRearRightSensor]};
  
  memcpy(in_out_buffer, fsrSensors, sizeof(fsrSensors));
}

static void get_sensor_fsrRight(float* in_out_buffer, float& size) {
  if (!initialized) {
    lua_initialize();
  }

  data->readingSensors = data->newestSensors;
  float * sensors = data->sensors[data->readingSensors];
  double fsrSensors[4] = {(double) sensors[rFSRFrontLeftSensor], (double) sensors[rFSRRearLeftSensor], (double) sensors[rFSRFrontRightSensor], (double) sensors[rFSRRearRightSensor]};
  
  memcpy(in_out_buffer, fsrSensors, sizeof(fsrSensors));
}

// static int get_sensor_ultraSonic(lua_State *L) {
//   if (!initialized) {
//     lua_initialize();
//   }

//   data->readingSensors = data->newestSensors;
//   float * sensors = data->sensors[data->readingSensors];
  
//   cout << "sensors[UsSensor]: " << sensors[UsSensor] << endl;
//   lua_pushnumber(L, sensors[UsSensor]);
  
//   return 1;
// }

static void set_actuator_ledFootLeft(std::vector<double> values) {
  if (!initialized) {
    lua_initialize();
  }
   
  for (unsigned int i = 0; i < 3; i++) {
    std::cout<<"l " << i << "   "<< values[i] << std::endl;
    data->luaBuffer[i + lFootLedRedActuator] = values[i];
  }

  data->luaNewSet = true;
}

static void set_actuator_ledFootRight(std::vector<double> values) {
  if (!initialized) {
    lua_initialize();
  }

  for (unsigned int i = 0; i < 3; i++) {
    // std::cout<<"l " << i << "   "<< values[i] << std::endl;
    data->luaBuffer[i + rFootLedRedActuator] = values[i];
  }

  data->luaNewSet = true;
}

static void set_actuator_ledEarsLeft(std::vector<double> values) {
  if (!initialized) {
    lua_initialize();
  }

  for (unsigned int i = 0; i < 10; i++) {
    // std::cout<<"l " << i << "   "<< values[i] << std::endl;
    data->luaBuffer[i + earsLedLeft0DegActuator] = values[i];
  }

  data->luaNewSet = true;
}

static void set_actuator_ledEarsRight(std::vector<double> values) {
  if (!initialized) {
    lua_initialize();
  }

  for (unsigned int i = 0; i < 10; i++) {
    // std::cout<<"l " << i << "   "<< values[i] << std::endl;
    data->luaBuffer[i + earsLedRight0DegActuator] = values[i];
  }

  data->luaNewSet = true;
}

static void set_actuator_ledFaceLeft(std::vector<double> values, int ind) {
  if (!initialized) {
    lua_initialize();
  }
  int index = ind - 1;

  for (unsigned int i = 0; i < values.size(); i++) {
    // std::cout<<"l " << i << "   "<< values[i] << std::endl;
    data->luaBuffer[i + faceLedRedLeft0DegActuator + index] = values[i];
  }

  data->luaNewSet = true;
}

static void set_actuator_ledFaceRight(std::vector<double> values, int ind) {
  if (!initialized) {
    lua_initialize();
  }
  int index = ind - 1;

  for (unsigned int i = 0; i < values.size(); i++) {
    data->luaBuffer[i + faceLedRedRight0DegActuator + index] = values[i];
  }

  data->luaNewSet = true;
}

static void set_actuator_ledChest(std::vector<double> values) {
  if (!initialized) {
    lua_initialize();
  }

  for (unsigned int i = 0; i < 3; i++) {
    // std::cout<<"l " << i << "   "<< values[i] << std::endl;
    data->luaBuffer[i + chestBoardLedRedActuator] = values[i];
  }

  data->luaNewSet = true;
}

static void set_actuator_ledHead(std::vector<double> values) {
  if (!initialized) {
    lua_initialize();
  }

  for (unsigned int i = 0; i < values.size(); i++) {
    // std::cout<<"l " << i << "   "<< values[i] << std::endl;
    data->luaBuffer[i + headLedRearLeft0Actuator] = values[i];
  }

  data->luaNewSet = true;
}


// static const struct luaL_Reg bhlowcmd_lib [] = {
//   // {"getdummy", luaBH_getdummy},

//   {"set_actuator_hardnesses", set_actuator_hardnesses},
//   {"set_actuator_positions", set_actuator_positions},
//   {"set_actuator_position", set_actuator_position},
//   {"set_actuator_hardness", set_actuator_hardness},  

//   {"get_actuator_position", get_actuator_position},
//   {"get_actuator_hardness", get_actuator_hardness},

//   {"set_actuator_command", set_actuator_command},
//   {"get_actuator_command", get_actuator_command},

//   {"get_sensor_position", get_sensor_position},
//   {"get_sensor_positions", get_sensor_positions},
//   {"get_actuator_hardnesses", get_actuator_hardnesses},

//   {"get_imu_angle", get_imu_angle},
//   {"get_imu_acc", get_imu_acc},
//   {"get_imu_gyr", get_imu_gyr},

//   {"get_flag", get_flag},
//   {"set_actuator_position_forever", set_actuator_position_forever},
//   {"get_sensor_current", get_sensor_current},
//   {"get_time", get_time},

//   // in original file but not implemented in this one
//   // {"set_actuator_mode", set_actuator_mode},
//   {"set_actuator_velocity", set_actuator_velocity},
//   {"get_actuator_velocity", get_actuator_velocity},
//   // {"set_actuator_joint_imu_angle_x", set_actuator_joint_imu_angle_x},
//   // {"set_actuator_joint_imu_angle_y", set_actuator_joint_imu_angle_y},

//   {"get_sensor_batteryCharge", get_sensor_batteryCharge},
//   {"get_sensor_button", get_sensor_button},
//   {"get_sensor_bumperLeft", get_sensor_bumperLeft},
//   {"get_sensor_bumperRight", get_sensor_bumperRight},
//   {"get_sensor_sonarLeft", get_sensor_sonarLeft},
//   {"get_sensor_sonarRight", get_sensor_sonarRight},
//   {"get_sensor_temperature", get_sensor_temperature},

//   {"set_actuator_ultraSonic", set_actuator_ultraSonic},

//   {"get_sensor_fsrLeft", get_sensor_fsrLeft},
//   {"get_sensor_fsrRight", get_sensor_fsrRight},
//   // {"get_sensor_ultraSonic", get_sensor_ultraSonic},

//   {"set_actuator_ledFootLeft", set_actuator_ledFootLeft},
//   {"set_actuator_ledFootRight", set_actuator_ledFootRight},
//   {"set_actuator_ledEarsLeft", set_actuator_ledEarsLeft},
//   {"set_actuator_ledEarsRight", set_actuator_ledEarsRight},
//   {"set_actuator_ledFaceLeft", set_actuator_ledFaceLeft},
//   {"set_actuator_ledFaceRight", set_actuator_ledFaceRight},
//   {"set_actuator_ledChest", set_actuator_ledChest},
//   {"set_actuator_ledHead", set_actuator_ledHead},
  
//   {NULL, NULL}
// };


// extern "C" int luaopen_bhlowcmd(lua_State *L) {
// #if LUA_VERSION_NUM == 502
//   luaL_newlib(L, bhlowcmd_lib);
// #else
//   luaL_register(L, "bhlowcmd", bhlowcmd_lib);
// #endif
//   return 1;
// }
