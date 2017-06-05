#include <sys/types.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <semaphore.h>
#include <csignal>
#include <sys/resource.h>
#include <ctime>
#include <cstring>

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

static int luaBH_getdummy (lua_State *L) {
  lua_pushnumber(L, 0);
  // TODO try to manipulate the shared memory "data"
  fd = shm_open(LBH_MEM_NAME, O_RDWR, S_IRUSR | S_IWUSR);
  data = (LBHData*)mmap(NULL, sizeof(LBHData), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);

  for (int i=0;i<10000;i++){
    float * actuators = data->actuators[data->newestActuators];
    std::cout<<"shm val actuators[0] " << actuators[0]<< " \n";
  }

  return 1;
}

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

static void lua_pushdouble_array(lua_State *L, double *v, int n) {
  lua_createtable(L, n, 0);
  for (int i = 0; i < n; i++) {
    lua_pushnumber(L, v[i]);
    lua_rawseti(L, -2, i+1);
  }
}

static void lua_pushvector(lua_State *L, std::vector<double> &v) {
  int n = v.size();
  lua_createtable(L, n, 0);
  for (int i = 0; i < n; i++) {
    lua_pushnumber(L, v[i]);
    lua_rawseti(L, -2, i+1);
  }
}

static std::vector<double> lua_checkvector(lua_State *L, int narg) {
  if (!lua_istable(L, narg))
    luaL_typerror(L, narg, "vector");
  int n = lua_objlen(L, narg);
  std::vector<double> v(n);
  for (int i = 0; i < n; i++) {
    lua_rawgeti(L, narg, i+1);
    v[i] = lua_tonumber(L, -1);
    lua_pop(L, 1);
  }
  return v;
}

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

static int set_actuator_positions(lua_State *L) {
  if (!initialized) {
    lua_initialize();
  }
  std::vector<double> vs = lua_checkvector(L, 1);
  std::vector<double> ids = lua_checkvector(L, 2);

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

  return 0;
}

static int set_actuator_hardnesses(lua_State *L) {
  if (!initialized) {
    lua_initialize();
  }
  // std::vector<double> vs = lua_checkvector(L, 1);
  // std::vector<double> ids = lua_checkvector(L, 2);

  // for (unsigned int i = 0; i < vs.size(); i++) {
  //   int bHumanIndex = luaToBHumanPos[(int) ids[i] - 1];
  //   data->luaBuffer[bHumanIndex + lbhNumOfPositionActuatorIds] = vs[i];
  // }

  // data->luaNewSet = true;

  std::vector<double> hardness_values = lua_checkvector(L, 1);
  int startIndex = luaL_checkint(L, 2) - 1;
  
  for (int i = startIndex; i < startIndex + hardness_values.size(); i++) {
    int bHumanIndex = luaToBHumanPos[i];
    data->luaBuffer[bHumanIndex + lbhNumOfPositionActuatorIds] = hardness_values[i];
  }

  data->luaNewSet = true;

  return 0;
}

static int set_actuator_position(lua_State *L) {
  if (!initialized) {
    lua_initialize();
  }

  double x = luaL_checknumber(L, 1);
  int index = luaL_checkint(L, 2) - 1; // convert to zero-indexed

  int bHumanIndex = luaToBHumanPos[index];
  data->luaBuffer[index] = x;

  data->luaNewSet = true;

  return 0;
}

static int set_actuator_hardness(lua_State *L) {
  if (!initialized) {
    lua_initialize();
  }
  
  double x = luaL_checknumber(L, 1);
  int index = luaL_checkint(L, 2) - 1; // convert to zero-indexed
  
  int bHumanIndex = luaToBHumanPos[index];
  data->luaBuffer[bHumanIndex + lbhNumOfPositionActuatorIds] = x;

  data->luaNewSet = true;

  return 0;
}

/**
 ** Returns actuator command for position
 **/
static int get_actuator_position(lua_State *L) {
  if (!initialized) {
    lua_initialize();
  }
  int index = luaL_checkint(L, 1) - 1; // convert to zero-indexed
  data->readingActuators = data->newestActuators;
  float * actuators = data->actuators[data->readingActuators];

  int bHumanIndex = luaToBHumanPos[index];
  lua_pushnumber(L, (double) actuators[bHumanIndex]);
  return 1;
}

/**
 ** Returns actuator command for hardsness
 **/
static int get_actuator_hardness(lua_State *L) {
  if (!initialized) {
    lua_initialize();
  }
  int index = luaL_checkint(L, 1) - 1; // convert to zero-indexed
  data->readingActuators = data->newestActuators;
  float * actuators = data->actuators[data->readingActuators];

  int bHumanIndex = luaToBHumanPos[index];
  lua_pushnumber(L, (double) actuators[bHumanIndex + lbhNumOfPositionActuatorIds]);
  return 1;
}

/**
 ** Sets entire list of actuator values starting with the initial index
 ** @param list of actuator positions, initial index
 **/
static int set_actuator_command(lua_State *L) {
  if (!initialized) {
    lua_initialize();
  }

  std::vector<double> joint_values = lua_checkvector(L, 1);
  int startIndex = luaL_checkint(L, 2) - 1;
  
  for (int i = startIndex; i < startIndex + joint_values.size(); i++) {
    int bHumanIndex = luaToBHumanPos[i];
    data->luaBuffer[bHumanIndex] = joint_values[i];
  }

  data->luaNewSet = true;
  return 0;
}

/**
 ** Returns entire list of actuator values starting with the initial index
 ** @param starting index of head, l/r arm, or l/r leg
 ** @return list of actuator positions
 **/
static int get_actuator_command(lua_State *L) {
  if (!initialized) {
    lua_initialize();
  }

  int startIndex = luaL_checkint(L, 1) - 1;
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
  double actuatorCommands[nJoint];
  for (int i = 0; i < numActuators; i++) {
    int bHumanIndex = luaToBHumanPos[i];
    actuatorCommands[i] = (double) actuators[bHumanIndex];
  }
  lua_pushdouble_array(L, actuatorCommands, numActuators);
  return 1;
}

static int get_sensor_position(lua_State *L) {
  if (!initialized) {
    lua_initialize();
  }

  int index = luaL_checkint(L, 1) - 1; // convert to zero-indexed

  data->readingSensors = data->newestSensors;
  float * sensors = data->sensors[data->newestSensors];
  int bHumanIndex = luaToBHumanPos[index];
  lua_pushnumber(L, (double) sensors[3*bHumanIndex]);

  return 1;
}

/**
 ** Returns all joint positions read by sensors
 **/
static int get_sensor_positions(lua_State *L) {  
  if (!initialized) {
    lua_initialize();
  }
  data->readingSensors = data->newestSensors;
  float * sensors = data->sensors[data->newestSensors];
  double actuatorSensors[nJoint];
  for (int i = 0; i < nJoint; i++) {
    int bHumanIndex = luaToBHumanPos[i];
    actuatorSensors[i] = (double) sensors[3*bHumanIndex];
  }
  lua_pushdouble_array(L, (double*) actuatorSensors, (int) nJoint);
  
  std::cout<< "get_sensor_positions is being called from bhlowcmd"<< std::endl;

  return 1;
}

/**
 ** Returns all set joint hardness values
 **/
static int get_actuator_hardnesses(lua_State *L) {
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
  lua_pushdouble_array(L, (double*) actuatorHardnesses, (int) nJoint);


  return 1;
}

/**
 ** Returns IMU angle readings 
 **/
static int get_imu_angle(lua_State *L) {
  if (!initialized) {
    lua_initialize();
  }
  data->readingSensors = data->newestSensors;
  float * sensors = data->sensors[data->readingSensors];
  double ImuReadings[2] = {(double) sensors[angleXSensor], (double) sensors[angleYSensor]};
  lua_pushdouble_array(L, ImuReadings, 2);
  return 1;
}

/**
 ** Returns IMU acc readings 
 **/
static int get_imu_acc(lua_State *L) {
  if (!initialized) {
    lua_initialize();
  }
  data->readingSensors = data->newestSensors;
  float * sensors = data->sensors[data->readingSensors];
  double AccReadings[3] = {(double) sensors[accXSensor], (double) sensors[accYSensor], (double) sensors[accZSensor]};
  lua_pushdouble_array(L, AccReadings, 3);
  return 1;
}

/**
 ** Returns IMU gyro readings 
 **/
static int get_imu_gyr(lua_State *L) {
  if (!initialized) {
    lua_initialize();
  }
  data->readingSensors = data->newestSensors;
  float * sensors = data->sensors[data->readingSensors];
  double GyroReadings[3] = {(double) sensors[gyroXSensor], (double) sensors[gyroYSensor], (double) sensors[gyroZSensor]};
  lua_pushdouble_array(L, GyroReadings, 3);
  return 1;
}

/**
 ** Returns whether the data->luaNewSet flag is on
 **/
static int get_flag(lua_State *L) {
  lua_pushboolean(L, data->luaNewSet);
  return 0;
}

/**
 ** Use CPP to set motion
 **/
static int set_actuator_position_forever(lua_State *L) {
  if (!initialized) {
    lua_initialize();
  }

  std::cout<<"i'm starting"<<std::endl;
  int cnt = 0.1;
  data->luaBuffer[headYawStiffnessActuator] = 0.5;
  while (1) {
    std::vector<double> ids = lua_checkvector(L, 2);
    if (data->luaNewSet == false) {
      std::vector<double> vs = lua_checkvector(L, 1);
      
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
    float * sensors = data->sensors[data->newestSensors];
    std::cout<<sensors[1] * 1000<< std::endl;
    std::cout<<data->luaNewSet<<std::endl;
  }
}

static int get_sensor_current(lua_State *L) {
  if (!initialized) {
    lua_initialize();
  }
  int index = luaL_checkint(L, 1) - 1; // convert to zero-indexed

  int bHumanIndex = luaToBHumanPos[index];
  float * sensors = data->sensors[data->newestSensors];
  lua_pushnumber(L, (double) 1000 * sensors[3*bHumanIndex+1]);
  return 1;
}

static int set_actuator_velocity(lua_State *L) {
  if (!initialized) {
    lua_initialize();
  }
  double x = luaL_checknumber(L, 1);
  int index = luaL_checkint(L, 2) - 1; // convert to zero-indexed
  return 0;
}

static int get_actuator_velocity(lua_State *L) {
  if (!initialized) {
    lua_initialize();
  }
  int index = luaL_checkint(L, 1) - 1; // convert to zero-indexed
  lua_pushnumber(L, 0);
  return 0;
}

static int get_time(lua_State *L) {
  if (!initialized) {
    lua_initialize();
  }
  while (!data->newTime) {} // wait for new time
  lua_pushnumber(L, data->dcmTime);
  data->newTime = false;
  return 1;
}

static const struct luaL_Reg bhlowcmd_lib [] = {
  {"getdummy", luaBH_getdummy},

  {"set_actuator_hardnesses", set_actuator_hardnesses},
  {"set_actuator_positions", set_actuator_positions},
  {"set_actuator_position", set_actuator_position},
  {"set_actuator_hardness", set_actuator_hardness},  

  {"get_actuator_position", get_actuator_position},
  {"get_actuator_hardness", get_actuator_hardness},

  {"set_actuator_command", set_actuator_command},
  {"get_actuator_command", get_actuator_command},

  {"get_sensor_position", get_sensor_position},
  {"get_sensor_positions", get_sensor_positions},
  {"get_actuator_hardnesses", get_actuator_hardnesses},

  {"get_imu_angle", get_imu_angle},
  {"get_imu_acc", get_imu_acc},
  {"get_imu_gyr", get_imu_gyr},

  {"get_flag", get_flag},
  {"set_actuator_position_forever", set_actuator_position_forever},
  {"get_sensor_current", get_sensor_current},
  {"get_time", get_time},

  // in original file but not implemented in this one
  // {"set_actuator_mode", set_actuator_mode},
  {"set_actuator_velocity", set_actuator_velocity},
  {"get_actuator_velocity", get_actuator_velocity},
  // {"set_actuator_joint_imu_angle_x", set_actuator_joint_imu_angle_x},
  // {"set_actuator_joint_imu_angle_y", set_actuator_joint_imu_angle_y},
  
  {NULL, NULL}
};


extern "C" int luaopen_bhlowcmd(lua_State *L) {
#if LUA_VERSION_NUM == 502
  luaL_newlib(L, bhlowcmd_lib);
#else
  luaL_register(L, "bhlowcmd", bhlowcmd_lib);
#endif
  return 1;
}
