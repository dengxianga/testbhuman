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

static void openActuators(float*& actuators)
{
  assert(writingActuators == -1);
  writingActuators = 0;
  if(writingActuators == data->newestActuators)
    ++writingActuators;
  if(writingActuators == data->readingActuators)
    if(++writingActuators == data->newestActuators)
      ++writingActuators;
  assert(writingActuators != data->newestActuators);
  assert(writingActuators != data->readingActuators);
  actuators = data->actuators[writingActuators];
}

static void closeActuators()
{
  assert(writingActuators >= 0);
  data->newestActuators = writingActuators;
  writingActuators = -1;
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
    data->luaBuffer[(int)ids[i]-1] = vs[i];
  }

  data->luaNewSet = true;
  ///////// 

  std::cout<<data->luaNewSet<<std::endl;

  return 0;
}

// static int set_speed(lua_State *L) {
//   if (!initialized) {
//     lua_initialize();
//   }

//   double new_speed = lua_tonumber(L, 1);
//   speed = new_speed;
// }

static int set_actuator_position_forever(lua_State *L) {
  if (!initialized) {
    lua_initialize();
  }

  std::cout<<"i'm starting"<<std::endl;
  int cnt = 0.1;
  data->luaBuffer[headYawStiffnessActuator] = 0.5;
  while (1) {
    if (data->luaNewSet == false) {
      std::vector<double> vs = lua_checkvector(L, 1);
      std::vector<double> ids = lua_checkvector(L, 2);
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

      float * sensors = data->sensors[data->newestSensors];
      std::cout<<sensors[2*ids[1]]<< std::endl;
      
    }
    std::cout<<data->luaNewSet<<std::endl;

  }
}

static int set_actuator_stiffneses(lua_State *L) {
  if (!initialized) {
    lua_initialize();
  }
  std::vector<double> vs = lua_checkvector(L, 1);
  std::vector<double> ids = lua_checkvector(L, 2);

  for (unsigned int i = 0; i < vs.size(); i++) {
    data->luaBuffer[(int) ids[i] -1 + lbhNumOfPositionActuatorIds] = vs[i];
  }

  data->luaNewSet = true;

  return 0;
}

static int get_flag(lua_State *L) {
  lua_pushboolean(L, data->luaNewSet);
  return 0;
}

static int get_actuator_position(lua_State *L) {
  if (!initialized) {
    lua_initialize();
  }
  int index = luaL_checkint(L, 1) - 1; // convert to zero-indexed
  float * actuators = data->actuators[data->newestActuators];
  lua_pushnumber(L, (double) actuators[index]);
  return 1;
}

static int get_actuator_hardness(lua_State *L) {
  if (!initialized) {
    lua_initialize();
  }
  int index = luaL_checkint(L, 1) -1 + 25; // convert to zero-indexed, add 26
  float * actuators = data->actuators[data->newestActuators];
  lua_pushnumber(L, (double) actuators[index]);
  return 1;
}

static int get_actuator_command(lua_State *L) {
  get_actuator_position(L);
}


static int get_sensor_position(lua_State *L) {
  if (!initialized) {
    lua_initialize();
  }
  int index = luaL_checkint(L, 1) -1; // convert to zero-indexed, add 26
  float * sensors = data->sensors[data->newestSensors];
  lua_pushnumber(L, (double) sensors[3*index]);
  return 1;
}

static int get_sensor_current(lua_State *L) {
  if (!initialized) {
    lua_initialize();
  }
  int index = luaL_checkint(L, 1) - 1; // convert to zero-indexed, add 26
  float * sensors = data->sensors[data->newestSensors];
  lua_pushnumber(L, (double) sensors[2*index]);
  return 1;
}


static const struct luaL_Reg bhlowcmd_lib [] = {
  {"getdummy", luaBH_getdummy},
  {"set_actuator_stiffneses", set_actuator_stiffneses},
  {"set_actuator_positions", set_actuator_positions},
  {"get_actuator_position", get_actuator_position},
  {"get_actuator_hardness", get_actuator_hardness},
  {"get_actuator_command", get_actuator_command},
  {"get_sensor_position", get_sensor_position},
  {"get_flag", get_flag},
  {"set_actuator_position_forever", set_actuator_position_forever},
  {"get_sensor_current", get_sensor_current},
  // {"set_speed", set_speed},
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
