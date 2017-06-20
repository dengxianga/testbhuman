#include <sys/types.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <semaphore.h>
#include <csignal>
#include <sys/resource.h>
#include <ctime>
#include <cstring>
#include <sys/time.h>
#include <cmath>
#include <lua.hpp>
static int luaBH_getdummy (lua_State *L);

static int set_actuator_positions(lua_State *L);
static int set_actuator_hardnesses(lua_State *L);

static int set_actuator_position(lua_State *L);

static int set_actuator_positions_adjust(lua_State *L);

static int set_actuator_command_adjust(lua_State *L);

static int set_actuator_hardness(lua_State *L);
/**
 ** Returns actuator command for position
 **/
static int get_actuator_position(lua_State *L);

/**
 ** Returns actuator command for all positions
 **/
static int get_actuator_positions(lua_State *L);

/**
 ** Returns actuator command for hardsness
 **/
static int get_actuator_hardness(lua_State *L);

/**
 ** Sets entire list of actuator values starting with the initial index
 ** @param list of actuator positions, initial index
 **/
static int set_actuator_command(lua_State *L);

/**
 ** Returns entire list of actuator values starting with the initial index
 ** @param starting index of head, l/r arm, or l/r leg
 ** @return list of actuator positions
 **/
static int get_actuator_command(lua_State *L);

static int get_sensor_position(lua_State *L);

/**
 ** Returns all joint positions read by sensors
 **/
static int get_sensor_positions(lua_State *L);

/**
 ** Returns all set joint hardness values
 **/
static int get_actuator_hardnesses(lua_State *L);

/**
 ** Returns IMU angle readings 
 **/
static int get_imu_angle(lua_State *L);

/**
 ** Returns IMU acc readings 
 **/
static int get_imu_acc(lua_State *L);

/**
 ** Returns IMU gyro readings 
 **/
static int get_imu_gyr(lua_State *L);

/**
 ** Returns whether the data->luaNewSet flag is on
 **/
static int get_flag(lua_State *L);

/**
 ** Use CPP to set motion
 **/
static int set_actuator_position_forever(lua_State *L);

static int get_sensor_current(lua_State *L);

static int get_actuator_velocity(lua_State *L);

static int get_time(lua_State *L);

static int get_sensor_batteryCharge(lua_State *L);

static int get_sensor_button(lua_State *L);

static int get_sensor_bumperLeft(lua_State *L);
static int get_sensor_bumperRight(lua_State *L);

static int get_sensor_sonarLeft(lua_State *L);

static int get_sensor_sonarRight(lua_State *L);

static int get_sensor_temperature(lua_State *L);

static int set_actuator_ultraSonic(lua_State *L);
static int get_sensor_fsrLeft(lua_State *L);

static int get_sensor_fsrRight(lua_State *L);

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

static int set_actuator_ledFootLeft(lua_State *L);

static int set_actuator_ledFootRight(lua_State *L);

static int set_actuator_ledEarsLeft(lua_State *L);

static int set_actuator_ledEarsRight(lua_State *L);

static int set_actuator_ledFaceLeft(lua_State *L);

static int set_actuator_ledFaceRight(lua_State *L);

static int set_actuator_ledChest(lua_State *L);

static int set_actuator_ledHead(lua_State *L);


extern "C" int luaopen_bhlowcmd(lua_State *L);
