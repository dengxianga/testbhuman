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

// static void set_actuator_positions(std::vector<double> vs, std::vector<double> ids);

// static void set_actuator_hardnesses(std::vector<double> vs, std::vector<double> ids);

// static void set_actuator_position(double x, int ind);

// static int get_actuator_positions(float* in_out_buffer, float& size);

// static void set_actuator_hardness(double x, int ind);

// static void get_actuator_position(float& value, int ind);

// static void get_actuator_hardness(float& value, int ind);

// static void set_actuator_command(std::vector<double> joint_values, int startInd);

// static void get_actuator_command(float* in_out_buffer, float& size, int startInd);

// static void get_sensor_position(float& value, int ind);

// static void get_sensor_positions(float* in_out_buffer, float& size);

// static void get_actuator_hardnesses(float* in_out_buffer, float& size);

// static void get_imu_angle(float* in_out_buffer, float& size);

// static void get_imu_acc(float* in_out_buffer, float& size);

// static void get_imu_gyr(float* in_out_buffer, float& size);

// static void get_flag(bool& flag);

// static void set_actuator_position_forever(std::vector<double> ids, std::vector<double> vs);

// static void get_sensor_current(float* in_out_buffer, float& size);

// static void set_actuator_velocity();

// static void get_actuator_velocity(float& value);

// static void get_time(float& time);

// static void get_sensor_batteryCharge(float& value);

// static void get_sensor_button(float& value);

// static void get_sensor_bumperLeft(float* in_out_buffer, float& size);

// static void get_sensor_bumperRight(float* in_out_buffer, float& size);

// static void get_sensor_sonarLeft(float* in_out_buffer, float& size);

// static void get_sensor_sonarRight(float* in_out_buffer, float& size);

// static void get_sensor_temperature(float* in_out_buffer, float& size);

// static void set_actuator_ultraSonic(int command);

// static void get_sensor_fsrLeft(float* in_out_buffer, float& size);

// static void get_sensor_fsrRight(float* in_out_buffer, float& size);

// static void set_actuator_ledFootLeft(std::vector<double> values);

// static void set_actuator_ledFootRight(std::vector<double> values);

// static void set_actuator_ledEarsLeft(std::vector<double> values);

// static void set_actuator_ledEarsRight(std::vector<double> values);

// static void set_actuator_ledFaceLeft(std::vector<double> values, int ind);

// static void set_actuator_ledFaceRight(std::vector<double> values, int ind);

// static void set_actuator_ledChest(std::vector<double> values);

// static void set_actuator_ledHead(std::vector<double> values);
