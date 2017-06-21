#include "dcmII.h"

double speed = 0.01;

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
LBHData * data;
bool initialized = false;
int writingActuators = -1;

DcmII::DcmII() {}

// int DcmII::testfunction() {
// 	return 779997;
// }

// int testfunction() {
// 	return 99779;
// }

void DcmII::lua_initialize() {
  fd = shm_open(LBH_MEM_NAME, O_RDWR, S_IRUSR | S_IWUSR);
  data = (LBHData*)mmap(NULL, sizeof(LBHData), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  initialized = true;
}

// static void lua_initialize () {
//   fd = shm_open(LBH_MEM_NAME, O_RDWR, S_IRUSR | S_IWUSR);
//   data = (LBHData*)mmap(NULL, sizeof(LBHData), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
//   initialized = true;
// }

// static void set_actuator_positions(std::vector<double> vs, std::vector<double> ids) {
//   if (!initialized) {
//     lua_initialize();
//   }
//   //////// control with luawrapper, buffer
//   for (unsigned int i = 0; i < vs.size(); i++) {
//     int bHumanIndex = luaToBHumanPos[(int) ids[i] - 1];
//     data->luaBuffer[bHumanIndex] = vs[i];
//   }

//   data->luaNewSet = true;
// }

// static void set_actuator_hardnesses(std::vector<double> vs, std::vector<double> ids) {
//   if (!initialized) {
//     lua_initialize();
//   }

//   for (unsigned int i = 0; i < vs.size(); i++) {
//     int bHumanIndex = luaToBHumanPos[(int) ids[i] - 1];
//     data->luaBuffer[bHumanIndex + lbhNumOfPositionActuatorIds] = vs[i];
//   }

//   data->luaNewSet = true;
// }

// static void set_actuator_position(double x, int ind) {
//   if (!initialized) {
//     lua_initialize();
//   }

//   int index = index - 1; // convert to zero-indexed

//   int bHumanIndex = luaToBHumanPos[index];
//   data->luaBuffer[bHumanIndex] = x;

//   data->luaNewSet = true;
// }

/**
 ** Returns actuator command for all positions
 **/

void DcmII::get_actuator_positions(float* in_out_buffer, float& size) {
  if (!initialized) {
    lua_initialize();
  }
  data->readingActuators = data->newestActuators;
  float * actuators = data->actuators[data->readingActuators];

  float actuatorPositions[nJoint];
  for (int i = 0; i < nJoint; i++) {
    int bHumanIndex = luaToBHumanPos[i];
    actuatorPositions[i] = actuators[bHumanIndex];
  }
  size = nJoint;

  // for (int i=0; i<size; i++) {
  //   cout<<actuatorPositions[i]<<endl;
  // }

  memcpy(in_out_buffer, actuatorPositions, sizeof(actuatorPositions));
}


// static void get_actuator_positions(float* in_out_buffer, float& size) {
//   if (!initialized) {
//     lua_initialize();
//   }
//   data->readingActuators = data->newestActuators;
//   float * actuators = data->actuators[data->readingActuators];
// 
//   double actuatorPositions[nJoint];
//   for (int i = 0; i < nJoint; i++) {
//     int bHumanIndex = luaToBHumanPos[i];
//     actuatorPositions[i] = (double) actuators[bHumanIndex];
//   }
//   size = nJoint;
//   memcpy(in_out_buffer, actuatorPositions, sizeof(actuatorPositions));
// }

// static void set_actuator_hardness(double x, int ind) {
//   if (!initialized) {
//     lua_initialize();
//   }
  
//   int index = ind - 1; // convert to zero-indexed
  
//   int bHumanIndex = luaToBHumanPos[index];
//   data->luaBuffer[bHumanIndex + lbhNumOfPositionActuatorIds] = x;

//   data->luaNewSet = true;
// }

// /**
//  ** Returns actuator command for single position
//  **/
// static void get_actuator_position(float& value, int ind) {
//   if (!initialized) {
//     lua_initialize();
//   }

//   int index = ind - 1; // convert to zero-indexed
//   data->readingActuators = data->newestActuators;
//   float * actuators = data->actuators[data->readingActuators];

//   int bHumanIndex = luaToBHumanPos[index];
//   value = actuators[bHumanIndex];
// }

// /**
//  ** Returns actuator command for single hardsness
//  **/
// static void get_actuator_hardness(float& value, int ind) {
//   if (!initialized) {
//     lua_initialize();
//   }
//   int index = ind - 1; // convert to zero-indexed
//   data->readingActuators = data->newestActuators;
//   float * actuators = data->actuators[data->readingActuators];

//   int bHumanIndex = luaToBHumanPos[index];
//   value = actuators[bHumanIndex + lbhNumOfPositionActuatorIds];
// }

// /**
//  ** Sets entire list of actuator values starting with the initial index
//  ** @param list of actuator positions, initial index
//  **/
// static void set_actuator_command(std::vector<double> joint_values, int startInd) {
//   if (!initialized) {
//     lua_initialize();
//   }

//   int startIndex = startInd - 1;
  
//   for (int i = startIndex; i < startIndex + joint_values.size(); i++) {
//     // std::cout << "i: " << i << "   joint_values[i]: " << joint_values[i-startIndex] << std::endl;
//     int bHumanIndex = luaToBHumanPos[i];
//     data->luaBuffer[bHumanIndex] = joint_values[i-startIndex];
//   }

//   data->luaNewSet = true;
// }

// /**
//  ** Returns entire list of actuator values starting with the initial index
//  ** @param starting index of head, l/r arm, or l/r leg
//  ** @return list of actuator positions
//  **/
// static void get_actuator_command(float* in_out_buffer, float& size, int startInd) {
//   if (!initialized) {
//     lua_initialize();
//   }

//   int startIndex = startInd - 1;
//   int numActuators;

//   switch (startIndex) {
//     case indexHead: numActuators = nJointHead; break;
//     case indexLArm: numActuators = nJointLArm; break;
//     case indexLLeg: numActuators = nJointLLeg; break;
//     case indexRLeg: numActuators = nJointRLeg; break;
//     case indexRArm: numActuators = nJointRArm; break;
//   }

//   data->readingActuators = data->newestActuators;
//   float * actuators = data->actuators[data->newestActuators];
//   float actuatorCommands[numActuators];
//   for (int i = 0; i < numActuators; i++) {
//     int bHumanIndex = luaToBHumanPos[i];
//     actuatorCommands[i] = (double) actuators[bHumanIndex];
//   }

//   size = numActuators;
//   memcpy(in_out_buffer, actuatorCommands, sizeof(actuatorCommands));
// }

// static void get_sensor_position(float& value, int ind) {
//   if (!initialized) {
//     lua_initialize();
//   }

//   int index = ind - 1; // convert to zero-indexed

//   data->readingSensors = data->newestSensors;
//   float * sensors = data->sensors[data->newestSensors];
//   int bHumanIndex = luaToBHumanPos[index];

//   value = sensors[3*bHumanIndex];

// }

// /**
//  ** Returns all joint positions read by sensors
//  **/
// static void get_sensor_positions(float* in_out_buffer, float& size) {  
//   if (!initialized) {
//     lua_initialize();
//   }
//   data->readingSensors = data->newestSensors;
//   float * sensors = data->sensors[data->newestSensors];
//   float actuatorSensors[nJoint];
//   for (int i = 0; i < nJoint; i++) {
//     int bHumanIndex = luaToBHumanPos[i];
//     actuatorSensors[i] = (double) sensors[3*bHumanIndex];
//   }

//   size = nJoint;
//   memcpy(in_out_buffer, actuatorSensors, sizeof(actuatorSensors));
// }

// /**
//  ** Returns all set joint hardness values
//  **/
// static void get_actuator_hardnesses(float* in_out_buffer, float& size) {
//   if (!initialized) {
//     lua_initialize();
//   }
//   data->readingActuators = data->newestActuators;
//   float * actuators = data->actuators[data->readingActuators];
//   double actuatorHardnesses[nJoint];
//   for (int i = 0; i < nJoint; i++) {
//     int bHumanIndex = luaToBHumanPos[i];
//     actuatorHardnesses[i] = (double) actuators[bHumanIndex +lbhNumOfPositionActuatorIds];
//   }
//   size = nJoint;
//   memcpy(in_out_buffer, actuatorHardnesses, sizeof(actuatorHardnesses));
// }

// /**
//  ** Returns IMU angle readings 
//  **/
// static void get_imu_angle(float* in_out_buffer, float& size) {
//   if (!initialized) {
//     lua_initialize();
//   }
//   data->readingSensors = data->newestSensors;
//   float * sensors = data->sensors[data->readingSensors];
//   double ImuReadings[3] = {(double) sensors[angleXSensor], (double) sensors[angleYSensor], sensors[angleZSensor]};
//   size = nJoint;
//   memcpy(in_out_buffer, ImuReadings, sizeof(ImuReadings));
// }

// /**
//  ** Returns IMU acc readings 
//  **/
// static void get_imu_acc(float* in_out_buffer, float& size) {
//   if (!initialized) {
//     lua_initialize();
//   }
//   data->readingSensors = data->newestSensors;
//   float * sensors = data->sensors[data->readingSensors];
//   double AccReadings[3] = {(double) sensors[accXSensor], (double) sensors[accYSensor], (double) sensors[accZSensor]};
//   size = nJoint;
//   memcpy(in_out_buffer, AccReadings, sizeof(AccReadings));
// }

// /**
//  ** Returns IMU gyro readings 
//  **/
// static void get_imu_gyr(float* in_out_buffer, float& size) {
//   if (!initialized) {
//     lua_initialize();
//   }
//   data->readingSensors = data->newestSensors;
//   float * sensors = data->sensors[data->readingSensors];
//   double GyroReadings[3] = {(double) sensors[gyroXSensor], (double) sensors[gyroYSensor], (double) -sensors[gyroZSensor]};
//   size = nJoint;
//   memcpy(in_out_buffer, GyroReadings, sizeof(GyroReadings));
// }

// /**
//  ** Returns whether the data->luaNewSet flag is on
//  **/
// static void get_flag(bool& flag) {
//   flag = data->luaNewSet;
// }

// /**
//  ** Use CPP to set motion
//  **/
// static void set_actuator_position_forever(std::vector<double> ids, std::vector<double> vs) {
//   if (!initialized) {
//     lua_initialize();
//   }

//   std::cout<<"i'm starting"<<std::endl;
//   int cnt = 0.1;
//   data->luaBuffer[headYawStiffnessActuator] = 0.5;
//   while (1) {
//     if (data->luaNewSet == false) {
      
//       vs[0] = cos(speed*cnt);
//       for (unsigned int i = 0; i < vs.size(); i++) {
//         data->luaBuffer[(int)ids[i]-1] = vs[i];
//       }

//       data->luaNewSet = true;
//       std::cout<<"i'm running"<<std::endl;
//       cnt++;
      
      
//     }
//     data->readingSensors = data->newestSensors;
//     float * sensors = data->sensors[data->readingSensors];
//     std::cout<<sensors[1] * 1000<< std::endl;
//     std::cout<<data->luaNewSet<<std::endl;
//   }
// }

// static void get_sensor_current(float* in_out_buffer, float& size) {
//   if (!initialized) {
//     lua_initialize();
//   }

//   data->readingSensors = data->newestSensors;
//   float * sensors = data->sensors[data->readingSensors];

//   double sensorCurrents[nJoint];
//   for (int i = 0; i < nJoint; i++) {
//     int bHumanIndex = luaToBHumanPos[i];
//     sensorCurrents[i] = (double) sensors[3 * bHumanIndex + 1];
//   }
//   size = nJoint;
//   memcpy(in_out_buffer, sensorCurrents, sizeof(sensorCurrents));
// }

// static void set_actuator_velocity() {
//   if (!initialized) {
//     lua_initialize();
//   }
// }

// static void get_actuator_velocity(float& value) {
//   if (!initialized) {
//     lua_initialize();
//   }
//   value = 0;
// }

// static void get_time(float& time) {
//   if (!initialized) {
//     lua_initialize();
//   }

//   struct timeval tv;
//   gettimeofday(&tv, NULL);
//   double t = tv.tv_sec + 1E-6*tv.tv_usec;
//   time = (float) t;
// }

// static void get_sensor_batteryCharge(float& value) {
//   if (!initialized) {
//     lua_initialize();
//   }

//   data->readingSensors = data->newestSensors;
//   float * sensors = data->sensors[data->readingSensors];
//   value = sensors[batteryChargeSensor];
// }

// static void get_sensor_button(float& value) {
//   if (!initialized) {
//     lua_initialize();
//   }

//   data->readingSensors = data->newestSensors;
//   float * sensors = data->sensors[data->readingSensors];
//   value = sensors[chestButtonSensor];
// }

// static void get_sensor_bumperLeft(float* in_out_buffer, float& size) {
//   if (!initialized) {
//     lua_initialize();
//   }

//   data->readingSensors = data->newestSensors;
//   float * sensors = data->sensors[data->readingSensors];
//   float bumperReadings[2] = {sensors[lBumperLeftSensor], sensors[lBumperRightSensor]};
//   memcpy(in_out_buffer, bumperReadings, sizeof(bumperReadings));
// }

// static void get_sensor_bumperRight(float* in_out_buffer, float& size) {
//   if (!initialized) {
//     lua_initialize();
//   }

//   data->readingSensors = data->newestSensors;
//   float * sensors = data->sensors[data->readingSensors];
//   double bumperReadings[2] = {(double) sensors[rBumperLeftSensor], (double) sensors[rBumperRightSensor]};
//   memcpy(in_out_buffer, bumperReadings, sizeof(bumperReadings));
// }

// static void get_sensor_sonarLeft(float* in_out_buffer, float& size) {
//   if (!initialized) {
//     lua_initialize();
//   }

//   data->readingSensors = data->newestSensors;
//   float * sensors = data->sensors[data->readingSensors];

//   double sonarSensors[10];
//   for (int i=0; i<10; i++) {
//     //std::cout << "lua_dcmII.cpp index: " << i+lUsSensor << std::endl;
//     sonarSensors[i]=sensors[i+lUsSensor];
//   }
//   memcpy(in_out_buffer, sonarSensors, sizeof(sonarSensors));
// }

// static void get_sensor_sonarRight(float* in_out_buffer, float& size) {
//   if (!initialized) {
//     lua_initialize();
//   }

//   data->readingSensors = data->newestSensors;
//   float * sensors = data->sensors[data->readingSensors];

//   double sonarSensors[10];
//   for (int i=0; i<10; i++) {
//     sonarSensors[i]=sensors[i+rUsSensor];
//   }
//   memcpy(in_out_buffer, sonarSensors, sizeof(sonarSensors));
// }

// static void get_sensor_temperature(float* in_out_buffer, float& size) {
//   if (!initialized) {
//     lua_initialize();
//   }

//   data->readingSensors = data->newestSensors;
//   float * sensors = data->sensors[data->readingSensors];

//   double sensorTemperatures[nJoint];
//   for (int i = 0; i < nJoint; i++) {
//     int bHumanIndex = luaToBHumanPos[i];
//     sensorTemperatures[i] = (double) sensors[3 * bHumanIndex + 2];
//   }
//   memcpy(in_out_buffer, sensorTemperatures, sizeof(sensorTemperatures));
// }

// static void set_actuator_ultraSonic(int command) {
//   if (!initialized) {
//     lua_initialize();
//   }

//   data->luaBuffer[usActuator] = command;
//   data->luaNewSet = true;
// }

// static void get_sensor_fsrLeft(float* in_out_buffer, float& size) {
//   if (!initialized) {
//     lua_initialize();
//   }

//   data->readingSensors = data->newestSensors;
//   float * sensors = data->sensors[data->readingSensors];
//   double fsrSensors[4] = {(double) sensors[lFSRFrontLeftSensor], (double) sensors[lFSRRearLeftSensor], (double) sensors[lFSRFrontRightSensor], (double) sensors[lFSRRearRightSensor]};
  
//   memcpy(in_out_buffer, fsrSensors, sizeof(fsrSensors));
// }

// static void get_sensor_fsrRight(float* in_out_buffer, float& size) {
//   if (!initialized) {
//     lua_initialize();
//   }

//   data->readingSensors = data->newestSensors;
//   float * sensors = data->sensors[data->readingSensors];
//   double fsrSensors[4] = {(double) sensors[rFSRFrontLeftSensor], (double) sensors[rFSRRearLeftSensor], (double) sensors[rFSRFrontRightSensor], (double) sensors[rFSRRearRightSensor]};
  
//   memcpy(in_out_buffer, fsrSensors, sizeof(fsrSensors));
// }

// static void set_actuator_ledFootLeft(std::vector<double> values) {
//   if (!initialized) {
//     lua_initialize();
//   }
   
//   for (unsigned int i = 0; i < 3; i++) {
//     std::cout<<"l " << i << "   "<< values[i] << std::endl;
//     data->luaBuffer[i + lFootLedRedActuator] = values[i];
//   }

//   data->luaNewSet = true;
// }

// static void set_actuator_ledFootRight(std::vector<double> values) {
//   if (!initialized) {
//     lua_initialize();
//   }

//   for (unsigned int i = 0; i < 3; i++) {
//     // std::cout<<"l " << i << "   "<< values[i] << std::endl;
//     data->luaBuffer[i + rFootLedRedActuator] = values[i];
//   }

//   data->luaNewSet = true;
// }

// static void set_actuator_ledEarsLeft(std::vector<double> values) {
//   if (!initialized) {
//     lua_initialize();
//   }

//   for (unsigned int i = 0; i < 10; i++) {
//     // std::cout<<"l " << i << "   "<< values[i] << std::endl;
//     data->luaBuffer[i + earsLedLeft0DegActuator] = values[i];
//   }

//   data->luaNewSet = true;
// }

// static void set_actuator_ledEarsRight(std::vector<double> values) {
//   if (!initialized) {
//     lua_initialize();
//   }

//   for (unsigned int i = 0; i < 10; i++) {
//     // std::cout<<"l " << i << "   "<< values[i] << std::endl;
//     data->luaBuffer[i + earsLedRight0DegActuator] = values[i];
//   }

//   data->luaNewSet = true;
// }

// static void set_actuator_ledFaceLeft(std::vector<double> values, int ind) {
//   if (!initialized) {
//     lua_initialize();
//   }
//   int index = ind - 1;

//   for (unsigned int i = 0; i < values.size(); i++) {
//     data->luaBuffer[i + faceLedRedLeft0DegActuator + index] = values[i];
//   }

//   data->luaNewSet = true;
// }

// static void set_actuator_ledFaceRight(std::vector<double> values, int ind) {
//   if (!initialized) {
//     lua_initialize();
//   }
//   int index = ind - 1;

//   for (unsigned int i = 0; i < values.size(); i++) {
//     data->luaBuffer[i + faceLedRedRight0DegActuator + index] = values[i];
//   }

//   data->luaNewSet = true;
// }

// static void set_actuator_ledChest(std::vector<double> values) {
//   if (!initialized) {
//     lua_initialize();
//   }

//   for (unsigned int i = 0; i < 3; i++) {
//     data->luaBuffer[i + chestBoardLedRedActuator] = values[i];
//   }

//   data->luaNewSet = true;
// }

// static void set_actuator_ledHead(std::vector<double> values) {
//   if (!initialized) {
//     lua_initialize();
//   }

//   for (unsigned int i = 0; i < values.size(); i++) {
//     data->luaBuffer[i + headLedRearLeft0Actuator] = values[i];
//   }

//   data->luaNewSet = true;
// }
