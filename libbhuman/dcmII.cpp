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
RShoulderPitch, 
RShoulderRoll,
RElbowYaw, 
RElbowRoll, 
RHipYawPitch,
RHipRoll, 
RHipPitch,
RKneePitch, 
RAnklePitch, 
RAnkleRoll,
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
rShoulderPitchPositionActuator,  
rShoulderRollPositionActuator,  
rElbowYawPositionActuator,  
rElbowRollPositionActuator,
lHipYawPitchPositionActuator, 
rHipRollPositionActuator, 
rHipPitchPositionActuator,  
rKneePitchPositionActuator, 
rAnklePitchPositionActuator,  
rAnkleRollPositionActuator}; 

int fd;
LBHData * data;
bool initialized = false;
int writingActuators = -1;

DcmII::DcmII() {}

int DcmII::testfunction() {
	return 779997;
}

void DcmII::lua_initialize() {
  fd = shm_open(LBH_MEM_NAME, O_RDWR, S_IRUSR | S_IWUSR);
  data = (LBHData*)mmap(NULL, sizeof(LBHData), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  initialized = true;
}

void DcmII::set_actuator_positions(float* vs, int* ids) {
  if (!initialized) {
    lua_initialize();
  }

	int size = 22;
	// int size = sizeof(vs)/sizeof(vs[0]);

  for (unsigned int i = 0; i < size; i++) {
    int bHumanIndex = luaToBHumanPos[ids[i]];
    data->luaBuffer[bHumanIndex] = vs[i];
  }

  data->luaNewSet = true;
}

void DcmII::set_actuator_hardnesses(float* vs, int* ids, int size) {
  if (!initialized) {
    lua_initialize();
  }

	// int size = sizeof(vs)/sizeof(vs[0]);

  for (unsigned int i = 0; i < size; i++) {
    int bHumanIndex = luaToBHumanPos[ids[i]];
    data->luaBuffer[bHumanIndex + lbhNumOfPositionActuatorIds] = vs[i];
  }

  data->luaNewSet = true;
}

void DcmII::set_actuator_position(float x, int index) {
  if (!initialized) {
    lua_initialize();
  }

  int bHumanIndex = luaToBHumanPos[index];
  data->luaBuffer[bHumanIndex] = x;

  data->luaNewSet = true;
}

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

  memcpy(in_out_buffer, actuatorPositions, sizeof(actuatorPositions));
}

void DcmII::set_actuator_hardness(float x, int index) {
  if (!initialized) {
    lua_initialize();
  }

  int bHumanIndex = luaToBHumanPos[index];
  data->luaBuffer[bHumanIndex + lbhNumOfPositionActuatorIds] = x;

  data->luaNewSet = true;
}

/**
 ** Returns actuator command for single position
 **/
void DcmII::get_actuator_position(float& value, int index) {
  if (!initialized) {
    lua_initialize();
  }

  data->readingActuators = data->newestActuators;
  float * actuators = data->actuators[data->readingActuators];

  int bHumanIndex = luaToBHumanPos[index];
  value = actuators[bHumanIndex];
}

/**
 ** Returns actuator command for single hardsness
 **/
void DcmII::get_actuator_hardness(float& value, int index) {
  if (!initialized) {
    lua_initialize();
  }
  
	data->readingActuators = data->newestActuators;
  float * actuators = data->actuators[data->readingActuators];

  int bHumanIndex = luaToBHumanPos[index];
  value = actuators[bHumanIndex + lbhNumOfPositionActuatorIds];
}

/**
 ** Sets entire list of actuator values starting with the initial index
 ** @param list of actuator positions, initial index
 **/
// void DcmII::set_actuator_command(float* joint_values, int startIndex) {
//   if (!initialized) {
//     lua_initialize();
//   }
// 
// 	int size = sizeof(joint_values)/sizeof(joint_values[0]);
// 
// 	cout << "size: " << size << endl;
// 
//   for (int i = startIndex; i < startIndex + size; i++) {
//     int bHumanIndex = luaToBHumanPos[i];
//     data->luaBuffer[bHumanIndex] = joint_values[i-startIndex];
//   }
// 
//   data->luaNewSet = true;
// }

/**
 ** Returns entire list of actuator values starting with the initial index
 ** @param starting index of head, l/r arm, or l/r leg
 ** @return list of actuator positions
 **/
// void DcmII::get_actuator_command(float* in_out_buffer, float& size, int startIndex) {
//   if (!initialized) {
//     lua_initialize();
//   }
// 
//   int numActuators;
// 
//   switch (startIndex) {
//     case indexHead: numActuators = nJointHead; break;
//     case indexLArm: numActuators = nJointLArm; break;
//     case indexLLeg: numActuators = nJointLLeg; break;
//     case indexRLeg: numActuators = nJointRLeg; break;
//     case indexRArm: numActuators = nJointRArm; break;
// 		default: numActuators = 0; break;
//   }
// 
//   data->readingActuators = data->newestActuators;
//   float * actuators = data->actuators[data->newestActuators];
//   float actuatorCommands[numActuators];
//   for (int i = 0; i < numActuators; i++) {
//     int bHumanIndex = luaToBHumanPos[i];
//     actuatorCommands[i] = actuators[bHumanIndex];
//   }
// 
//   size = numActuators;
//   memcpy(in_out_buffer, actuatorCommands, sizeof(actuatorCommands));
// }

void DcmII::get_sensor_list(float* in_out_buffer, float& size) {
	if (!initialized) {
		lua_initialize();
	}

	data->readingSensors = data->newestSensors;
	float * sensors = data->sensors[data->readingSensors];
	float sensorList[23] = {
		sensors[gyroXSensor],
		sensors[gyroYSensor],
		-sensors[gyroZSensor],
		sensors[accXSensor],
		sensors[accYSensor],
		sensors[accZSensor],
		sensors[angleXSensor],
		sensors[angleYSensor],
		sensors[angleZSensor],
		sensors[batteryChargeSensor],
		sensors[lFSRFrontLeftSensor],
		sensors[lFSRFrontRightSensor],
		sensors[lFSRRearLeftSensor],
		sensors[lFSRRearRightSensor],
		sensors[rFSRFrontLeftSensor],
		sensors[rFSRFrontRightSensor],
		sensors[rFSRRearLeftSensor],
		sensors[rFSRRearRightSensor],
		sensors[chestButtonSensor],
		sensors[lBumperLeftSensor],
		sensors[lBumperRightSensor],
		sensors[rBumperLeftSensor],
		sensors[rBumperRightSensor],
	};

	size = 23;
	memcpy(in_out_buffer, sensorList, sizeof(sensorList));
}









/////////////////////// DP /////////////////////////

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
void DcmII::get_sensor_positions(float* in_out_buffer, float& size) {  
  if (!initialized) {
    lua_initialize();
  }
  data->readingSensors = data->newestSensors;
  float * sensors = data->sensors[data->newestSensors];
  float actuatorSensors[nJoint];
  for (int i = 0; i < nJoint; i++) {
    int bHumanIndex = luaToBHumanPos[i];
    actuatorSensors[i] =  sensors[3*bHumanIndex];
  }

  size = nJoint;
  memcpy(in_out_buffer, actuatorSensors, sizeof(actuatorSensors));
}

// /**
//  ** Returns all set joint hardness values
//  **/
void DcmII::get_actuator_hardnesses(float* in_out_buffer, float& size) {
  if (!initialized) {
    lua_initialize();
  }
  data->readingActuators = data->newestActuators;
  float * actuators = data->actuators[data->readingActuators];
  float actuatorHardnesses[nJoint];
  for (int i = 0; i < nJoint; i++) {
    int bHumanIndex = luaToBHumanPos[i];
    actuatorHardnesses[i] = actuators[bHumanIndex + lbhNumOfPositionActuatorIds];
  }
  size = nJoint;
  memcpy(in_out_buffer, actuatorHardnesses, sizeof(actuatorHardnesses));
}

/**
 ** Returns IMU angle readings 
 **/
void DcmII::get_imu_angle(float* in_out_buffer, float& size) {
  if (!initialized) {
    lua_initialize();
  }
  data->readingSensors = data->newestSensors;
  float * sensors = data->sensors[data->readingSensors];
  float ImuReadings[3] = {sensors[angleXSensor], sensors[angleYSensor], sensors[angleZSensor]};
  size = 3;
  memcpy(in_out_buffer, ImuReadings, sizeof(ImuReadings));
}

/**
 ** Returns IMU acc readings 
 **/
void DcmII::get_imu_acc(float* in_out_buffer, float& size) {
  if (!initialized) {
    lua_initialize();
  }
  data->readingSensors = data->newestSensors;
  float * sensors = data->sensors[data->readingSensors];
  float AccReadings[3] = {sensors[accXSensor], sensors[accYSensor], sensors[accZSensor]};
  size = 3;
  memcpy(in_out_buffer, AccReadings, sizeof(AccReadings));
}

/**
 ** Returns IMU gyro readings 
 **/
void DcmII::get_imu_gyr(float* in_out_buffer, float& size) {
  if (!initialized) {
    lua_initialize();
  }
  data->readingSensors = data->newestSensors;
  float * sensors = data->sensors[data->readingSensors];
  float GyroReadings[3] = {sensors[gyroXSensor], sensors[gyroYSensor], -sensors[gyroZSensor]};
  size = 3;
  memcpy(in_out_buffer, GyroReadings, sizeof(GyroReadings));
}


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

void DcmII::get_sensor_current(float* in_out_buffer, float& size) {
  if (!initialized) {
    lua_initialize();
  }

  data->readingSensors = data->newestSensors;
  float * sensors = data->sensors[data->readingSensors];

  float sensorCurrents[nJoint];
  for (int i = 0; i < nJoint; i++) {
    int bHumanIndex = luaToBHumanPos[i];
    sensorCurrents[i] = sensors[3 * bHumanIndex + 1];
  }

  size = nJoint;
  memcpy(in_out_buffer, sensorCurrents, sizeof(sensorCurrents));
}

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

void DcmII::get_time(float& time) {
  if (!initialized) {
    lua_initialize();
  }

  struct timeval tv;
  gettimeofday(&tv, NULL);
  double t = tv.tv_sec + 1E-6*tv.tv_usec;
  time = (float) t;
}

void DcmII::get_sensor_batteryCharge(float& value) {
  if (!initialized) {
    lua_initialize();
  }

  data->readingSensors = data->newestSensors;
  float * sensors = data->sensors[data->readingSensors];
  value = sensors[batteryChargeSensor];
}

void DcmII::get_sensor_button(float& value) {
  if (!initialized) {
    lua_initialize();
  }

  data->readingSensors = data->newestSensors;
  float * sensors = data->sensors[data->readingSensors];
  value = sensors[chestButtonSensor];
}

void DcmII::get_sensor_bumperLeft(float* in_out_buffer, float& size) {
  if (!initialized) {
    lua_initialize();
  }

  data->readingSensors = data->newestSensors;
  float * sensors = data->sensors[data->readingSensors];
  float bumperReadings[2] = {sensors[lBumperLeftSensor], sensors[lBumperRightSensor]};
  
  size = 2;
  memcpy(in_out_buffer, bumperReadings, sizeof(bumperReadings));
}

void DcmII::get_sensor_bumperRight(float* in_out_buffer, float& size) {
  if (!initialized) {
    lua_initialize();
  }

  data->readingSensors = data->newestSensors;
  float * sensors = data->sensors[data->readingSensors];
  float bumperReadings[2] = {sensors[rBumperLeftSensor], sensors[rBumperRightSensor]};
  
  size = 2;
  memcpy(in_out_buffer, bumperReadings, sizeof(bumperReadings));
}

void DcmII::get_sensor_sonarLeft(float* in_out_buffer, float& size) {
  if (!initialized) {
    lua_initialize();
  }

  data->readingSensors = data->newestSensors;
  float * sensors = data->sensors[data->readingSensors];

  float sonarSensors[10];
  for (int i=0; i<10; i++) {
    sonarSensors[i]=sensors[i+lUsSensor];
  }

  size = 10;
  memcpy(in_out_buffer, sonarSensors, sizeof(sonarSensors));
}

void DcmII::get_sensor_sonarRight(float* in_out_buffer, float& size) {
  if (!initialized) {
    lua_initialize();
  }

  data->readingSensors = data->newestSensors;
  float * sensors = data->sensors[data->readingSensors];

  float sonarSensors[10];
  for (int i=0; i<10; i++) {
    sonarSensors[i]=sensors[i+rUsSensor];
  }

  size = 10;
  memcpy(in_out_buffer, sonarSensors, sizeof(sonarSensors));
}

void DcmII::get_sensor_temperature(float* in_out_buffer, float& size) {
  if (!initialized) {
    lua_initialize();
  }

  data->readingSensors = data->newestSensors;
  float * sensors = data->sensors[data->readingSensors];

  float sensorTemperatures[nJoint];
  for (int i = 0; i < nJoint; i++) {
    int bHumanIndex = luaToBHumanPos[i];
    sensorTemperatures[i] = sensors[3 * bHumanIndex + 2];
  }

  size = nJoint;
  memcpy(in_out_buffer, sensorTemperatures, sizeof(sensorTemperatures));
}

void DcmII::set_actuator_ultraSonic(int command) {
  if (!initialized) {
    lua_initialize();
  }

  data->luaBuffer[usActuator] = command;
  data->luaNewSet = true;
}

void DcmII::get_sensor_fsrLeft(float* in_out_buffer, float& size) {
  if (!initialized) {
    lua_initialize();
  }

  data->readingSensors = data->newestSensors;
  float * sensors = data->sensors[data->readingSensors];
  float fsrSensors[4] = {sensors[lFSRFrontLeftSensor], sensors[lFSRRearLeftSensor], sensors[lFSRFrontRightSensor], sensors[lFSRRearRightSensor]};
  
  size = 4;
  memcpy(in_out_buffer, fsrSensors, sizeof(fsrSensors));
}

void DcmII::get_sensor_fsrRight(float* in_out_buffer, float& size) {
  if (!initialized) {
    lua_initialize();
  }

  data->readingSensors = data->newestSensors;
  float * sensors = data->sensors[data->readingSensors];
  float fsrSensors[4] = {sensors[rFSRFrontLeftSensor], sensors[rFSRRearLeftSensor], sensors[rFSRFrontRightSensor], sensors[rFSRRearRightSensor]};
  
  size = 4;
  memcpy(in_out_buffer, fsrSensors, sizeof(fsrSensors));
}

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
