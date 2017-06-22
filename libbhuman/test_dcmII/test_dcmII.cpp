#include "dcmII.h"
#include <iostream>
#include <cmath>

void simple_test(DcmII dcmIIobj) {
	int test = dcmIIobj.testfunction();
	std::cout << test << std::endl;
}

void set_actuator_positions_hardnesses_test(DcmII dcmIIobj) {
	// int ids[2] = {6, 7};
	int ids[22] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21};
	float hardness_values[22] = {0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75};


	// float hardness_values[2] = {0.7, 0.7};
	dcmIIobj.set_actuator_hardnesses(hardness_values, ids, 22);

	// float values[2] = {0.5, -0.5};
	// dcmIIobj.set_actuator_positions(values, ids);
}

void rotate_set_actuator_positions_hardnesses_test(DcmII dcmIIobj) {
	// rotate head and print positions/hardnesses
	// int ids[2] = {0, 1};

	// float hardness_values[2] = {0.7, 0.7};
	// dcmIIobj.set_actuator_hardnesses(hardness_values, ids, 22);
	int hardness_ids[22] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21};
	float hardness_values[22] = {0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75};

	dcmIIobj.set_actuator_hardnesses(hardness_values, hardness_ids, 22);

	int position_ids[22] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21};
	float position_values[22] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

	float speed = 0.04;
	int count = 0;
	// float result[100] = {0};
	// float size = 0.0f;
	// float hardness_result[100] = {0};
	// float hardness_size = 0.0f;

	std::cout<< "entered function" <<std::endl;
	while (1) {
		position_values[2] = cos(speed*count);
		dcmIIobj.set_actuator_positions(position_values, position_ids);
		count = count + 1;
		usleep(0.01*1000000);

		std::cout<< "loop" <<std::endl;

// 		if (count % 1 == 0) {
// 			dcmIIobj.get_actuator_positions(result, size);
// 			dcmIIobj.get_actuator_hardnesses(hardness_result, hardness_size);
// 			
// 			for (int i = 0; i < size; i++) {
// 				std::cout << i << "   position: " << result[i];
// 				std::cout << "      hardness: " << hardness_result[i] << std::endl;
// 			}
// 		}
	}
}

void set_actuator_position_hardness_test(DcmII dcmIIobj) {
	dcmIIobj.set_actuator_position(0.55, 17); // knee pitch
	dcmIIobj.set_actuator_hardness(0.58, 17);
}

void get_actuator_position_hardness_test(DcmII dcmIIobj) {
	float pos = 0.0f;
	dcmIIobj.get_actuator_position(pos, 17);
	cout << "position value: " << pos << endl;

	float hardness = 0.0f;
	dcmIIobj.get_actuator_hardness(hardness, 17);
	cout << "hardness value: " << hardness << endl;
}

// void set_actuator_command_test(DcmII dcmIIobj) {
// 	float joint_values[6] = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
// 	dcmIIobj.set_actuator_command(joint_values, 6);
// 
// 	float hardness_values[6] = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
// 	int ids[6] = {6, 7, 8, 9, 10, 11};
// 	dcmIIobj.set_actuator_hardnesses(hardness_values, ids);
// }
// 
// void get_actuator_command_test(DcmII dcmIIobj) {
// 	float result[100] = {};
// 	float size = 0.0f;
// 	int startIndex = 6;
// 
// 	dcmIIobj.get_actuator_command(result, size, startIndex);
// 	for (int i=0; i<size; i++) {
// 		cout << "i: " << result[i] << endl;
// 	}
// }

void get_sensor_list_test(DcmII dcmIIobj) {
	float result[100] = {};
	float size = 0.0f;
	dcmIIobj.get_sensor_list(result, size);

	for (int i=0; i<size; i++) {
		cout << i << ": " << result[i] << endl;
	}

	cout << "size: " << size << endl;
}





////////////////////// DP /////////////////////





void get_actuator_positions_hardnesses_test(DcmII dcmIIobj) {
	int count = 0;

	float result[100] = {0};
	float size = 0.0f;
	float hardness_result[100] = {0};
	float hardness_size = 0.0f;
	while (count < 1) {
		dcmIIobj.get_actuator_positions(result, size);
		dcmIIobj.get_actuator_hardnesses(hardness_result, hardness_size);
		
		for (int i = 0; i < size; i++) {
			std::cout << i << "   position: " << result[i];
			std::cout << "      hardness: " << hardness_result[i] << std::endl;
		}

		count = count + 1;
	}
	
	std::cout << "position size: " << size << std::endl;
	std::cout << "hardness size: " << hardness_size << std::endl;
}

void get_sensor_positions_test(DcmII dcmIIobj) {
	float result[100] = {0};
	float size = 0.0f;
	dcmIIobj.get_sensor_positions(result, size);
	
	for (int i = 0; i < size; i++) {
		std::cout << i << "   position: " << result[i] << std::endl;
	}
	
	std::cout << "position size: " << size << std::endl;
}

void get_imu_acc_test(DcmII dcmIIobj) {
	float result[100] = {0};
	float size = 0.0f;
	int count = 0;

	while (count < 10000000) {
		if (count % 100 == 0) {
			dcmIIobj.get_imu_acc(result, size);
			for (int i = 0; i < size; i++) {
				std::cout << i << "   acc: " << result[i] << std::endl;
			}
			std::cout << std::endl;
		}
		count = count + 1;
	}
	
	std::cout << "imu acc size: " << size << std::endl;
}

void get_imu_angle_test(DcmII dcmIIobj) {
	float result[100] = {0};
	float size = 0.0f;
	int count = 0;

	while (count < 10000000) {
		if (count % 100 == 0) {
			dcmIIobj.get_imu_angle(result, size);
			for (int i = 0; i < size; i++) {
				std::cout << i << "   angle: " << result[i] << std::endl;
			}
			std::cout << std::endl;
		}
		count = count + 1;
	}

	std::cout << "imu angle size: " << size << std::endl;
}

void get_imu_gyr_test(DcmII dcmIIobj) {
	float result[100] = {0};
	float size = 0.0f;
	int count = 0;

	while (count < 10000000) {
		if (count % 100 == 0) {
			dcmIIobj.get_imu_gyr(result, size);
			for (int i = 0; i < size; i++) {
				std::cout << i << "   gyr: " << result[i] << std::endl;
			}
			std::cout << std::endl;
		}
		count = count + 1;
	}
	
	std::cout << "imu gyr size: " << size << std::endl;
}

void get_time_test(DcmII dcmIIobj) {
	float result = 0.0f;
  int count = 0;
	while (count < 10000000) {
		if (count % 100 == 0) {
				dcmIIobj.get_time(result);
				std::cout << "time: " << result << std::endl;
			}
		count = count + 1;
	}
	
}

void get_sensor_batteryCharge_test(DcmII dcmIIobj) {
	float result = 0.0f;
	dcmIIobj.get_sensor_batteryCharge(result);
	
	std::cout << "battery Charge (x/1): " << result << std::endl;
}

void get_sensor_button_test(DcmII dcmIIobj) {
  float result = 0.0f;
  int count = 0;

  while (count < 10000000) {
    if (count % 1000 == 0) {
        dcmIIobj.get_sensor_button(result);
        std::cout << "button pressed: " << result << std::endl;
      }
    count = count + 1;
  }
}

void get_sensor_bumperLeft_test(DcmII dcmIIobj) {
  float result[100] = {0};
  float size = 0.0f;
  int count = 0;

  while (count < 10000000) {
    if (count % 100 == 0) {
      dcmIIobj.get_sensor_bumperLeft(result, size);
      for (int i = 0; i < size; i++) {
        std::cout << i << "   bumperLeft: " << result[i] << std::endl;
      }
      std::cout << std::endl;
    }
    count = count + 1;
  }
  
  std::cout << "bumperLeft size: " << size << std::endl;
}

void get_sensor_bumperRight_test(DcmII dcmIIobj) {
  float result[100] = {0};
  float size = 0.0f;
  int count = 0;

  while (count < 10000000) {
    if (count % 100 == 0) {
      dcmIIobj.get_sensor_bumperRight(result, size);
      for (int i = 0; i < size; i++) {
        std::cout << i << "   bumperRight: " << result[i] << std::endl;
      }
      std::cout << std::endl;
    }
    count = count + 1;
  }
  
  std::cout << "bumperRight size: " << size << std::endl;
}

void get_sensor_sonarLeft_test(DcmII dcmIIobj) {
  float result[100] = {0};
  float size = 0.0f;
  int count = 0;

  dcmIIobj.set_actuator_ultraSonic(68);
  while (count < 10000000) {
    if (count % 100 == 0) {
      dcmIIobj.get_sensor_sonarLeft(result, size);
      for (int i = 0; i < size; i++) {
        std::cout << i << "   sonarLeft: " << result[i] << std::endl;
      }
      std::cout << std::endl;
    }
    count = count + 1;
  }
  
  std::cout << "sonarLeft size: " << size << std::endl;
}

void get_sensor_sonarRight_test(DcmII dcmIIobj) {
  float result[100] = {0};
  float size = 0.0f;
  int count = 0;

  dcmIIobj.set_actuator_ultraSonic(68);
  while (count < 10000000) {
    if (count % 1000 == 0) {
      dcmIIobj.get_sensor_sonarRight(result, size);
      for (int i = 0; i < size; i++) {
        std::cout << i << "   sonarRight: " << result[i] << std::endl;
      }
      std::cout << std::endl;
    }
    count = count + 1;
  }
  
  std::cout << "sonarRight size: " << size << std::endl;
}

void get_sensor_temperature_test(DcmII dcmIIobj) {
  float result[100] = {0};
  float size = 0.0f;
  dcmIIobj.get_sensor_temperature(result, size);
  
  for (int i = 0; i < size; i++) {
    std::cout << i << "   temperature: " << result[i] << std::endl;
  }
  
  std::cout << "temperature size: " << size << std::endl;
}

void get_sensor_current_test(DcmII dcmIIobj) {
  float result[100] = {0};
  float size = 0.0f;
  dcmIIobj.get_sensor_current(result, size);
  
  for (int i = 0; i < size; i++) {
    std::cout << i << "   current: " << result[i] << std::endl;
  }
  
  std::cout << "current size: " << size << std::endl;
}

void get_sensor_fsrLeft_test(DcmII dcmIIobj) {
  float result[100] = {0};
  float size = 0.0f;
  int count = 0;

  while (count < 10000000) {
    if (count % 100 == 0) {
      dcmIIobj.get_sensor_fsrLeft(result, size);
      for (int i = 0; i < size; i++) {
        std::cout << i << "   fsrLeft: " << result[i] << std::endl;
      }
      std::cout << std::endl;
    }
    count = count + 1;
  }
  
  std::cout << "fsrLeft size: " << size << std::endl;
}

void get_sensor_fsrRight_test(DcmII dcmIIobj) {
  float result[100] = {0};
  float size = 0.0f;
  int count = 0;

  while (count < 10000000) {
    if (count % 100 == 0) {
      dcmIIobj.get_sensor_fsrRight(result, size);
      for (int i = 0; i < size; i++) {
        std::cout << i << "   fsrRight: " << result[i] << std::endl;
      }
      std::cout << std::endl;
    }
    count = count + 1;
  }
  
  std::cout << "fsrRight size: " << size << std::endl;
}

int main() {
	DcmII dcmIIobj;

	rotate_set_actuator_positions_hardnesses_test(dcmIIobj);
	// set_actuator_positions_hardnesses_test(dcmIIobj);

	std::cout <<"Hello World!" <<std::endl;

	return 0;
}
