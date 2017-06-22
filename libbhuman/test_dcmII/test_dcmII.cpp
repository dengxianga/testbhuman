#include "dcmII.h"
#include <iostream>
#include <cmath>

void simple_test(DcmII dcmIIobj) {
	int test = dcmIIobj.testfunction();
	std::cout << test << std::endl;
}

// ========== Retest! ==================
void set_actuator_positions_hardnesses_test(DcmII dcmIIobj) {
	int ids[2] = {0, 1};

	float hardness_values[2] = {0.7, 0.7};
	dcmIIobj.set_actuator_hardnesses(hardness_values, ids, 2);

	float values[2] = {0.5, -0.5};
	dcmIIobj.set_actuator_positions(values, ids, 2);
}

// ========== Retest! ==================
void rotate_set_actuator_positions_hardnesses_test(DcmII dcmIIobj) {
	// rotate head and print positions/hardnesses
	int ids[2] = {0, 1};

	float hardness_values[2] = {0.7, 0.7};
	dcmIIobj.set_actuator_hardnesses(hardness_values, ids, 2);

	float values[2] = {0.5, 0};
	float speed = 0.0005;
	int count = 0;
	float result[100] = {0};
	float size = 0.0f;
	float hardness_result[100] = {0};
	float hardness_size = 0.0f;
	while (1) {
		values[0] = cos(speed*count);
		dcmIIobj.set_actuator_positions(values, ids, 2);
		count = count + 1;

		if (count % 1 == 0) {
			dcmIIobj.get_actuator_positions(result, size);
			dcmIIobj.get_actuator_hardnesses(hardness_result, hardness_size);
			
			for (int i = 0; i < size; i++) {
				std::cout << i << "   position: " << result[i];
				std::cout << "      hardness: " << hardness_result[i] << std::endl;
			}
		}
	}
}

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

// ========== Retest! ==================
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

// ========== Retest! ==================
void get_sensor_sonarRight_test(DcmII dcmIIobj) {
  float result[100] = {0};
  float size = 0.0f;
  int count = 0;

  dcmIIobj.set_actuator_ultraSonic(68);
  while (count < 10000000) {
    if (count % 100 == 0) {
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

// ========== Retest! ==================
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
	
	get_sensor_bumperLeft_test(dcmIIobj);

	std::cout <<"Hello World!" <<std::endl;

	return 0;
}
