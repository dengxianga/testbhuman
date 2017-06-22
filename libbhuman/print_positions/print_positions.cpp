#include "dcmII.h"
#include <iostream>
#include <cmath>

int main() {
	DcmII dcmIIobj;

	float time = 0.00000000000f;

	float expected_result[22] = {0};
	float expected_size = 0.0f;

	float actual_result[22] = {0};
	float actual_size = 0.0f;

	while (1) {
		dcmIIobj.get_time(time);
		dcmIIobj.get_actuator_positions(expected_result, expected_size);
		dcmIIobj.get_sensor_positions(actual_result, actual_size);

		std::cout << std::fixed;
		std::cout << time << " "; 

		for (int i = 0; i < expected_size; i++) {
			std::cout << expected_result[i] << " ";
		}

		for (int i = 0; i < actual_size; i++) {
			std::cout << actual_result[i] << " ";
		}

		std::cout << std::endl;


	}

	return 0;
}
