#include "dcmII.h"
#include <iostream>

int main() {

	DcmII dcmIIobj;
	// int test = dcmIIobj.testfunction();

	// std::cout << test << std::endl;

	float result[100] = {0};
	float size = 0.0f;
	dcmIIobj.get_actuator_positions(result, size);
	// for (int i = 0; i < size; i++) {
	// 	std::cout << i << "   " << result[i] << std::endl;
	// }
	// 
	// std::cout << "size: " << size << std::endl;

	//std::cout <<"Hello World!" <<std::endl;

	return 0;
}
