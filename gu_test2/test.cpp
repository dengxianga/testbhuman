#include "gup_standalone.h"
//#include "MotionCombinator.h"

#include <iostream>

int main() {
	GetUpEngine gup;
	gup.generateLuaMof(Mof::frontFast);

	std::cout << "hello world" << std::endl;
}