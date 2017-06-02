#include <iostream>

#include "../bhuman.h"
#include "../bhuman.cpp"
 
int main()
{
  std::cout << "Hello World!" << std::endl;

  boost::shared_ptr<AL::ALBroker> pB = boost::shared_ptr<AL::ALBroker>();

  BHuman bh(pB);

  // std::cout << sizeof(bh) << std::endl;

  LBHData lbh;
  std::cout << sizeof(lbh) << std::endl;



  std::cout << "Goodbye World!" << std::endl;
  return 0;
}