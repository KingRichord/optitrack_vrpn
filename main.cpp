#include "optitrack.h"
#include <iostream>
#include <thread>
void test(){

    optitrack::optitrack_manager optitrackManager;
}

int main()
{
	std::cout <<"node start "<<std::endl;
    std::thread a =std::thread(&test);
    std::cout <<"jion"<<std::endl;
    while(1);
    a.join();
	return 0;
}
