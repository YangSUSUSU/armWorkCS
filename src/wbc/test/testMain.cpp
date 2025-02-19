
#include <iostream>  
#include "../robotData/robotData.h"
int main(int argc, char* argv[]) {

    RobotData& robotData = RobotData::getInstance();
    std::cout<<"==build ok=="<<std::endl;
    return 0;
}