//
//
//
//
//

#include <algorithm>    // std::fill
#include <iostream>
#include <string>
#include <fstream>
#include "StateMachine.h"

int main(int _argc, char **_argv) {

    ros::init(_argc, _argv, "publisher_leica");
    std::thread spinThread = std::thread([&](){
        ros::spin();
    });
    
    StateMachine stateMachine;
    if(!stateMachine.init(_argc, _argv)){
        std::cout << "Error initializing the application" << std::endl;
        return -1;
    }

	while(ros::ok()){
        stateMachine.run();
	}
}
