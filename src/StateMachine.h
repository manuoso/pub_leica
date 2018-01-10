

//
//
//
//
//

#include <ros/ros.h>
#include "LogManager.h"
#include <mutex>
#include <thread>
#include <std_msgs/Float32MultiArray.h>
#include <ros/ros.h>

class StateMachine
{
  public:
    bool init(int _argc, char** _argv);

    bool run();

  private:

    int initSocket(int _port);
    int split(std::string message, int len, std::vector<std::vector<std::string>> &medidas);
    bool leicaPosEstimate();
    void leicaListenCallback();

  private:

    ros::Publisher mPubFinalPosition;
    std::thread mLeicaListeningThread;
    float mCurrentX = 0, mCurrentY = 0, mCurrentZ = 0;
    float mLastLeicaX = 0, mLastLeicaY = 0, mLastLeicaZ = 0;
    std::mutex mSecureLeica;
};
