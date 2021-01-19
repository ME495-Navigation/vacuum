/// \brief A node that simulates a robot vacuum cleaner
/// 
/// Services:
///  start - (std_srvs/Empty) start the vacuum cleaner if it is not charging
#include"ros/ros.h"
#include"std_srvs/Empty.h"


int main(int argc, char * argv[])
{
    ros::init(argc, argv, "vacuum");
    return 0;
}
