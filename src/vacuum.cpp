/// \brief A node that simulates a robot vacuum cleaner
/// 
/// Services:
///  start - (std_srvs/Empty) start the vacuum cleaner if it is not charging
#include"ros/ros.h"
#include"std_srvs/Empty.h"

class Vacuum
{
    public:
        Vacuum():
            nh(),
            start(nh.advertiseService("start", &Vacuum::start_callback, this))
        {
            ROS_INFO_STREAM("HELLO");
        }

        bool start_callback(std_srvs::Empty::Request &, std_srvs::Empty::Response &) 
        {
            ROS_INFO_STREAM("Starting!");
            return true;
        }

    private:
        ros::NodeHandle nh;
        ros::ServiceServer start;
};

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "vacuum");

    ros::NodeHandle nh;
    const Vacuum v;
    ros::spin();
    return 0;
}
