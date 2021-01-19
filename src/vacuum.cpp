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
            start(nh.advertiseService("start", &Vacuum::start_callback, this)),
            loop(nh.createTimer(ros::Duration(1.0), &Vacuum::timer_callback, this)),
            state(State::CHARGING)
        {
        }

        bool start_callback(std_srvs::Empty::Request &, std_srvs::Empty::Response &) 
        {
            return true;
        }

        void timer_callback(const ros::TimerEvent &)
        {
            switch(state)
            {
                case State::CHARGING:
                    ROS_INFO("Charging");
                    break;
                case State::VACUUMING:
                    ROS_INFO("Vacuuming");
                    break;
                case State::RETURNING:
                    ROS_INFO("Returning");
                    break;
                default:
                    throw std::logic_error("Invalid State");
            }
        }
    private:
        ros::NodeHandle nh;
        ros::ServiceServer start;
        ros::Timer loop;
        enum class State {CHARGING, VACUUMING, RETURNING} state;
};

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "vacuum");
    const Vacuum v;
    ros::spin();
    return 0;
}
