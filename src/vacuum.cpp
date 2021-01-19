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
            state(State::CHARGING),
            started(false),
            power(100)
        {
        }

        bool start_callback(std_srvs::Empty::Request &, std_srvs::Empty::Response &) 
        {
            started = true;
            return true;
        }

        void timer_callback(const ros::TimerEvent &)
        {
            switch(state)
            {
                case State::CHARGING:
                    ROS_INFO_STREAM("Charging: " << power);
                    if(started && power > 75)
                    {
                        state = State::VACUUMING;
                    }

                    // technically power could be another state
                    if(power < 100)
                    {
                        power += 10;
                    }
                    break;
                case State::VACUUMING:
                    ROS_INFO_STREAM("Vacuuming: " << power);
                    power -= 10;
                    if(power < 25)
                    {
                        state = State::RETURNING;
                    }
                    break;
                case State::RETURNING:
                    ROS_INFO_STREAM("Returning: " << power);
                    power -= 5;
                    if(power == 10)
                    {
                        state = State::CHARGING;
                    }
                    break;
                default:
                    throw std::logic_error("Invalid State");
            }
            // reset the events
            started = false;
        }
    private:
        ros::NodeHandle nh;
        ros::ServiceServer start;
        ros::Timer loop;
        enum class State {CHARGING, VACUUMING, RETURNING} state;
        bool started;
        int power;
};

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "vacuum");
    const Vacuum v;
    ros::spin();
    return 0;
}
