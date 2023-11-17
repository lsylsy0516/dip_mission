#include "ros/ros.h"
#include "planner/Rect.h"
#include "geometry_msgs/Twist.h"

class Planner {
public:
    Planner(int argc, char** argv);
    ~Planner();

    void run();

private:
    ros::NodeHandle nh;
    ros::Subscriber rect_sub;
    ros::Publisher vel_pub;

    geometry_msgs::Twist vel_msg;
    planner::Rect rect_msg;
    void rectCallback(const planner::Rect::ConstPtr& msg);
};