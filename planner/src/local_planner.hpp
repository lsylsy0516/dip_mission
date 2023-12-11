#include "ros/ros.h"
#include "planner/Rect.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int8.h"
#include "utils/utils.hpp"

class Planner {
public:
    Planner(int argc, char** argv);
    ~Planner();

    void run();

private:
    int task;      // 0 for nurse, 1 for pile

    ros::NodeHandle nh;
    ros::Subscriber rect_sub;
    ros::Publisher vel_pub;
    ros::Subscriber task_update_sub;

    geometry_msgs::Twist vel_msg;
    planner::Rect rect_msg;
    void rectCallback(const planner::Rect::ConstPtr& msg);
    void taskUpdateCallback(const std_msgs::Int8::ConstPtr& msg);
    void setVelocity(std::vector<cv::Point>& points);
    void TurnLeft();
    void TurnRight();
};

enum task_object{
    nurse=0,
    pile,
    turn_left,
    turn_right
};