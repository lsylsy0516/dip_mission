#include "ros/ros.h"
#include "planner/Rect.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int8.h"
#include "utils/utils.hpp"

class GlobalPlanner {
public:
    GlobalPlanner(int argc, char** argv);
    ~GlobalPlanner();

    void run();
private:
    int task;      // 0 for nurse, 1 for pile , 2 for turn left, 3 for turn right
    int left_or_right; // 0 for left, 1 for right
    int taskFinishFlag;
    cv::Point left_point;
    cv::Point right_point;


    ros::NodeHandle nh;
    ros::Subscriber rect_sub;
    ros::Subscriber task_finish_sub;
    ros::Publisher task_pub;


    planner::Rect rect_msg;
    std_msgs::Int8 task_msg;
    void rectCallback(const planner::Rect::ConstPtr& msg);
    void taskFinishCallback(const std_msgs::Int8::ConstPtr& msg);
    void setTask(std::vector<cv::Point>& points);
    void setTask();

};

enum task_object{
    nurse=0,
    pile,
    turn_left,
    turn_right
};