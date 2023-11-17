#include "ros/ros.h"
#include "planner_node.hpp"

Planner::Planner(int argc, char** argv)
{   
    ROS_INFO("Planner node is starting");
    rect_sub = nh.subscribe("/rect", 1, &Planner::rectCallback, this);
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
}

Planner::~Planner()
{
    ROS_INFO("Planner node is shutting down");
}

void Planner::run()
{
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void Planner::rectCallback(const planner::Rect::ConstPtr& msg)
{
    ROS_INFO("Received rect message");
    rect_msg = *msg;

    vel_msg.linear.x = 0;
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = 0;
    vel_pub.publish(vel_msg);
}



int main(int argc, char * argv[])
{
    ros::init(argc, argv, "planner_node");
    auto planner_node = Planner(argc, argv);
    planner_node.run();
    ros::shutdown();
    return 0;
}
