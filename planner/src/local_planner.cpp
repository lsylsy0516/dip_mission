#include "ros/ros.h"
#include "local_planner.hpp"
#include "opencv2/opencv.hpp"
#include "utils/utils.hpp"

Planner::Planner(int argc, char** argv)
{   
    ROS_INFO("Planner node is starting");

    task = task_object::pile;
    left_angular_vel = ros::param::param<int>("left_angular_vel", 90);
    left_turn_time = ros::param::param<int>("left_turn_time", 33);
    right_angular_vel = ros::param::param<int>("right_angular_vel", -90);
    right_turn_time = ros::param::param<int>("right_turn_time", 33);
    control_freq = ros::param::param<int>("control_freq", 1);

    rect_sub = nh.subscribe("/rect", 1, &Planner::rectCallback, this);
    
    std::string vel_topic = nh.param<std::string>("vel_topic", "/cmd_vel");
    vel_pub = nh.advertise<geometry_msgs::Twist>(vel_topic, 1);
    task_update_sub = nh.subscribe("/taskUpdate", 1, &Planner::taskUpdateCallback, this);
}

Planner::~Planner()
{
    ROS_INFO("Planner node is shutting down");
}

void Planner::run()
{
    ros::Rate loop_rate(control_freq);
    while (ros::ok())
    {
        vel_pub.publish(vel_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void Planner::rectCallback(const planner::Rect::ConstPtr& msg)
{
    ROS_INFO("-----------------------------------");
    ROS_INFO("Received rect message");
    rect_msg = *msg;
    std::vector<cv::Rect> Rects;
    for (int i = 0; i < rect_msg.xs.size(); i++)
    {
        Rects.push_back(cv::Rect(rect_msg.xs[i], rect_msg.ys[i], rect_msg.widths[i], rect_msg.heights[i]));
    }
    std::vector<cv::Point> points = getRealPosition(Rects);
    setVelocity(points);
}

void Planner::setVelocity(std::vector<cv::Point>& points)
{
    if (task == task_object::pile){
        std::vector<cv::Point> LCR_point = get_LCR_point(points);
        cv::Point left_point = LCR_point[0];
        cv::Point center_point = LCR_point[1];
        cv::Point right_point = LCR_point[2];

        ROS_INFO("left_point: (%d, %d)", left_point.x, left_point.y);
        ROS_INFO("right_point: (%d, %d)", right_point.x, right_point.y);
        ROS_INFO("center_point: (%d, %d)", center_point.x, center_point.y);

        double vel_coef = nh.param<double>("vel_coef", 0.0);
        double ang_coef = nh.param<double>("ang_coef", 0.0);

        // set the velocity
        if (center_point.y > 100) // use default velocity
            vel_msg.linear.x = nh.param<double>("vel_default", 0.0);
        else if(center_point.y > 52)
            vel_msg.linear.x = vel_coef*(center_point.y-55);
        else
            vel_msg.linear.x = 0.01;

        // set the angular velocity
        int x_sum = left_point.x + right_point.x;
        vel_msg.angular.z = ang_coef* (x_sum);
    }
    
    else if (task ==task_object::nurse)
    {
        // 只会返回一个rect
        cv::Point point = points[0];
        // set the velocity
        if (point.y > 10) // use default velocity
            {
            double t = nh.param<double>("vel", 0.0);
            ROS_INFO("t=%f", t);
            vel_msg.linear.x = t;}
        else{
            double t =  0.1+0.1*point.y;
            ROS_INFO("t=%f", t);
            vel_msg.linear.x = t;}
        
        // set the angular velocity
        float pre = nh.param<double>("nurse_coef", 0.0);
        vel_msg.angular.z = 0.05 * (point.x-pre);
    }

    ROS_INFO("Velocity: linear.x=%f, angular.z=%f", vel_msg.linear.x, vel_msg.angular.z);
}

void Planner::taskUpdateCallback(const std_msgs::Int8::ConstPtr& msg)
{
    task = msg->data;
    ROS_INFO("Task is updated to %d", task);
    if (task == task_object::turn_left)
    {
        TurnLeft();
    }
    else if (task == task_object::turn_right)
    {
        TurnRight();
    }
}

void Planner::TurnLeft()
{
    vel_msg.linear.x = 0.0;
    vel_msg.angular.z = left_angular_vel;
    ros::Rate loop_rate(10);
    for (int i = 0; i < left_turn_time; i++)
    {
        vel_pub.publish(vel_msg);
        ros::spinOnce();
        loop_rate.sleep();
        ROS_INFO("Turning left");
    }

    ros::Publisher finish_pub = nh.advertise<std_msgs::Int8>("/taskFinished", 1);
    std_msgs::Int8 finish_msg;
    finish_msg.data = 1;
    finish_pub.publish(finish_msg);
    ROS_INFO("Turn left finished");
}

void Planner::TurnRight()
{
    vel_msg.linear.x = 0.0;
    vel_msg.angular.z = right_angular_vel;
    ros::Rate loop_rate(10);
    for (int i = 0; i < right_turn_time; i++)
    {
        ROS_INFO("Turning right");
        vel_pub.publish(vel_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::Publisher finish_pub = nh.advertise<std_msgs::Int8>("/taskFinished", 1);
    std_msgs::Int8 finish_msg;
    finish_msg.data = task_object::pile;
    finish_pub.publish(finish_msg);
    ROS_INFO("Turn right finished");    
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "local_planner");
    auto planner_node = Planner(argc, argv);
    planner_node.run();
    ros::shutdown();
    return 0;
}

