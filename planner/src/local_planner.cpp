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
    forward_time = ros::param::param<int>("forward_time", 20);
    forward_vel = ros::param::param<float>("forward_vel", 0);

    control_freq = ros::param::param<int>("control_freq", 10);
    max_angular_vel =  ros::param::param<double>("max_ang_vel", 100);
    max_vel = ros::param::param<double>("max_vel", 100);

    rect_sub = nh.subscribe("/rect", 1, &Planner::rectCallback, this);
    task_update_sub = nh.subscribe("/taskUpdate", 1, &Planner::taskUpdateCallback, this);
    
    std::string vel_topic = nh.param<std::string>("vel_topic", "/cmd_vel");
    vel_pub = nh.advertise<geometry_msgs::Twist>(vel_topic, 1);
    finish_pub = nh.advertise<std_msgs::Int8>("/taskFinish", 1);
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

        // set the angular velocity
        int x_sum = left_point.x + right_point.x;
        int pre = nh.param<int>("ang_pre", 0);
        x_sum -= pre;
        vel_msg.angular.z = ang_coef * (x_sum);

        int dis_to_turn = nh.param<int>("dis_to_turn", 62);
        // set the velocity
        if (center_point.y > 100) // use default velocity
            vel_msg.linear.x = nh.param<double>("vel_default", 0.0);
        else if(center_point.y > dis_to_turn)
            vel_msg.linear.x = vel_coef*(center_point.y-55);
        else{
            vel_msg.linear.x = 0;
            vel_msg.angular.z = 0;
        }
    }
    
    else if (task ==task_object::nurse)
    {
        // 只会返回一个rect
        cv::Point point = points[0];
        ROS_INFO("point: (%d, %d)", point.x, point.y);
        
        // set the velocity
        int dis_thre = nh.param<int>("dis_thre", 0);
        double nurse_coef = nh.param<double>("nurse_coef", 0.0);
        double vel_linear = double(point.y-dis_thre) * nurse_coef;
        if ( vel_linear >max_vel)
            vel_linear = max_vel;
        else if (vel_linear < -max_vel)
            vel_linear = -max_vel; 

        ROS_INFO("vel_linear: %f", vel_linear);
        vel_msg.linear.x = vel_linear;
        
        // set the angular velocity
        float coef = nh.param<double>("nurse_angle_coef", 0.0);
        vel_msg.angular.z = coef * point.x;
    }
    vel_pub.publish(vel_msg);
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
    StepForward();
    vel_msg.linear.x = 0.1;
    vel_msg.angular.z = left_angular_vel;
    ros::Rate loop_rate(10);
    for (int i = 0; i < left_turn_time; i++)
    {
        vel_pub.publish(vel_msg);
        ros::spinOnce();
        loop_rate.sleep();
        ROS_INFO("Turning left");
    }

    std_msgs::Int8 finish_msg;
    finish_msg.data = 1;
    finish_pub.publish(finish_msg);
    ROS_INFO("Turn left finished");
}

void Planner::TurnRight()
{
    StepForward();
    vel_msg.linear.x = 0.1;
    vel_msg.angular.z = right_angular_vel;
    ros::Rate loop_rate(10);
    for (int i = 0; i < right_turn_time; i++)
    {
        ROS_INFO("Turning right");
        vel_pub.publish(vel_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    
    std_msgs::Int8 finish_msg;
    finish_msg.data = task_object::pile;
    finish_pub.publish(finish_msg);
    ROS_INFO("Turn right finished");    
}

void Planner::StepForward()
{
    vel_msg.linear.x = forward_vel;
    vel_msg.angular.z = 0.0;
    ros::Rate loop_rate(10);
    for (int i = 0; i < forward_time; i++)
    {
        ROS_INFO("Step_Forward");
        vel_pub.publish(vel_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "local_planner");
    auto planner_node = Planner(argc, argv);
    planner_node.run();
    ros::shutdown();
    return 0;
}

