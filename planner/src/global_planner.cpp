#include "ros/ros.h"
#include "global_planner.hpp"
#include "opencv2/opencv.hpp"
#include "utils/utils.hpp"

GlobalPlanner::GlobalPlanner(int argc, char **argv)
{
    ROS_INFO("Global planner node is starting");
    rect_sub = nh.subscribe("/rect", 1, &GlobalPlanner::rectCallback, this);
    task_finish_sub = nh.subscribe("/taskFinish", 1, &GlobalPlanner::taskFinishCallback, this);
    task_pub = nh.advertise<std_msgs::Int8>("/taskUpdate", 1);
    left_point = cv::Point(0, 1000);
    right_point = cv::Point(0, 1000);
    task = task_object::pile;
    taskFinishFlag = 1; // finished

    int task_first = task_object::pile;
    task_msg.data = task_first;
    task_pub.publish(task_msg);
}

GlobalPlanner::~GlobalPlanner()
{
    ROS_INFO("Global planner node is shutting down");
}

void GlobalPlanner::run()
{
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void GlobalPlanner::rectCallback(const planner::Rect::ConstPtr &msg)
{
    // ROS_INFO("-----------------------------------");
    // ROS_INFO("Received rect message");
    rect_msg = *msg;
    std::vector<cv::Rect> Rects;
    for (int i = 0; i < rect_msg.xs.size(); i++)
    {
        Rects.push_back(cv::Rect(rect_msg.xs[i], rect_msg.ys[i], rect_msg.widths[i], rect_msg.heights[i]));
    }
    std::vector<cv::Point> points = getRealPosition(Rects);
    setTask(points);
}

void GlobalPlanner::setTask(std::vector<cv::Point> &points)
{
    if (task == task_object::nurse)
    {
        // 上一时刻的任务为nurse,则保持不变
        return;
    }
    if (task == task_object::pile)
    {
        // 上一时刻的任务为pile
        // 找到最左边和最右边的点，并更新
        cv::Point center_point = cv::Point(1000, 0);
        for (const auto &point : points)
        {
            if (point.x < left_point.x)
            {
                left_point = point;
            }
            if (point.x > right_point.x)
            {
                right_point = point;
            }
            if (abs(point.x) < abs(center_point.x))
            {
                center_point = point;
            }
        }

        ROS_INFO("center_point y:%d", center_point.y);
        if (abs(left_point.x) > abs(right_point.x))
        {
            left_or_right = 0; // 左边的点更远，说明要向左转
        }
        else
        {
            left_or_right = 1; // 反之往右转
        }

        int dis_to_turn = nh.param<int>("dis_to_turn", 50);
        if (center_point.y < dis_to_turn && taskFinishFlag == 1)
        {
            if (left_or_right == 0)
            {
                task = task_object::turn_left;
            }
            else
            {
                task = task_object::turn_right;
            }
            ROS_INFO("time to turn");
            // 嘘，这里是个小秘密
            task = nh.param<int>("left_or_right", 2);

            task_msg.data = task;
            task_pub.publish(task_msg);
            taskFinishFlag = 0;
        }
    }

    if (task == task_object::turn_left && taskFinishFlag)
    {
        task = task_object::pile;
        task_msg.data = task;
        task_pub.publish(task_msg);
    }
    if (task == task_object::turn_right && taskFinishFlag)
    {
        task = task_object::pile;
        task_msg.data = task;
        task_pub.publish(task_msg);
    }
}

void GlobalPlanner::setTask()
{
    if (task == task_object::turn_left && taskFinishFlag==2)
    {
        task = task_object::pile;
        task_msg.data = task;
        task_pub.publish(task_msg);
    }
    if (task == task_object::turn_right && taskFinishFlag==2)
    {
        task = task_object::pile;
        task_msg.data = task;
        task_pub.publish(task_msg);
    }
    if ( task == task_object::pile && taskFinishFlag==3)
    {
        task = task_object::nurse;
        task_msg.data = task;
        task_pub.publish(task_msg);
    }
}

void GlobalPlanner::taskFinishCallback(const std_msgs::Int8::ConstPtr &msg)
{
    ROS_INFO("-----------------------------------");
    ROS_INFO("Received taskFinish message");
    if (taskFinishFlag ==2)
        taskFinishFlag = 3;
    if (taskFinishFlag == 0)
        taskFinishFlag = 2;
    setTask();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "global_planner");
    GlobalPlanner global_planner(argc, argv);
    global_planner.run();
    return 0;
}