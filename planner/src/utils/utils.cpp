/**
 * @file utils.cpp
 * @brief Utility functions for the planner
 */

#include "utils.hpp"

std::vector<cv::Point> getRealPosition(std::vector<cv::Rect>& Rects){
    std::vector<cv::Point> realPosition;
    for(const auto& rect:Rects){
        cv::Point point;
        // get the center of the rect
        point.x = rect.x + rect.width/2;
        point.y = rect.y + rect.height/2;
        // ROS_INFO("Rect: x=%d, y=%d,width=%d ", point.x, point.y, rect.width);
        // get the real position
        // 图像尺寸
        int camera_width = ros::param::param<int>("camera_width", 0);
        int camera_height = ros::param::param<int>("camera_height", 0);

        point.x = (float(point.x - camera_width/2) * 200 *(camera_height/rect.height) / camera_width); // 归一化后的x坐标，范围[-10,10]

        point.x += 5 ; //左边的相机，所以需要一个偏移量

        // 假设距离1m时，矩形的高度为height/2像素，距离越远，矩形越小，简化为反比例关系
        point.y = float(camera_height / 0.02)/ rect.height ; // 归一化后的y坐标
        realPosition.push_back(point);
        
        // ROS_INFO("Real position: x=%d, y=%d,width=%d ", point.x, point.y, rect.width);
    
    }
    int width = ros::param::param<int>("pic_width", 0);
    int height = ros::param::param<int>("pic_height", 0);
    drawRealPosition(realPosition,width,height);
    return realPosition;
}

void drawRealPosition(std::vector<cv::Point>& points,int width, int height){
    cv::Mat img = cv::Mat::zeros(height, width, CV_8UC3);
    // 假设机器人在()处，绘制坐标系
    cv::Point origin(width/2,height);
    cv::Point x_axis(width,height);
    cv::Point y_axis(width/2,0);
    cv::line(img, origin, x_axis, cv::Scalar(0, 0, 255), 2);
    cv::line(img, origin, y_axis, cv::Scalar(0, 0, 255), 2);
    // 绘制目标点
    for(const auto& point:points){
        cv::Point target(point.x+width/2,height-point.y);
        cv::circle(img, target, 5, cv::Scalar(0, 255, 0), -1);
    }

    // 绘制左中右三个点
    std::vector<cv::Point> LCR_point = get_LCR_point(points);
    cv::Point left_point = LCR_point[0];
    cv::Point center_point = LCR_point[1];
    cv::Point right_point = LCR_point[2];


    left_point = cv::Point(left_point.x+width/2,height-left_point.y);
    right_point = cv::Point(right_point.x+width/2,height-right_point.y);
    center_point = cv::Point(center_point.x+width/2,height-center_point.y);

    cv::circle(img,left_point,5,cv::Scalar(255,255,0),-1);
    cv::circle(img,right_point,5,cv::Scalar(255,255,0),-1);
    cv::circle(img,center_point,5,cv::Scalar(255,255,0),-1);

    cv::imshow("real position", img);
    int key = cv::waitKey(1);
    if (key == 'q' || key == 27) {  // 'q' or Esc 
        return;
    }
}

std::vector<cv::Point> get_LCR_point(std::vector<cv::Point>& points){
    cv::Point left_point   = cv::Point(1000, 1000);
    cv::Point right_point  = cv::Point(1000, 1000);
    cv::Point center_point = cv::Point(1000, 1000);

    int center_thre = ros::param::param<int>("center_thre", 0);
    int left_thre = ros::param::param<int>("left_thre", 0);
    int right_thre = ros::param::param<int>("right_thre", 0);

    for (const auto& point:points)
    {
        if (point.x < -left_thre)
        {
            if(left_point.y>point.y) // 找到最近的点
            {
                left_point = point;
            }
        }
        else if (point.x > right_thre)
        {
            if(right_point.y>point.y)
            {
                right_point = point;
            }
        }
        
        if (abs(point.x)<center_thre){
            if (abs(point.x)<abs(center_point.x))
                {
                    center_point = point;
                }
        }
    }

    if (center_point.x==1000)
    {
        center_point = cv::Point(0,250); // 为找到中心点，假设在中间
    }
    
    std::vector<cv::Point> LCR_point;
    LCR_point.push_back(left_point);
    LCR_point.push_back(center_point);
    LCR_point.push_back(right_point);
    return LCR_point;
}