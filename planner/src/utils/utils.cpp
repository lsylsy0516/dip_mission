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
        // get the real position
        // 图像尺寸为2208 × 1242
        point.x = float((point.x - 1104)* 5 / rect.width); // target到相机的水平距离

        // 假设距离2m时，矩形的宽度为40像素，距离越远，矩形越小，简化为反比例关系
        point.y = float(2 * 40 * 50 / rect.width ); // target到相机的垂直距离
        realPosition.push_back(point);
        
        ROS_INFO("Real position: x=%d, y=%d,width=%d ", point.x, point.y, rect.width);
    
    }
    drawRealPosition(realPosition);
    return realPosition;
}

void drawRealPosition(std::vector<cv::Point>& points){
    cv::Mat img = cv::Mat::zeros(1000, 1000, CV_8UC3);
    // 假设机器人在(500,1000)处，绘制坐标系
    cv::Point origin(500,1000);
    cv::Point x_axis(1000,1000);
    cv::Point y_axis(500,0);
    cv::line(img, origin, x_axis, cv::Scalar(0, 0, 255), 2);
    cv::line(img, origin, y_axis, cv::Scalar(0, 0, 255), 2);
    // 绘制目标点
    for(const auto& point:points){
        cv::Point target(point.x*10+500,1000-point.y*10);
        cv::circle(img, target, 5, cv::Scalar(0, 255, 0), -1);
    }
    // get the left and right closest point
    cv::Point left_point = cv::Point(100, 100);
    cv::Point right_point = cv::Point(100, 100);
    for (const auto& point:points)
    {
        if (point.x < 0)
        {
            if(left_point.y>point.y)
            {
                left_point = point;
            }
        }
        else
        {
            if(right_point.y>point.y)
            {
                right_point = point;
            }
        }
    }
    left_point = cv::Point(left_point.x*10+500,1000-left_point.y*10);
    right_point = cv::Point(right_point.x*10+500,1000-right_point.y*10);

    cv::circle(img,left_point,5,cv::Scalar(255,255,255),-1);
    cv::circle(img,right_point,5,cv::Scalar(255,255,255),-1);

    cv::imshow("real position", img);
    int key = cv::waitKey(1);
    if (key == 'q' || key == 27) {  // 'q' or Esc 
        return;
    }
}

