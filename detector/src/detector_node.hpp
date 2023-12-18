#pragma once
#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/extract_clusters.h>
#include "utils/utils.h"
#include "detector/Rect.h"
#include "std_msgs/Int8.h"

class Detector {
public:
    Detector(int argc, char** argv);
    ~Detector();

    void run(int argc);

private:
    int task;      // 0 for nurse, 1 for pile
    int time_thre;
    int count;

    ros::NodeHandle nh;
    ros::Publisher rect_pub;
    ros::Subscriber taskUpdate_sub;
    ros::Publisher finish_pub;

    detector::Rect rect_msg;
    std::vector<cv::Rect> Rects;

    std::string video_dir_path;
    std::string nurse_path;
    std::string pile_path;

    cv::VideoCapture cap ;
    cv::Scalar lower_red ;
    cv::Scalar upper_red ;

    double nurse_threshold;
    double nurse_scaleStep;
    double nurse_minScale;
    double nurse_maxScale;

    double pile_threshold;
    double pile_scaleStep;
    double pile_minScale;
    double pile_maxScale;

    int h_lower;
    int s_lower;
    int v_lower;
    int pile_h_upper;
    int nurse_h_upper;
    int s_upper;
    int v_upper;

    int camera_index;
    int camera_width;
    int camera_height;

    void processFrame(const cv::Mat& frame);
    void taskUpdateCallback(const std_msgs::Int8::ConstPtr& msg);
};



enum task_object{
    nurse=0,
    pile,
    turn_left,
    turn_right
};