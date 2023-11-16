#pragma once
#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/extract_clusters.h>
#include "utils/utils.h"

class Detector {
public:
    Detector(int argc, char** argv);
    ~Detector();

    void run();

private:
    cv::VideoCapture cap;
    ros::NodeHandle nh;
    int task;      // 0 for nurse, 1 for pile
    cv::Scalar lower_red ;
    cv::Scalar upper_red ;

    double threshold = 0.75;
    double scaleStep = 0.4;
    double minScale = 0.4;
    double maxScale = 1.6;

    void processFrame(const cv::Mat& frame);
};



enum task_object{
    nurse=0,
    pile
};