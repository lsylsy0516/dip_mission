#include "detector_node.hpp"
#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "utils/utils.h"

// 初始化类
Detector::Detector(int argc, char** argv) {

    if (argc > 1) {
        const std::string video_filename = argv[1];
        const std::string video_path = "/home/luo/Documents/dip_ws/bags/" + video_filename;
        cap.open(video_path);
    } else {
        cap.open(0);
    }
    if (!cap.isOpened()) {
        ROS_ERROR("Error opening video stream or file.");
        ros::shutdown();
    }
    task = task_object::nurse ;  // 初始化任务识别为消防栓

    // 读取参数
    int h_lower = nh.param<int>("h_lower", 0);
    int s_lower = nh.param<int>("s_lower", 0);
    int v_lower = nh.param<int>("v_lower", 0);
    int h_upper = nh.param<int>("h_upper", 0);
    int s_upper = nh.param<int>("s_upper", 0);
    int v_upper = nh.param<int>("v_upper", 0);
    lower_red = cv::Scalar(h_lower, s_lower, v_lower);
    upper_red = cv::Scalar(h_upper, s_upper, v_upper);

    // adpative_match 参数
    threshold = nh.param<double>("threshold", 0.75);
    scaleStep = nh.param<double>("scaleStep", 0.4);
    minScale = nh.param<double>("minScale", 0.4);
    maxScale = nh.param<double>("maxScale", 1.6);
}

Detector::~Detector() {
    // Release the VideoCapture object
    cap.release();
}

void Detector::processFrame(const cv::Mat& frame) {
    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    // Define the range for red color in HSV


    // Create a binary mask for the red color
    cv::Mat mask;
    cv::inRange(hsv, lower_red, upper_red, mask);

    // Perform closing operation on the mask
    int closing_size = 5; // Adjust the kernel size as needed
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(closing_size, closing_size));
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
    
    // cv::imshow("origin", frame);
    // cv::imshow("mask",mask);
    if (task == task_object::nurse) {
        // nurse
        // std::cout<<"nurse"<<std::endl;
        const std::string template_path = "/home/luo/Documents/dip_ws/src/detector/pics/nurse.jpg";
        adaptive_match(mask,template_path,threshold,scaleStep,minScale,maxScale);
    } else if (task == task_object::pile) {
        // pile
        // std::cout<<"pile"<<std::endl;
        const std::string template_path = "/home/luo/Documents/dip_ws/src/detector/pics/pile.jpg";
        adaptive_match(mask,template_path,threshold,scaleStep,minScale,maxScale);
    } 
    
}

void Detector::run() {
    while (ros::ok()) {
        cv::Mat frame;
        cap >> frame;

        if (frame.empty()) {
            ROS_INFO("End of video stream.");
            break;
        }
        processFrame(frame);
        int key = cv::waitKey(1);
        if (key == 'q' || key == 27) {  // 'q' or Esc 
            break;
        }

        ros::spinOnce();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "detector_node");
    ROS_INFO("detector_node inited");
    Detector detector(argc, argv);
    detector.run();
    return 0;
}