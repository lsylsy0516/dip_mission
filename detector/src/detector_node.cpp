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

    rect_pub = nh.advertise<detector::Rect>("rect", 10); 
    taskUpdate_sub = nh.subscribe("taskUpdate", 10, &Detector::taskUpdateCallback, this);
    task = task_object::pile ;  // 初始化任务识别为消防栓

    // 读取参数
    h_lower = nh.param<int>("h_lower", 0);
    s_lower = nh.param<int>("s_lower", 0);
    v_lower = nh.param<int>("v_lower", 0);
    pile_h_upper = nh.param<int>("pile_h_upper", 0);
    nurse_h_upper = nh.param<int>("nurse_h_upper", 0);
    s_upper = nh.param<int>("s_upper", 0);
    v_upper = nh.param<int>("v_upper", 0);

    nurse_threshold = nh.param<double>("nurse_threshold", 0);
    nurse_scaleStep = nh.param<double>("nurse_scaleStep", 0);
    nurse_minScale = nh.param<double>("nurse_minScale", 0);
    nurse_maxScale = nh.param<double>("nurse_maxScale", 0);

    pile_threshold = nh.param<double>("pile_threshold", 0);
    pile_scaleStep = nh.param<double>("pile_scaleStep", 0);
    pile_minScale = nh.param<double>("pile_minScale", 0);
    pile_maxScale = nh.param<double>("pile_maxScale", 0);

}

Detector::~Detector() {
    // Release the VideoCapture object
    cap.release();
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

void Detector::processFrame(const cv::Mat& frame) {
    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    cv::Mat mask;   // Create a binary mask for the red color
    
    if (task == task_object::nurse){
        lower_red = cv::Scalar(h_lower, s_lower, v_lower);
        upper_red = cv::Scalar(nurse_h_upper, s_upper, v_upper);
        cv::inRange(hsv, lower_red, upper_red, mask);
    }
    else if (task == task_object::pile){
        lower_red = cv::Scalar(h_lower, s_lower, v_lower);
        upper_red = cv::Scalar(pile_h_upper, s_upper, v_upper);
        cv::inRange(hsv, lower_red, upper_red, mask);
    }

    // Perform closing operation on the mask
    int closing_size = 5; // Adjust the kernel size as needed
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(closing_size, closing_size));
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
    
    // // Show the frames
    // cv::imshow("origin", frame);
    // cv::imshow("mask",mask);
    
    if (task == task_object::nurse) {   // nurse
        // std::cout<<"nurse"<<std::endl;
        const std::string template_path = "/home/luo/Documents/dip_ws/src/detector/pics/nurse.jpg";
        Rects = adaptive_match(mask,template_path,nurse_threshold,nurse_scaleStep,nurse_minScale,nurse_maxScale);
    } else if (task == task_object::pile) { // pile
        // std::cout<<"pile"<<std::endl;
        const std::string template_path = "/home/luo/Documents/dip_ws/src/detector/pics/pile.jpg";
        Rects = adaptive_match(mask,template_path,pile_threshold,pile_scaleStep,pile_minScale,pile_maxScale);
    } 
    
    for (auto rect : Rects) {
        rect_msg.xs.push_back(rect.x);
        rect_msg.ys.push_back(rect.y);
        rect_msg.widths.push_back(rect.width);
    }


    rect_pub.publish(rect_msg);    // Publish the message

    rect_msg.xs.clear();
    rect_msg.ys.clear();
    rect_msg.widths.clear();
}

void Detector::taskUpdateCallback(const std_msgs::Int8::ConstPtr& msg) {
    task = msg->data;
    ROS_INFO("task updated to %d", task);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "detector_node");
    ROS_INFO("detector_node inited");
    Detector detector(argc, argv);
    detector.run();
    return 0;
}