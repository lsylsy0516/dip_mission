#include "detector_node.hpp"
#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "utils/utils.h"

// 初始化类
Detector::Detector(int argc, char** argv) {

    // 读取参数
    h_lower = nh.param<int>("h_lower", 0);
    s_lower = nh.param<int>("s_lower", 0);
    v_lower = nh.param<int>("v_lower", 0);
    s_upper = nh.param<int>("s_upper", 0);
    v_upper = nh.param<int>("v_upper", 0);
    pile_h_upper = nh.param<int>("pile_h_upper", 0);
    nurse_h_upper = nh.param<int>("nurse_h_upper", 0);

    nurse_threshold = nh.param<double>("nurse_threshold", 0);
    nurse_scaleStep = nh.param<double>("nurse_scaleStep", 0);
    nurse_minScale = nh.param<double>("nurse_minScale", 0);
    nurse_maxScale = nh.param<double>("nurse_maxScale", 0);

    pile_threshold = nh.param<double>("pile_threshold", 0);
    pile_scaleStep = nh.param<double>("pile_scaleStep", 0);
    pile_minScale = nh.param<double>("pile_minScale", 0);
    pile_maxScale = nh.param<double>("pile_maxScale", 0);

    camera_index = nh.param<int>("camera_index", 0);
    camera_width = nh.param<int>("camera_width", 0);
    camera_height = nh.param<int>("camera_height", 0);

    video_dir_path = nh.param<std::string>("video_dir_path", "");
    nurse_path = nh.param<std::string>("nurse_path", "");
    pile_path = nh.param<std::string>("pile_path", "");

    rect_pub = nh.advertise<detector::Rect>("rect", 10); 
    taskUpdate_sub = nh.subscribe("taskUpdate", 10, &Detector::taskUpdateCallback, this);

    if (argc > 1) {
        const std::string video_filename = argv[1];
        const std::string video_path = video_dir_path + video_filename;
        cap.open(video_path);
    } else {
        cap.open(camera_index);
    }

    if (!cap.isOpened()) {
        ROS_ERROR("Error opening video stream or file.");
        ros::shutdown();
    }

    task = task_object::pile ;  
}

Detector::~Detector() {
    // Release the VideoCapture object
    cap.release();
}

void Detector::run(int argc) {
    while (ros::ok()) {
        cv::Mat frame;
        cap >> frame;

        if (argc == 1) // 使用摄像头时，需要将图像分成两半，只使用左半边
            frame = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));
        
        // Resize the frame
        cv::Size size = cv::Size(camera_width, camera_height);
        cv::resize(frame, frame, size);

        if (frame.empty()) {
            ROS_INFO("End of video stream.");
            break;
        }
        processFrame(frame);
        int key = cv::waitKey(10);
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
    else{   // turn right or left ,don't need to detect
        ROS_INFO("turn right or left ,don't need to detect");
        return;
    }

    // Perform closing operation on the mask
    int closing_size = 5; // Adjust the kernel size as needed
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(closing_size, closing_size));
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
    
    // // Show the frames
    // cv::imshow("origin", frame);
    // cv::imshow("mask", mask);
    // cv::waitKey(1000);
    
    if (task == task_object::nurse) {   // nurse
        const std::string template_path = nurse_path;
        Rects = adaptive_match(mask,template_path,nurse_threshold,nurse_scaleStep,nurse_minScale,nurse_maxScale,task);
    } else if (task == task_object::pile) { // pile
        const std::string template_path = pile_path;
        Rects = adaptive_match(mask,template_path,pile_threshold,pile_scaleStep,pile_minScale,pile_maxScale,task);
    } 
    
    for (auto rect : Rects) {
        rect_msg.xs.push_back(rect.x);
        rect_msg.ys.push_back(rect.y);
        rect_msg.widths.push_back(rect.width);
        rect_msg.heights.push_back(rect.height);
    }

    if (Rects.size() == 0) {
        ROS_INFO("No target detected.");
    } else {
        rect_pub.publish(rect_msg);    // Publish the message
        rect_msg.xs.clear();
        rect_msg.ys.clear();
        rect_msg.widths.clear();
        rect_msg.heights.clear();
    }

    
}

void Detector::taskUpdateCallback(const std_msgs::Int8::ConstPtr& msg) {
    task = msg->data;
    ROS_INFO("task updated to %d", task);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "detector_node");
    ROS_INFO("detector_node inited");
    Detector detector(argc, argv);
    detector.run(argc);
    return 0;
}