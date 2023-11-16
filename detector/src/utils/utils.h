/**
    * @file utils.h
    * @brief Utility functions for the detector.
    */

#pragma once
#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/extract_clusters.h>

/**
    * @brief Remove fully covered rectangles
    * @param input The input vector of rectangles
    * @return The output vector of rectangles
*/
std::vector<cv::Rect>removeFullyCoveredRectangles(const std::vector<cv::Rect>& input);

/**
    * @brief Adaptive template matching
    * @param frame The frame to search in
    * @param template_path The path to the template image
    * @param threshold The threshold for the template matching
    * @param scaleStep The step size for the scale
    * @param minScale The minimum scale
    * @param maxScale The maximum scale
*/
void adaptive_match(const cv::Mat& frame, const std::string& template_path,double threshold,double scaleStep,double minScale,double maxScale);

/**
    * @brief Cluster the points in the frame
    * @param frame The frame to cluster
    * @return The cluster centers
*/
std::vector<cv::Point> Cluster(const cv::Mat& frame);