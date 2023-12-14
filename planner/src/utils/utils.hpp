/**
 * @file utils.hpp
 * @brief Utility functions for the planner.
 */

#pragma once
#include "ros/ros.h"
#include "opencv2/opencv.hpp"

std::vector<cv::Point> getRealPosition(std::vector<cv::Rect>& Rects);

void drawRealPosition(std::vector<cv::Point>& points,int width, int height);