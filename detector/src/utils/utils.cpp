/**
    * @file utils.cpp
    * @brief Utility functions for the detector
    */

#include "utils.h"

std::vector<cv::Rect> removeFullyCoveredRectangles(const std::vector<cv::Rect>& input) {
    
    int width_thre = ros::param::param<int>("pile_width", 0) * 0.5;
    int x1_ther = ros::param::param<int>("camera_width", 0) / 5;
    int x2_ther = ros::param::param<int>("camera_width", 0) * 4 / 5;
    
    std::vector<cv::Rect> output;

    for (const auto& rect : input) {
        bool isFullyCovered = false;

        for (const auto& otherRect : input) {
            if (rect != otherRect) {
                if (rect.x >= otherRect.x && rect.y >= otherRect.y &&
                    rect.x + rect.width <= otherRect.x + otherRect.width &&
                    rect.y + rect.height <= otherRect.y + otherRect.height) {
                    isFullyCovered = true;
                    break;
                }
            }
            // 若rect 很小，且和其他rect的重合率超过自己的60%，则认为是噪声
            if (rect.width <= width_thre && rect != otherRect) {
                int x1 = std::max(rect.x, otherRect.x);
                int y1 = std::max(rect.y, otherRect.y);
                int x2 = std::min(rect.x + rect.width, otherRect.x + otherRect.width);
                int y2 = std::min(rect.y + rect.height, otherRect.y + otherRect.height);
                int area = (x2 - x1) * (y2 - y1);
                int area1 = rect.width * rect.height;
                if (area > 0.8 * area1) {
                    isFullyCovered = true;
                    break;
                }
            }
        }
        // 若rect 很小且出现在图像的左右边缘，则认为是噪声
        if (rect.width <= width_thre && (rect.x < x1_ther || rect.x > x2_ther)) {
            isFullyCovered = true;
        }

        if (!isFullyCovered) {
            output.push_back(rect);
        }
    }

    return output;
}

// 使用欧式聚类算法，对二值图像进行聚类，返回聚类中心
// 使用pcl::EuclideanClusterExtraction
std::vector<cv::Point> Cluster(const cv::Mat& frame){
    // 将二值图像转换为点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    int rows = frame.rows;
    int cols = frame.cols;
    int count = 0;
    for(int i=0;i<rows;i++){
        for(int j=0;j<cols;j++){
            if(frame.at<float>(i,j)>0){
                count++;
                pcl::PointXYZ point;
                point.x = i;
                point.y = j;
                point.z = 0;
                cloud->points.push_back(point);
            }
        }
    }
    // std::cout<<"count: "<<count<<std::endl;

    // 聚类
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud); // 设置搜索空间

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setInputCloud(cloud); 
    ec.setClusterTolerance(10); // 设置聚类的最大距离
    ec.setMinClusterSize(20);   // 设置聚类的最小点数
    ec.setMaxClusterSize(10000);    // 设置聚类的最大点数
    ec.setSearchMethod(tree);
    ec.extract(cluster_indices);

    // 显示聚类结果
    cv::Mat cluster_result = cv::Mat::zeros(frame.size(),CV_8UC1);
    for(auto it=cluster_indices.begin();it!=cluster_indices.end();it++){
        for(auto pit=it->indices.begin();pit!=it->indices.end();pit++){
            int row = cloud->points[*pit].x;
            int col = cloud->points[*pit].y;
            cluster_result.at<uchar>(row,col) = 255;
        }
    }

    // 计算聚类中心并返回
    std::vector<cv::Point> centers;
    for(auto it=cluster_indices.begin();it!=cluster_indices.end();it++){
        int sum_x = 0;
        int sum_y = 0;
        int count = 0;
        for(auto pit=it->indices.begin();pit!=it->indices.end();pit++){
            sum_x += cloud->points[*pit].x;
            sum_y += cloud->points[*pit].y;
            count++;
        }
        int center_x = sum_x/count;
        int center_y = sum_y/count;
        // cv::Point center(center_x,center_y);
        cv::Point center(center_y,center_x);
        // 我阐释你的梦
        centers.push_back(center);
    }

    cluster_result = cv::Mat::zeros(frame.size(),CV_8UC1);
    return centers;
}

std::vector<cv::Rect> adaptive_match(const cv::Mat& frame, const std::string& template_path,double threshold,double scaleStep,double minScale,double maxScale,int task) {

    cv::Mat result;
    cv::Mat templateImage = cv::imread(template_path);
    cv::cvtColor(templateImage, templateImage, cv::COLOR_BGR2GRAY);
    int matchMethod = cv::TM_CCOEFF_NORMED; 

    // cv::imshow("templateImage",templateImage);
    // cv::waitKey(1000);
    // 用于存储所有匹配结果的容器
    std::vector<cv::Rect> matches;

    // 循环尺度变化
    for (double scale = minScale; scale <= maxScale; scale += scaleStep) {
        cv::Mat scaledTemplate;
        cv::resize(templateImage, scaledTemplate, cv::Size(), scale, scale);
        cv::matchTemplate(frame, scaledTemplate, result, matchMethod);
        cv::threshold(result, result, threshold, 1.0, cv::THRESH_TOZERO);
        cv::Mat binaryResult;
        binaryResult.create(result.size(), CV_8UC1);
        cv::threshold(result, binaryResult, threshold, 1.0, cv::THRESH_BINARY);

        int flag = cv::countNonZero(binaryResult);
        if (flag) {
            // 进行聚类
            std::vector<cv::Point> centers = Cluster(binaryResult);

            // 将聚类中心转换为匹配结果
            for (const auto& center : centers) {
                int centerX = static_cast<int>(center.x );
                int centerY = static_cast<int>(center.y );
                int width = scaledTemplate.cols;
                int height = scaledTemplate.rows;
                // if (width < 80) {
                    // continue;
                // }
                cv::Rect matchRect(centerX, centerY, width, height);
                matches.push_back(matchRect);
            }

        } else {
            // std::cout << "No points with value 1 in binaryResult. Skipping clustering." << std::endl;
        }
    }

    cv::Mat resultImage = frame.clone();
    matches = removeFullyCoveredRectangles(matches);

    if (task == 0 && matches.size()>1){
        // cv::Mat beforefind = frame.clone();
        // for (const auto& match : matches) {
        //     cv::rectangle(beforefind, match, cv::Scalar(100), 2); 
        // }
        // cv::imshow("beforefind", beforefind);
        matches = findBestMatch(matches,frame,templateImage);
    }

    for (const auto& match : matches) {
        cv::rectangle(resultImage, match, cv::Scalar(100), 2); 
    }
    cv::imshow("Matching Result", resultImage);

    return matches;
}

std::vector<cv::Rect> findBestMatch(std::vector<cv::Rect> matches, const cv::Mat frame, const cv::Mat templateImage)
{
    double maxSimilarity = -1.0;
    cv::Rect bestMatchRect;

    for (const auto& match : matches) {
        // Extract ROI from frame using the current match
        cv::Mat roi = frame(match);
        cv::resize(roi, roi, templateImage.size());
        cv::Mat result;
        cv::matchTemplate(roi, templateImage, result, cv::TM_CCOEFF_NORMED);

        // Find the maximum similarity score and its location
        double minVal, maxVal;
        cv::Point minLoc, maxLoc;
        cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc);
        ROS_INFO("maxVal: %f",maxVal);
        // Update the best match if the current match has higher similarity
        if (maxVal > maxSimilarity) {
            maxSimilarity = maxVal;
            bestMatchRect = match;
        }
    }
    
    ROS_INFO("maxSimilarity: %f",maxSimilarity);
    std::vector<cv::Rect> bestMatch;
    bestMatch.push_back(bestMatchRect);
    return bestMatch;
}

