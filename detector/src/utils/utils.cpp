/**
    * @file utils.cpp
    * @brief Utility functions for the detector
    */

#include "utils.h"

std::vector<cv::Rect> removeFullyCoveredRectangles(const std::vector<cv::Rect>& input) {
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
        }

        if (!isFullyCovered) {
            output.push_back(rect);
        }
    }

    return output;
}

std::vector<cv::Point> Cluster(const cv::Mat& frame){
    // 使用欧式聚类算法，对二值图像进行聚类，返回聚类中心
    // 使用pcl::EuclideanClusterExtraction

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

std::vector<cv::Rect> adaptive_match(const cv::Mat& frame, const std::string& template_path,double threshold,double scaleStep,double minScale,double maxScale) {


    cv::Mat templateImage = cv::imread(template_path);
    cv::cvtColor(templateImage, templateImage, cv::COLOR_BGR2GRAY);
    // cv::imshow("templateImage", templateImage);
    // cv::waitKey(0);
    cv::Mat result;

    int matchMethod = cv::TM_CCOEFF_NORMED; 

    // 用于存储所有匹配结果的容器
    std::vector<cv::Rect> matches;

    // 循环尺度变化
    for (double scale = minScale; scale <= maxScale; scale += scaleStep) {
        // std::cout << "Scale: " << scale << std::endl;
        // 调整模板大小
        cv::Mat scaledTemplate;
        cv::resize(templateImage, scaledTemplate, cv::Size(), scale, scale);
        // 进行模板匹配
        // cv::imshow("scaledTemplate", scaledTemplate);
        // cv::imshow("frame", frame);
        // cv::waitKey(0);
        cv::matchTemplate(frame, scaledTemplate, result, matchMethod);
        // 寻找所有得分大于阈值的匹配
        cv::threshold(result, result, threshold, 1.0, cv::THRESH_TOZERO);
        cv::Mat binaryResult;
        binaryResult.create(result.size(), CV_8UC1);

        cv::threshold(result, binaryResult, threshold, 1.0, cv::THRESH_BINARY);

        // std::string windowName = "result"+std::to_string(scale);
        // cv::imshow (windowName, binaryResult);

        int flag = cv::countNonZero(binaryResult);
        // std::cout << "flag: " << flag << std::endl;
        if (flag) {
            // 进行聚类
            std::vector<cv::Point> centers = Cluster(binaryResult);

            // 将聚类中心转换为匹配结果
            for (const auto& center : centers) {
                int centerX = static_cast<int>(center.x );
                int centerY = static_cast<int>(center.y );
                int width = scaledTemplate.cols;
                int height = scaledTemplate.rows;
                // std::cout << "Match at (" << centerX << ", " << centerY << ")  " << std::endl;
                // std::cout << "Width: " << width << ", Height: " << height << std::endl;
                
                if (width < 80) {
                    // std::cout << "Width or height too large. Skipping." << std::endl;
                    continue;
                }
                
                cv::Rect matchRect(centerX, centerY, width, height);
                matches.push_back(matchRect);
            }

        } else {
            // std::cout << "No points with value 1 in binaryResult. Skipping clustering." << std::endl;
        }
    }

    cv::Mat resultImage = frame.clone();
    matches = removeFullyCoveredRectangles(matches);
    // 绘制所有匹配结果
    for (const auto& match : matches) {
        cv::rectangle(resultImage, match, cv::Scalar(255), 5); // -1表示填充
        
    }
    // 显示结果
    cv::imshow("Matching Result", resultImage);
    return matches;
}