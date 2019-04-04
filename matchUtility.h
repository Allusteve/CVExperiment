#pragma once
#include <opencv2/imgproc.hpp>

class MatchUtility{
public:
    // 内部使用的matcher为BruteForce
    // 在原始的knnMatch进行的过程中，可能会发生错误的匹配，主要有两种类型：匹配的特征点是错误的；某些特征点无法匹配
    static void knnMatch(cv::InputArray queryDesc, cv::InputArray trainDesc,std::vector<cv::DMatch>& matches);

    // 内部调用了findHomography()并指定了method为CV_RANSAC，由此利用到了threshold（threshold为允许的最大反投影错误，仅在使用RANSAC时有效）
    // mask指出匹配的点是否为离群值，用于优化匹配结果
    // RANSAC方法计算基础矩阵，并细化匹配结果
    static void ransac(const std::vector<cv::DMatch> &matches,const std::vector<cv::KeyPoint> &kp1, const std::vector<cv::KeyPoint> &kp2,
                       std::vector<char>& mask, int threshold=5);
};
