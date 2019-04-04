#include "matchUtility.h"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d.hpp"


void MatchUtility::knnMatch(cv::InputArray queryDesc, cv::InputArray trainDesc,std::vector<cv::DMatch>& matches){
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce");
    std::vector<std::vector<cv::DMatch>> knnMatches;

    // KNN中的K设为2时，对每个匹配返回两个最近邻描述符
    matcher->knnMatch(queryDesc,trainDesc,knnMatches,2);

    matches.clear();
    matches.reserve(knnMatches.size());

    // 剔除错误匹配的方法有交叉过滤和比率测试两种，以下使用后者
    // 仅当第一个和第二个匹配之间的距离足够小时，才认为其是一个正确的匹配
    // 以下阈值来自网络
    const float minRatio = 1.0f / 1.5f;
    for(unsigned i = 0; i < knnMatches.size(); i++){
        if(knnMatches[i][0].distance / knnMatches[i][1].distance < minRatio)
            matches.push_back(knnMatches[i][0]);
    }
}

void MatchUtility::ransac(const std::vector<cv::DMatch> &matches, const std::vector<cv::KeyPoint> &kp1, const std::vector<cv::KeyPoint> &kp2,
                          std::vector<char>& mask, int threshold){
    std::vector<int> queryIdxs(matches.size());
    std::vector<int> trainIdxs(matches.size());
    for (size_t i = 0; i < matches.size(); i++){
        queryIdxs[i] = matches[i].queryIdx;
        trainIdxs[i] = matches[i].trainIdx;
    }

    // 将特征点转换为二维的点，convert()的第三个参数用于指明只将特定的点进行转换
    std::vector<cv::Point2f> points1;
    cv::KeyPoint::convert(kp1, points1, queryIdxs);
    std::vector<cv::Point2f> points2;
    cv::KeyPoint::convert(kp2, points2, trainIdxs);

    // 获取两个平面间的投影变换矩阵（也称为单应矩阵）
    cv::Mat homography = cv::findHomography(cv::Mat(points1), cv::Mat(points2), CV_RANSAC, threshold);

    // 简而言之，mask用于记录inlier
    mask.resize(matches.size());

    cv::Mat points;
    cv::perspectiveTransform(cv::Mat(points1), points, homography);
    for (unsigned i = 0; i < points1.size(); ++i){
        // 不超过阈值的即视为内点
        if (norm(points2[i] - points.at<cv::Point2f>(i, 0)) <= threshold)
            mask[i] = 1;
    }
}
