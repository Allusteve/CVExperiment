#pragma once
#include <QImage>
#include <opencv2/imgproc.hpp>

class ImageUtility{
public:
    static void MatToQImage(const cv::Mat &mat,QImage &out);

    // 实际上这个函数并没有被用到，不过顺手写下来了
    static void QImageToMat(const QImage &img,cv::Mat &out);

    // 在图像上画出关键点的梯度方向
    static void drawKeyPoints(const cv::Mat &in,const std::vector<cv::KeyPoint>& kps,cv::Mat &out);

private:
    static void drawSingleKeyPoint(const cv::KeyPoint &kp,const cv::Scalar &color,cv::Mat &out);
};
