#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/xfeatures2d/nonfree.hpp"
#include <vector>

#include <QImage>

#ifndef FOO_H
#define FOO_H

class Foo
{
public:
    //@remark 用于画出真正匹配的点对
    static void myDrawMatches(const cv::Mat& in,const std::vector<cv::KeyPoint>& keypoints1,
                              const cv::Mat& in2,const std::vector<cv::KeyPoint>& keypoints2,
                              const std::vector<cv::DMatch>& matches1to2,cv::Mat& out,
                              const std::vector<char>& matchesMask,const cv::Scalar& matchColor = cv::Scalar::all((-1)));

    //@remark 通过图像配准实现拼接
    static void myStitch(const cv::Mat& left_img, const cv::Mat& right_img,
                         cv::Mat& dst,int method);
};


#endif // FOO_H
