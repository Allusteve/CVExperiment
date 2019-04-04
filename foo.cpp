#include "foo.h"
#include <QDebug>
#include "matchUtility.h"

const int draw_shift_bits = 4;
const int draw_multiplier = 1 << draw_shift_bits;

struct Corners
{
    cv::Point2f left_top;
    cv::Point2f left_bottom;
    cv::Point2f right_top;
    cv::Point2f right_bottom;
};

//-----------------------------------------------------
//@remark 计算经过H矩阵投影转换之后的对应图像的四个角的坐标
//@param dst即为目标图像
static void calcNewPosAfterH(const cv::Mat& H, const cv::Mat& src,Corners & corners)
{
    double v2[] = {0,0,1};//原图左上角，注意由于H是3*3矩阵，所以这里采用齐次坐标，第三位1表示是点，若为0说明是向量
    double v1[3]; //目标坐标
    cv::Mat V2 = cv::Mat(3,1,CV_64FC1,v2);// 转为Mat，方便计算
    cv::Mat V1 = cv::Mat(3,1,CV_64FC1,v1);

    V1 = H * V2;// 简单粗暴
    corners.left_top.x = v1[0]/ v1[2]; // 第三位不为1.f，这一次除法我们称之为 nonuniform foreshortening
    corners.left_top.y = v1[1] / v1[2];
    //计算左下角（0，src.cols,1）
    v2[0] = 0;
    v2[1] = src.rows;
    v2[2] = 1;
    V2 = cv::Mat(3,1,CV_64FC1,v2);// 转为Mat，方便计算
    V1 = cv::Mat(3,1,CV_64FC1,v1);
    V1 = H * V2;
    corners.left_bottom.x = v1[0]/ v1[2];
    corners.left_bottom.y = v1[1] / v1[2];

    //右上角(src.cols,0,1)
    v2[0] = src.cols;
    v2[1] = 0;
    v2[2] = 1;
    V2 = cv::Mat(3, 1, CV_64FC1, v2);  //列向量
    V1 = cv::Mat(3, 1, CV_64FC1, v1);  //列向量
    V1 = H * V2;
    corners.right_top.x = v1[0] / v1[2];
    corners.right_top.y = v1[1] / v1[2];

    //右下角(src.cols,src.rows,1)
    v2[0] = src.cols;
    v2[1] = src.rows;
    v2[2] = 1;
    V2 = cv::Mat(3, 1, CV_64FC1, v2);  //列向量
    V1 = cv::Mat(3, 1, CV_64FC1, v1);  //列向量
    V1 = H * V2;
    corners.right_bottom.x = v1[0] / v1[2];
    corners.right_bottom.y = v1[1] / v1[2];
}

//------------------------------------------------------
//@remark 直接将两张图片拼接会出现断裂，需要进行“混合”过渡
//@param img1是左边的图像
//@param trans是右边进行投影之后的图像
//@param dst是最终配准得到的图像
//@param corners是右图投影之后四个角的坐标
static void OptimizeSeam(const cv::Mat& img1,const cv::Mat& trans,cv::Mat& dst,const Corners& corners)
{

    int start = MIN(corners.left_top.x, corners.left_bottom.x);//开始位置，即重叠区域的左边界
    double processWidth = img1.cols - start;//重叠区域的宽度
    int rows = dst.rows;
    int cols = img1.cols; //注意，是列数*通道数
    double alpha = 1;//img1中像素的权重
    for (int i = 0; i < rows; i++)
    {
        const uchar* p = img1.ptr<uchar>(i);  //获取第i行的首地址
        const uchar* t = trans.ptr<uchar>(i);
        uchar* d = dst.ptr<uchar>(i);
        for (int j = start; j < cols; j++)
        {
            //如果遇到图像trans中无像素的黑点，则完全拷贝img1中的数据
            if (t[j * 3] == 0 && t[j * 3 + 1] == 0 && t[j * 3 + 2] == 0)
            {
                alpha = 1;
            }
            else
            {
                //img1中像素的权重，与当前处理点距重叠区域左边界的距离成正比，实验证明，这种方法确实好
                alpha = (processWidth - (j - start)) / processWidth;
            }

            d[j * 3] = p[j * 3] * alpha + t[j * 3] * (1 - alpha);
            d[j * 3 + 1] = p[j * 3 + 1] * alpha + t[j * 3 + 1] * (1 - alpha);
            d[j * 3 + 2] = p[j * 3 + 2] * alpha + t[j * 3 + 2] * (1 - alpha);
            }
        }
}
//----------------------------------------------------------------------
void Foo::myDrawMatches(const cv::Mat &in, const std::vector<cv::KeyPoint> &keypoints1,
                        const cv::Mat &in2, const std::vector<cv::KeyPoint> &keypoints2,
                        const std::vector<cv::DMatch> &matches1to2, cv::Mat &out,
                        const std::vector<char> &matchesMask,const cv::Scalar &matchColor)
{
    cv::RNG& rng = cv::theRNG();
    bool isRandMatchColor = matchColor == cv::Scalar::all(-1);
    cv::Scalar color = isRandMatchColor ? cv::Scalar( rng(256), rng(256), rng(256) ) : matchColor;

    //判断mask大小应该与matches的大小一致
    if( matchesMask.size() != matches1to2.size() && !matchesMask.empty() )
        return;

    cv::Size outSize = cv::Size(in.size().width + in2.size().width,
                                MAX(in.size().height,in2.size().height));
    //两张图片整合
    out.create(outSize,CV_MAKETYPE(in.depth(),3));
    //两张临时图片，分别复制in,in2的数据
    cv::Mat out1 = out(cv::Rect(0,0,in.size().width,in.size().height)),
            out2 = out(cv::Rect(in.size().width,0,in2.size().width,in2.size().height));

    //假如是灰度图，需要统一转成BGR形式
    if(in.type() == CV_8U)
        cv::cvtColor(in,out1,cv::COLOR_GRAY2BGR);
    else
        in.copyTo(out1);

    if(in2.type() == CV_8U)
        cv::cvtColor(in2,out2,cv::COLOR_GRAY2BGR);
    else
        in2.copyTo(out2);

    for(size_t m = 0; m < matches1to2.size(); m++)
    {
        if( matchesMask.empty() || matchesMask[m])
        {
            color = isRandMatchColor ? cv::Scalar( rng(256), rng(256), rng(256) ) : matchColor;

            int index1 = matches1to2[m].queryIdx;
            int index2 = matches1to2[m].trainIdx;

            CV_Assert(index1 >= 0 && index1 < static_cast<int>(keypoints1.size()));
            CV_Assert(index2 >= 0 && index2 < static_cast<int>(keypoints2.size()));
            //两个关键点的位置
            cv::Point center1(cvRound(keypoints1[index1].pt.x * draw_multiplier),
                              cvRound(keypoints1[index1].pt.y * draw_multiplier));

            cv::Point center2(cvRound(keypoints2[index2].pt.x * draw_multiplier),
                              cvRound(keypoints2[index2].pt.y * draw_multiplier));
            //以两个关键点的尺度为正方形的边长圆
            int radius1 = cvRound(keypoints1[index1].size / 2 * draw_multiplier);
            int radius2 = cvRound(keypoints1[index2].size / 2 * draw_multiplier);
            cv::circle(out1,center1,radius1,color,1,cv::LINE_AA,draw_shift_bits);
            cv::circle(out2,center2,radius2,color,1,cv::LINE_AA,draw_shift_bits);

            //两个关键点连线
            cv::Point2f dpt2 =
                    cv::Point2f(std::min(keypoints2[index2].pt.x + out1.size().width,float(out.size().width - 1)),
                                keypoints2[index2].pt.y);
            cv::line(out,
                     cv::Point(center1.x,center1.y),
                     cv::Point(cvRound(dpt2.x * draw_multiplier),cvRound(dpt2.y * draw_multiplier)),
                     color,1,cv::LINE_AA,draw_shift_bits);
        }
    }
}

//-----------------------------------------------
void Foo::myStitch(const cv::Mat &left_img, const cv::Mat &right_img, cv::Mat &dst, int method)
{
    std::vector<cv::KeyPoint> keypoint1,keypoint2;
    cv::Mat desc1,desc2;
    std::vector<cv::DMatch> matches;
    //detect and compute
    if(method == 1)
    {
        cv::Ptr<cv::ORB> orb = cv::ORB::create(3000);
        orb->detectAndCompute(left_img,cv::Mat(),keypoint2,desc2);
        orb->detectAndCompute(right_img,cv::Mat(),keypoint1,desc1);
    }
    else if(method == 2)
    {
        cv::Ptr<cv::xfeatures2d::SIFT> sift = cv::xfeatures2d::SIFT::create(3000);
        sift->detectAndCompute(left_img,cv::Mat(),keypoint2,desc2);
        sift->detectAndCompute(right_img,cv::Mat(),keypoint1,desc1);
    }
    else if(method == 3)
    {
        cv::Ptr<cv::xfeatures2d::SURF> surf = cv::xfeatures2d::SURF::create(3000);
        surf->detectAndCompute(left_img,cv::Mat(),keypoint2,desc2);
        surf->detectAndCompute(right_img,cv::Mat(),keypoint1,desc1);
    }
    else
    {
        dst = cv::Mat();
        return;
    }
    //匹配 采用knn+ransac
    MatchUtility::knnMatch(desc1,desc2,matches);

    std::vector<char> matchesMask;
    MatchUtility::ransac(matches,keypoint1,keypoint2,matchesMask);

    //ransac之后的结果点
    std::vector<cv::Point2f> imagePoints1,imagePoints2;
    for(size_t m = 0; m < matches.size(); ++m)
    {
        if(matchesMask[m])
        {
            imagePoints1.push_back(keypoint1[matches[m].queryIdx].pt);
            imagePoints2.push_back(keypoint2[matches[m].trainIdx].pt);
        }
    }

    //获取最佳匹配的单应性矩阵作为投影矩阵
    cv::Mat homo = cv::findHomography(imagePoints1,imagePoints2,CV_RANSAC);

    Corners corners;
    calcNewPosAfterH(homo,right_img,corners);

    //图像配准
    //@param imageTransform是右边图像放射变换之后的结果

    cv::Mat imageTransform;
    cv::warpPerspective(right_img, imageTransform, homo, cv::Size(MAX(corners.right_top.x, corners.right_bottom.x), left_img.rows));

    //创建拼接后的图,需提前计算图的大小
    int dst_width = imageTransform.cols;  //取最右点的长度为拼接图的长度
    int dst_height = left_img.rows;

    //Mat dst(dst_height, dst_width, CV_8UC3);
    dst.create(dst_height, dst_width, CV_8UC3);
    dst.setTo(0);

    imageTransform.copyTo(dst(cv::Rect(0, 0, imageTransform.cols, imageTransform.rows)));

    left_img.copyTo(dst(cv::Rect(0, 0, left_img.cols, left_img.rows)));

    OptimizeSeam(left_img,imageTransform,dst,corners);

}
