#include "imageUtility.h"

void ImageUtility::MatToQImage(const cv::Mat &mat,QImage &out){
    cv::Mat temp;
    switch(mat.type()){
    case CV_8UC4:                              //RGBA
        mat.copyTo(temp);
        break;
    case CV_8UC3:                              //BGR to RGB
        cvtColor(mat, temp, cv::COLOR_BGR2BGRA);
        break;
    case CV_8UC1:                              //灰度图
        cv::cvtColor(mat, temp, cv::COLOR_GRAY2BGRA);
        break;
    default:
        return;
    }

    out = QImage(temp.data, temp.cols, temp.rows, temp.step[0], QImage::Format_ARGB32).copy();
}

void ImageUtility::QImageToMat(const QImage &img,cv::Mat &out){
    switch (img.format()) {
    case QImage::Format_Invalid:
        return;
    case QImage::Format_Indexed8:
        cv::Mat(img.height(),img.width(),CV_8UC1,(void*)img.constBits(),img.bytesPerLine()).copyTo(out);
        break;
    case QImage::Format_RGB32:
        cv::Mat(img.height(),img.width(),CV_8UC4,(void*)img.constBits(),img.bytesPerLine()).copyTo(out);
        break;
    case QImage::Format_RGB888:{
        cv::Mat temp=cv::Mat(img.height(),img.width(),CV_8UC3,(void*)img.constBits(),img.bytesPerLine());
        cv::cvtColor(temp,temp,cv::COLOR_RGB2BGR);
        temp.copyTo(out);
        break;
    }
    default:
        QImage temp=img.convertToFormat(QImage::Format_ARGB32);
        cv::Mat(temp.height(),temp.width(),CV_8UC4,(void*)temp.constBits(),temp.bytesPerLine()).copyTo(out);
    }
}

void ImageUtility::drawKeyPoints(const cv::Mat &in,const std::vector<cv::KeyPoint>& kps,cv::Mat &out){
    switch (CV_MAT_CN(in.type())) {
    case 1:
        in.copyTo(out);
        break;
    case 3:
        cv::cvtColor(in,out,CV_BGR2GRAY);
        break;
    case 4:
        cv::cvtColor(in,out,CV_BGRA2GRAY);
        break;
    default:
        throw "Image Type Error!";
    }

    cv::Scalar color = cv::Scalar::all(-1);
    for (auto iter=kps.begin();iter!=kps.end();++iter){
        drawSingleKeyPoint(*iter,color,out);
    }
}

void ImageUtility::drawSingleKeyPoint(const cv::KeyPoint &kp,const cv::Scalar &color,cv::Mat &out){
    // cvRound()能够将指定的浮点数转换为最接近的整数
    cv::Point center( cvRound(kp.pt.x * 16), cvRound(kp.pt.y * 16));
    int radius=cvRound(kp.size/2*16);

    // 此处比较大小或存在bug，后续会修改
    if (kp.angle != -1.0f){
        float radian=kp.angle*CV_PI/180.0f;
        cv::Point dest(cvRound(cos(radian)*radius)+center.x,cvRound(sin(radian)*radius)+center.y);
        cv::line(out,center,dest,color,4);
    }
    else{
        radius=1*16;
        cv::circle(out,center,radius,color,1,cv::LINE_AA,4);
    }
}
