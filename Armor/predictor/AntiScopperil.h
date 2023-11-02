//
// Created by yyy on 23-4-5.
//
#pragma once

#ifndef JLUROBOVISION_ANTISCOPPERIL_H
#define JLUROBOVISION_ANTISCOPPERIL_H
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

class AntiScopperil
        {
public:
    void get_appear_range(std::vector<cv::Point>& points, int margin, cv::Rect& rect);
    void predict(cv::Mat& img, cv::Rect& rect, double& period, cv::Point& pos);
    void predict_filtered(cv::Mat& img, cv::Rect& rect, double& period, cv::Point& pos);
    void show_result(cv::Mat& img, cv::Point& pos, int radius);
    void antiScopperil(Point target,Mat img);
    vector<cv::Point> points;    //先装一些点用于拟合出现位置
    int flag = 1;                 //是否要重新预测

};

#endif //JLUROBOVISION_ANTISCOPPERIL_H
