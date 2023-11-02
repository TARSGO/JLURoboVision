//
// Created by zhouhb on 23-4-5.
//

#include "AntiScopperil.h"


void AntiScopperil::get_appear_range(std::vector<cv::Point>& points, int margin, cv::Rect& rect)
{
    // 通过 minMaxLoc 函数求出点集的最大和最小坐标
    cv::Point minPoint;
    minPoint.x=0;
    minPoint.y=0;
    cv::Point maxPoint;
    maxPoint.x=0;
    maxPoint.y=0;
    //cv::minMaxLoc(points, &minPoint, &maxPoint);
    for (int i = 0; i < points.size(); i++)
    {
        if(points[i].x > maxPoint.x)
        {
            maxPoint.x = points[i].x;
        }
        else if(points[i].x < minPoint.x)
        {
            minPoint.x = points[i].x;
        }
        if(points[i].y > maxPoint.y)
        {
            maxPoint.y = points[i].y;
        }
        else if(points[i].y < minPoint.y)
        {
            minPoint.y = points[i].y;
        }
    }

    // 范围外扩像素数 margin
    cv::Size size(margin, margin);

    // 计算出现范围
    //rect.x = max(0, minPoint.x - margin);
    //rect.y = max(0, minPoint.y - margin);
    if (minPoint.x - margin>0)
    {
        rect.x = minPoint.x - margin;
    }
    else
    {
        rect.x = 0;
    }
    if (minPoint.x - margin>0)
    {
        rect.y = minPoint.y - margin;
    }
    else
    {
        rect.y = 0;
    }
    rect.width = maxPoint.x - rect.x + margin;
    rect.height = maxPoint.y - rect.y + margin;
}

void AntiScopperil::predict(cv::Mat& img, cv::Rect& rect, double& period, cv::Point& pos)
{
    // 将图片转为灰度图
    cv::Mat gray;
    cv::cvtColor(img, gray,cv::COLOR_BGR2GRAY);

    // 提取感兴趣区域
    cv::Mat roi = gray(rect);

    // 计算垂直方向上的行均值
    cv::Scalar meanValue = cv::mean(roi);

    // 通过差分计算周期
    int lastPixel = meanValue[0];
    for (int i = 1; i < roi.rows; i++)
    {
        int pixel = roi.at<uchar>(i, 0);
        if (pixel - lastPixel > 10) // 发生变化
        {
            double timeNow = cv::getTickCount() / cv::getTickFrequency(); // 现在时间
            period = timeNow / i; // 计算周期
            break;
        }
        lastPixel = pixel;
    }

    // 计算位置
    cv::Point maxLoc;
    cv::minMaxLoc(meanValue, NULL, NULL, NULL, &maxLoc); // 求平均值最大的位置
    pos.x = maxLoc.x + rect.x; // 加上偏移量
    pos.y = maxLoc.y + rect.y;
}

void AntiScopperil::predict_filtered(cv::Mat& img, cv::Rect& rect, double& period, cv::Point& pos)
{
    const int windowSize = 10; // 滑动窗口大小
    double sumPeriod = 0;
    cv::Point sumPos;
    for (int i = 0; i < windowSize; i++)
    {
        double periodNow;
        cv::Point posNow;
        predict(img, rect, periodNow, posNow); // 调用之前的周期和位置预测函数
        sumPeriod += periodNow;
        sumPos += posNow;
        cv::waitKey(1); // 延时一毫秒，防止界面卡死
    }
    period = sumPeriod / windowSize;
    pos = sumPos / windowSize;
}

void AntiScopperil::show_result(cv::Mat& img, cv::Point& pos, int radius)
{
    cv::circle(img, pos, radius, cv::Scalar(0, 0, 255), -1);
    cv::imshow("Result",img);
    cv::waitKey(1); // 延时一毫秒，防止界面卡死
}

void AntiScopperil::antiScopperil(Point target,Mat img)
{
    Rect rect;
    double period;
    Point predictPoint;
    if ((flag) && (points.size() < 250))
    {
        points.push_back(target);
    }
    get_appear_range(points,250, rect);
    predict(img, rect,period,predictPoint);
    predict_filtered(img, rect, period, predictPoint);
    show_result( img,  predictPoint,  2);
}