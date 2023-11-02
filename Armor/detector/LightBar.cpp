/*
*	@Author: Mountain
*	@Date:	 2020.04.24
*	@Brief:  This cpp file define the object "light"
*/
#include "Armor/Armor.h"


LightBar::LightBar(){
    center = Point2f();
    angle = 0;
    width =0;
    length = 0;
    lightRect = cv::RotatedRect();
}

LightBar::LightBar(const cv::RotatedRect &lightRect){
    cv::Point2f p[4];
    lightRect.points(p);
    std::sort(p, p + 4, [](const cv::Point2f & a, const cv::Point2f & b) { return a.y < b.y; });
    center = lightRect.center;
    top = (p[0] + p[1]) / 2;
    bottom = (p[2] + p[3]) / 2;

    length = cv::norm(top - bottom);
    width = cv::norm(p[0] - p[1]);

    angle = std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y));
    angle = angle / CV_PI * 180;
}

LightBar::~LightBar(){}
