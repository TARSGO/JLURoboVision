#include "Armor/Armor.h"

bool ArmorDetector::isLight(const LightBar & light)
{
    // The ratio of light (short side / long side)
    float ratio = light.width / light.length;
    bool ratio_ok = 0.1 < ratio && ratio < 0.4;
    bool angle_ok = light.angle < 25.0;
    bool is_light = ratio_ok && angle_ok;

    return is_light;
}

/**
* @brief: find all the possible lights of armor  检测所有可能的灯条
*/
void ArmorDetector::findLights() {
	vector<vector<Point>> lightContours;  //candidate contours of lights roiIng中的候选灯条轮廓
	Mat contourImg; //image for the useage of findContours avoiding the unexpected change of itself 给findContours用的图像，防止findContours改变roiImg
	srcImg_binary.copyTo(contourImg); //a copy of roiImg, contourImg
	findContours(contourImg, lightContours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); //CV_RETR_EXTERNAL = 0, CV_CHAIN_APPROX_SIMPLE = 2       最耗时的操作，优化方向
	//cv::RotatedRect lightRect;  //RotatedRect for fitEllipse 拟合椭圆来的灯条旋转矩形
	//LightBar light;  //template light 临时灯条

	for (const auto& lightContour : lightContours) {
		if (lightContour.size() < 5) continue; //if contour's size is less than 6 , then it can not used to fitEllipse 轮廓点数小于6，不可拟合椭圆

    //TODO:远距离击打和识别装甲板到底有没有必要，我认为是不必要的
    if (contourArea(lightContour) < 20 || contourArea(lightContour) > 270) continue; //minarea of lightContour to filter some small blobs 面积筛选滤去小发光点
		auto lightRect = minAreaRect(lightContour); //lightContour fits into a RotatedRect 拟合椭圆
		auto light = LightBar(lightRect,contourArea(lightContour));//construct to a lightBar 构造为灯条

		if (isLight(light))
        {
            int sum_r = 0, sum_b = 0;
            // 由于直接从灯条解算出的装甲板矩形可能超出src的边界，需要求一下boundingRect和src矩形的交集再截取图像
            // size[1]是cols（列数，对应x向长度），size[0]是rows（行数，对应y向长度），所以这里要反一下
            auto cropRect = lightRect.boundingRect() & cv::Rect2i(0, 0, src.size[1], src.size[0]);
            auto roi= src(cropRect);
            // Iterate through the ROI

            for (int i = 0; i < roi.rows; i++) {
                for (int j = 0; j < roi.cols; j++) {
                    if (cv::pointPolygonTest(lightContour, cv::Point2f(j + lightRect.center.x, i + lightRect.center.y), false) >= 0) {
                        // if point is inside contour
                        sum_r += roi.at<cv::Vec3b>(i, j)[2];
                        sum_b += roi.at<cv::Vec3b>(i, j)[0];
                    }
                }
            }
            // Sum of red pixels > sum of blue pixels ?
            auto detectedColor = sum_r > sum_b ? Color::RED : Color::BLUE;
            if( enemyColor != detectedColor)
            {
                continue;
            }
            lights.emplace_back(light);
        }
    }
    if (lights.size() < 2 || lights.size() > 20) {
        state = DetectorState::LIGHTS_NOT_FOUND; //if lights is less than 2, then set state not found lights 灯条少于两条则设置状态为没找到灯条
        return; //exit
    }

	// sort the lightBars from left to right 将灯条从左到右排序
	sort(lights.begin(), lights.end(),
		[](LightBar & a1, LightBar & a2) {
		return a1.center.x < a2.center.x; });
	state = DetectorState::LIGHTS_FOUND;
	return;
}



