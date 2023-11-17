#include "Armor/Armor.h"
#include "../General/General.h"
#include "imgui.h"
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
/**
 *@brief: show all the lights found in a copy of srcImg  在图像中显示找到的所有灯条
 */
void showLights(Mat & image, const vector<LightBar> & lights)
{
    Mat lightDisplay = Mat::zeros(image.size(), CV_8UC3);;//image for the use of dialaying the lights 显示灯条用的图像
    image.copyTo(lightDisplay);//get a copy of srcImg 获取源图像的拷贝
    //if detector finds lights 如果找到了灯条
    if (!lights.empty())
    {
        putText(lightDisplay, "LIGHTS FOUND!", Point(100, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 255), 1, 8, false); //title LIGHT_FOUND 大标题 “找到了灯条”
        for (auto light : lights)
        {
            Point2f lightVertices[4];
            light.lightRect.points(lightVertices);
            //draw all the lights' contours 画出所有灯条的轮廓
            cv::circle(lightDisplay, light.top, 3, cv::Scalar(0, 255, 0), 1);
            cv::circle(lightDisplay, light.bottom, 3, cv::Scalar(0, 255, 0), 1);
            cv::line(lightDisplay, light.top, light.bottom, cv::Scalar(0, 255, 0), 1);

            //draw the lights's center point 画出灯条中心
            circle(lightDisplay, light.center, 2, Scalar(0, 255, 0), 2, 8, 0);

            //show the lights' center point x,y value 显示灯条的中心坐标点\角度\长度
            putText(lightDisplay, std::to_string(int(light.angle)), light.center - Point2f(0, 15), cv::FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0), 1, 8, false);
            // putText(lightDisplay, std::to_string(int(light.center.x)), light.center, cv::FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0), 1, 8, false);
            // putText(lightDisplay, std::to_string(int(light.center.y)), light.center + Point2f(0, 15), cv::FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0), 1, 8, false);
            // putText(lightDisplay, std::to_string(int(light.length)), light.center + Point2f(0, 30), cv::FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0), 1, 8, false);
            putText(lightDisplay, std::to_string(light.area), light.center + Point2f(0, 30), cv::FONT_HERSHEY_PLAIN, 2, Scalar(255, 255, 255), 1, 8, false);
            
        }
    }
        //if detector does not find lights 如果没找到灯条
    else
    {
        putText(lightDisplay, "LIGHTS NOT FOUND!", Point(100, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 1, 8, false);//title LIGHT_NOT_FOUND 大标题 “没找到灯条”
    }
    //show the result image 显示结果图
    imshow("Lights Monitor", lightDisplay);
}

void ArmorDetector::SetCameraMatrix(cv::Mat& matrix) {
    cv::cv2eigen(matrix, m_CameraMatrix);
}

cv::Point2f ArmorDetector::Reproject(Eigen::Vector3f xyz) {
    auto result = (1.0 / xyz(2)) * m_CameraMatrix * xyz;
    return cv::Point2f(result(0), result(1));
}

/**
 *@brief: show all the armors matched in a copy of srcImg  在图像中显示找到的所有装甲板
 */
void ArmorDetector::showArmors(Mat & image, const vector<ArmorBox> & armors, const vector<LightBar>& lights,Eigen::Vector3f absxyz_show,Eigen::Vector3f camxyz_show)
{
    Mat armorDisplay = Mat::zeros(image.size(), CV_8UC3); //Image for the use of displaying armors 展示装甲板的图像
    image.copyTo(armorDisplay); //get a copy of srcImg 源图像的拷贝
    // if armors is not a empty vector (ARMOR_FOUND) 如果找到了装甲板
    if (!armors.empty())
    {
        ImGui::Begin("Info");
        ImGui::TextColored(ImVec4(0, 1, 0, 1), "ARMOR FOUND");
        ImGui::End();
        putText(armorDisplay, "ARMOR FOUND!", Point(100, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 0), 1, 8, false); //title FOUND 大标题 “找到了装甲板”
        //draw all the armors' vertices and center 画出所有装甲板的顶点边和中心
        for (auto armor : armors)
        {
            //draw the center 画中心
            circle(armorDisplay, armor.center, 2, Scalar(0, 255, 0), 2);

            for (size_t i = 0; i < 4; i++)
            {
                cv::circle(armorDisplay, armor.l_light.top, 3, cv::Scalar(0, 255, 0),  2, 8, 0);
                cv::circle(armorDisplay, armor.r_light.top, 3, cv::Scalar(0, 255, 0),  2, 8, 0);
                cv::circle(armorDisplay, armor.l_light.bottom, 3, cv::Scalar(0, 255, 0),   2, 8, 0);
                cv::circle(armorDisplay, armor.r_light.bottom, 3, cv::Scalar(0, 255, 0),   2, 8, 0);

                line(armorDisplay, armor.l_light.top, armor.l_light.bottom, Scalar(255, 255, 255), 2, 8, 0);
                line(armorDisplay, armor.r_light.top, armor.r_light.bottom, Scalar(255, 255, 255), 2, 8, 0);

                line(armorDisplay, armor.l_light.top, armor.r_light.bottom, Scalar(255, 255, 255), 2, 8, 0);
                line(armorDisplay, armor.l_light.bottom, armor.r_light.top, Scalar(255, 255, 255), 2, 8, 0);

                //SHOW THE DEBUG POINTS
                Point2f camLightcenter= Reproject(Eigen::Vector3f{0.0,0.0,camxyz_show(2)});
                Point2f abspoint=Reproject(Eigen::Vector3f{absxyz_show(0) ,absxyz_show(1) , absxyz_show(2) });//
                Point2f campoint=Reproject(camxyz_show);

                cv::circle(armorDisplay,Point(image.cols/2,image.rows/2),2,Scalar(255,255,255),3);//WHITE:the center of image
                cv::circle(armorDisplay, camLightcenter, 2, Scalar(0,255,0), 3);//GREEN:the center of camera lightcenter
                cv::circle(armorDisplay, abspoint, 1, Scalar(0,0,255), 3);//RED:position we predict in world xyz
                cv::circle(armorDisplay, campoint, 2, Scalar(255,0,0), 3);//BLUE:position we predict in camera xyz

                cv::putText(armorDisplay,  armor.classfication_result, Point2f(armor.center.x ,armor.center.y - 50),  cv::FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255));
            }
        }
    }
        //if armors is a empty vector (ARMOR_NOT FOUND) 如果没找到装甲板
    else
    {
        ImGui::Begin("Info");
        ImGui::TextColored(ImVec4(1, 0.2, 0.2, 1), "ARMOR NOT FOUND");
        ImGui::End();
        putText(armorDisplay, "ARMOR NOT FOUND!", Point(100, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255), 1, 8, false);//title NOT FOUND 大标题 “没找到装甲板”
    }
    line(armorDisplay, Point(image.cols/2, 0), Point(image.cols/2 ,image.rows), Scalar(0, 0, 255), 1, 8, 0);
    line(armorDisplay, Point(0 ,image.rows/2), Point(image.cols ,image.rows/2), Scalar(0, 0, 255), 1, 8, 0);
    //line(armorDisplay, armor.l_light.top, armor.r_light.bottom, Scalar(255, 255, 255), 2, 8, 0);

    //show the result armors image 显示结果图
    imshow("Armor Monitor",armorDisplay);
}

/**
 *@brief: lights, armors, lights to armors every information in one 所有调试用数据输出
 */
void ArmorDetector::showDebugInfo(bool showSrcImg_ON, bool showSrcBinary_ON, bool showLights_ON, bool showArmors_ON)
{
    if (showSrcImg_ON)
        imshow("src", src);
    if (showSrcBinary_ON)
        imshow("srcImg_Binary", srcImg_binary);
    if (showLights_ON)
        showLights(src_full, lights);
    if (showArmors_ON)
        showArmors(src_full, armors, lights,absxyz_show,camxyz_show);

}

