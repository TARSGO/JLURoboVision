#ifndef GENERAL_H
#define GENERAL_H

#include<condition_variable>
#include<mutex>
#include<thread>
#include<vector>
#include<opencv2/opencv.hpp>
#include<iostream>
#include<math.h>
#include<cstring>
#include"../Serial/Serial.h"
#include <Eigen/Dense>
#include "Debug/imgui/imgui.h"

#ifndef M_PI
#define M_PI 3.1415926535897932384
#endif


using cv::Point;
using cv::Point2f;
using cv::Point3f;
using cv::Scalar;
using cv::Mat;
using cv::Size;
using cv::FONT_HERSHEY_SIMPLEX;
using cv::MORPH_RECT;
using cv::waitKey;
using std::vector;
using std::cout;
using std::endl;

//#define DEBUG_MODE
#define RELEASE_MOD

// extern variables
extern std::mutex Globalmutex;            // threads conflict due to image-updating
extern std::condition_variable GlobalCondCV;     // threads conflict due to image-updating
extern std::condition_variable condVarMainThreadExit;
extern bool imageReadable;                  // threads conflict due to image-updating
extern cv::Mat src;                         // Transfering buffer
extern SerialReceiveData Communication_data;

extern std::atomic_bool SingleFrameFlag, SingleFrameMode, FrameFetched;

/**
* @brief: imageUpdating thread using camera
*/
int imageUpdatingThreadCamera();

/**
* @brief: imageUpdating thread using local video
*/
int imageUpdatingThreadLocal(std::string videoFilename);

/**
* @brief: armorDetecting thread
*/
void armorDetectingThread();


/**
 *@brief: the types of armor BIG SMALL 大装甲板 小装甲板
 */
enum class ArmorType
{
    SMALL_ARMOR = 0,
    BIG_ARMOR = 1,
};
/**
* @brief: colors in order B G R 颜色B蓝 G绿 R红
*/
enum class Color
{
    BLUE = 0,
    RED = 1
};

struct RotationAtt {
    double pitch, yaw, distance;
};
struct RelCoordAtt {
    double x, y, z;
    operator Eigen::Vector3d() const {
        return Eigen::Vector3d(x, y, z);
    }
};

/*
 *@brief: get the distance of two points(a and b) 获取两点之间的距离 
 */
inline float getPointsDistance(const Point2f& a, const Point2f& b)
{
	float delta_x = a.x - b.x;
	float delta_y = a.y - b.y;
	//return sqrtf(delta_x * delta_x + delta_y * delta_y);

	return sqrt(delta_x * delta_x + delta_y * delta_y);
}

/**
 * @brief 海伦公式计算三角形面积
 *
 * @param pts 三角形顶点
 * @return float 面积
 */
inline float calcTriangleArea(cv::Point2f pts[3])
{
    auto a = sqrt(pow((pts[0] - pts[1]).x, 2) + pow((pts[0] - pts[1]).y, 2));
    auto b = sqrt(pow((pts[1] - pts[2]).x, 2) + pow((pts[1] - pts[2]).y, 2));
    auto c = sqrt(pow((pts[2] - pts[0]).x, 2) + pow((pts[2] - pts[0]).y, 2));

    auto p = (a + b + c) / 2.f;

    return sqrt(p * (p - a) * (p - b) * (p - c));
}

/**
 * @brief 计算四边形面积
 *
 * @param pts 四边形顶点
 * @return float 面积
 */
inline float calcTetragonArea(cv::Point2f pts[4])
{
    return calcTriangleArea(&pts[0]) + calcTriangleArea(&pts[1]);
}

inline cv::Mat GetTransMat(float x, float y, float z)
{
    return cv::Mat_<float>(4, 4) <<
            1,  0,  0,  x,
            0,  1,  0,  y,
            0,  0,  1,  z,
            0,  0,  0,  1;
}

//将相机坐标系转换成世界坐标系
inline Eigen::Matrix3f GetRotMatq4(float q0, float q1, float q2, float q3)
{
    Eigen::Matrix3f Rcb;
    Rcb << 0,  0,  -1,
           1,  0,  0,
           0,  -1,  0;
    Eigen::Quaternionf q_raw(q0, q1, q2, q3);
    //cout <<"q:" <<q_raw<<endl;
    ImGui::Begin("RotMat");
    ImGui::Text("q0 : %f" ,q0);
    ImGui::Text("q1 : %f" ,q1);
    ImGui::Text("q2 : %f" ,q2);
    ImGui::Text("q3 : %f" ,q3);
    Eigen::Matrix3f Rbn = q_raw.toRotationMatrix();
    ImGui::End();

    return Rbn * Rcb;
}

inline RotationAtt xyz2PitchYawDis(double x, double y, double z)
{
    double yaw ,pitch ,distance;
    yaw = atan2(y, x) * 180 / CV_PI;
    pitch = atan2(z, sqrt (x * x + y * y)) * 180 / CV_PI;
    distance = sqrt(x * x + y * y + z * z);
    ImGui::Begin("inline");
    ImGui::Text("X: %lf", yaw );
    ImGui::Text("Y: %lf", pitch );
    ImGui::Text("Z: %lf", distance );
    ImGui::End();
    return RotationAtt{yaw, pitch, distance};
}

/**
* @brief 将角度限制在[-PI,PI]的范围内
* @return 处理后的角度
*/
inline float rangedAngleRad(float &angle)
{
    if (fabs(angle) > 180)
    {
        angle -= (angle / fabs(angle)) * 360;
        angle = rangedAngleRad(angle);
    }
    return angle;
}

/**
* @brief 将角度限制在[-PI,PI]的范围内
* @return 处理后的角度
*/
inline double easyFilter(double angle_yaw)
{
  double angle = angle_yaw;
  if(angle>90)angle-=180;
  if(angle<-90)angle+=180;
  return angle;
}

std::string CurrentPreciseTime();
std::ostream& HexDump(std::ostream& os, const void *buffer,
                      std::size_t bufsize, bool showPrintableChars = true);
std::string CurrentDateTime();

#endif // GENERAL_H
