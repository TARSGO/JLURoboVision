/*
*	@Author: Qunshan He + WaterMelonJuice + a Vegetable Code Stitcher zhb
*	@Date:	 2020.06.15
*	@Brief:  This header file declares all the classes and params used to solve eular angle
*/

#ifndef ANGLE_SOLVER
#define ANGLE_SOLVER

#include "../General/General.h"
#include "../Armor/Armor.h"
#include <Eigen/Dense>

using namespace std;

class AngleSolver
{
public:
    AngleSolver();
    ~AngleSolver();

    /**
    * @brief Set camera params
    * @param camera_matrix: camera IntrinsicMatrix
    * @param distortion_coeff: camera DistortionCoefficients
    */
    bool setCameraParam();

    void setArmorSize(ArmorType type, float width, float height);

    void solveAngles(ArmorBox& target);

    cv::Mat getCameraMatrix();

    Eigen::Matrix<float,6,1> getArmorState(ArmorBox& target);

private:
    //Camera params
    Mat CAMERA_MATRIX;    //IntrinsicMatrix		  fx,fy,cx,cy
    Mat DISTORTION_COEFF; //DistortionCoefficients k1,k2,p1,p2

    //Object points in world coordinate
    vector<Point3f> SMALL_ARMOR_POINTS_3D;
    vector<Point3f> BIG_ARMOR_POINTS_3D;

    //Targets
    vector<Point2f> targetContour;
    ArmorType targetType;

    // calculated by solvePnP
    Mat rVec;    //rot rotation between camera and target center
    Mat tVec;  //trans translation between camera and target center

    //相机相对于枪管的偏移
    float m_x_off;
    float m_y_off;
    float m_z_off;

    //陀螺仪相对于水平面的偏移
    float m_yaw_off;
    float m_pitch_off;

    //对xyz的补偿，用于在弹道歪掉的情况下修正
    float m_chx_off;
    float m_chy_off;
    float m_chz_off;
};

#endif // !ANGLE_SOLVER

