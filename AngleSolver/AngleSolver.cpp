#include "../AngleSolver/AngleSolver.h"

#include <string>
#include <opencv2/core/eigen.hpp>
#include <yaml-cpp/yaml.h>

//hero:1 sentry_above:2 sentry_below:3 infantry1:4 infantry2:5 139_06:6 new139-01:7 new139-03:8

AngleSolver::AngleSolver()
{
    cv::FileStorage file_settings( "../General/config.yaml", cv::FileStorage::READ);
    if (!file_settings.isOpened()) {
        std::cerr << "Failed to open YAML file in AngleSolver.cpp" << std::endl;
        return ;
    }
    file_settings["m_x_off"]>>m_x_off;
    file_settings["m_y_off"]>>m_y_off;
    file_settings["m_z_off"]>>m_z_off;
    file_settings["m_yaw_off"]>>m_yaw_off;
    file_settings["m_pitch_off"]>>m_pitch_off;
    file_settings["m_chx_off"]>>m_chx_off;
    file_settings["m_chy_off"]>>m_chy_off;
    file_settings["m_chz_off"]>>m_chz_off;
    file_settings.release();
}

AngleSolver::~AngleSolver()
{
}


bool AngleSolver::setCameraParam() {

    Mat camera_matrix;
    Mat distortion_coeff;

    cv::FileStorage file_settings( "../General/config.yaml", cv::FileStorage::READ);
    if (!file_settings.isOpened()) {
        std::cerr << "Failed to open YAML file in AngleSolver.cpp MATRIX" << std::endl;
        return false;
    }
    file_settings["CAMERA_MATRIX"] >> camera_matrix;
    file_settings["DISTORTION_COEFF"] >> distortion_coeff;
    file_settings.release();

    CAMERA_MATRIX = camera_matrix.clone();
    DISTORTION_COEFF = distortion_coeff.clone();

    return true;
}

void AngleSolver::setArmorSize(ArmorType type, float width, float height)
{
    float half_x = width / 2.0;
    float half_y = height / 2.0;
    switch (type)
    {
        case ArmorType::SMALL_ARMOR:
            SMALL_ARMOR_POINTS_3D.emplace_back(Point3f(-half_x, half_y, 0));   //tl top left
            SMALL_ARMOR_POINTS_3D.emplace_back(Point3f(half_x, half_y, 0));	//tr top right
            SMALL_ARMOR_POINTS_3D.emplace_back(Point3f(half_x, -half_y, 0));   //br below right
            SMALL_ARMOR_POINTS_3D.emplace_back(Point3f(-half_x, -half_y, 0));  //bl below left
            break;

        case ArmorType::BIG_ARMOR:
            BIG_ARMOR_POINTS_3D.emplace_back(Point3f(-half_x, half_y, 0));   //tl top left
            BIG_ARMOR_POINTS_3D.emplace_back(Point3f(half_x, half_y, 0));    //tr top right
            BIG_ARMOR_POINTS_3D.emplace_back(Point3f(half_x, -half_y, 0));   //bl below left
            BIG_ARMOR_POINTS_3D.emplace_back(Point3f(-half_x, -half_y, 0));  //br below right
            break;
        default: break;
    }
}

void AngleSolver::solveAngles(ArmorBox& target) {
    targetContour = target.armorVertices;
    targetType = target.type;

    switch (targetType) {
        case ArmorType::SMALL_ARMOR:
            solvePnP(SMALL_ARMOR_POINTS_3D, targetContour, CAMERA_MATRIX, DISTORTION_COEFF, rVec, tVec, false,
                     cv::SOLVEPNP_IPPE);
            break;
        case ArmorType::BIG_ARMOR:
            solvePnP(BIG_ARMOR_POINTS_3D, targetContour, CAMERA_MATRIX, DISTORTION_COEFF, rVec, tVec, false,
                     cv::SOLVEPNP_IPPE);
            break;
        default:
            break;
    }
    // Save rVec & tVec
    target.rMat = rVec.clone();
    target.tMat = tVec.clone();
}

Eigen::VectorXf AngleSolver::getArmorState(ArmorBox& target)
{
    Mat mtxR,mtxQ;
    solveAngles(target);
    Eigen::Vector3f tVec_Eigen;

    cv2eigen(tVec ,tVec_Eigen);
    //tVec_Eigen(1) -= 140;
    //tVec_Eigen(2) += 120;
    cv::Rodrigues(rVec, rVec);
    cv::Vec3f eulerAngles = cv::RQDecomp3x3(rVec, mtxR, mtxQ);//Pitch Yaw Roll
    Eigen::Matrix<float,6,1 > state;
    state << tVec_Eigen(0),tVec_Eigen(1),tVec_Eigen(2), eulerAngles[0], eulerAngles[1], eulerAngles[2];
    return state;
}

// HACK
cv::Mat AngleSolver::getCameraMatrix() {
    return CAMERA_MATRIX;
}





