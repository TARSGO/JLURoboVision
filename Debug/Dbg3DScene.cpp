//
// Created by zhouhb on 23-3-19.
//

#include "Eigen/Eigen"
#include "opencv2/core/eigen.hpp"
#include "imgui.h"
#include "Dbg3DScene.h"

void Dbg3DScene::BeginFrame() {
    ImGuiIO& io = ImGui::GetIO();
    Perspective(fov, io.DisplaySize.x / io.DisplaySize.y, 0.1f, 100.f, cameraProjection);
    ImGuizmo::SetOrthographic(false);
    ImGuizmo::BeginFrame();
    ImGuizmo::AllowAxisFlip(false);
}

// Created by Cursor
// Create a 4x4 transformation matrix from a rotation matrix and a translation vector
Eigen::Matrix4d CreateTransformMatrix(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation) {
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = rotation;
    transform.block<3, 1>(0, 3) = translation;
    return transform;
}

// Invert a 4x4 transformation matrix
Eigen::Matrix4d InvertTransformMatrix(const Eigen::Matrix4d& transform) {
    Eigen::Matrix3d rotation = transform.block<3, 3>(0, 0);
    Eigen::Vector3d translation = transform.block<3, 1>(0, 3);
    Eigen::Matrix3d inverse_rotation = rotation.transpose();
    Eigen::Vector3d inverse_translation = -inverse_rotation * translation;
    Eigen::Matrix4d inverse_transform = Eigen::Matrix4d::Identity();
    inverse_transform.block<3, 3>(0, 0) = inverse_rotation;
    inverse_transform.block<3, 1>(0, 3) = inverse_translation;
    return inverse_transform;
}
// End created by Cursor

void Dbg3DScene::LookAt(const float* eye, const float* at, const float* up, float* m16)
{
    float X[3], Y[3], Z[3], tmp[3];

    tmp[0] = eye[0] - at[0];
    tmp[1] = eye[1] - at[1];
    tmp[2] = eye[2] - at[2];
    Normalize(tmp, Z);
    Normalize(up, Y);

    Cross(Y, Z, tmp);
    Normalize(tmp, X);

    Cross(Z, X, tmp);
    Normalize(tmp, Y);

    m16[0] = X[0];
    m16[1] = Y[0];
    m16[2] = Z[0];
    m16[3] = 0.0f;
    m16[4] = X[1];
    m16[5] = Y[1];
    m16[6] = Z[1];
    m16[7] = 0.0f;
    m16[8] = X[2];
    m16[9] = Y[2];
    m16[10] = Z[2];
    m16[11] = 0.0f;
    m16[12] = -Dot(X, eye);
    m16[13] = -Dot(Y, eye);
    m16[14] = -Dot(Z, eye);
    m16[15] = 1.0f;
}

void Dbg3DScene::Frame(int count) {
    //Debug_Item1Matrix();
    if (first_frame)
    {
        float eye[] = { cosf(camYAngle) * cosf(camXAngle) * camDistance, sinf(camXAngle) * camDistance, sinf(camYAngle) * cosf(camXAngle) * camDistance };
        float at[] = { 0.f, 0.f, 0.f };
        float up[] = { 0.f, 1.f, 0.f };
        LookAt(eye, at, up, cameraView);
        first_frame = false;
    }

    static ImGuizmo::MODE mCurrentGizmoMode(ImGuizmo::LOCAL);
    ImGuiIO& io = ImGui::GetIO();
    float viewManipulateRight = io.DisplaySize.x;
    float viewManipulateTop = 0;

    ImGui::SetNextWindowSize(ImVec2(800, 400));
    ImGui::SetNextWindowPos(ImVec2(400,20));
    ImGui::PushStyleColor(ImGuiCol_WindowBg, (ImVec4)ImColor(0.35f, 0.3f, 0.3f));
    ImGui::Begin("Gizmo");
    ImGuizmo::SetDrawlist();
    float windowWidth = (float)ImGui::GetWindowWidth();
    float windowHeight = (float)ImGui::GetWindowHeight();
    ImGuizmo::SetRect(ImGui::GetWindowPos().x, ImGui::GetWindowPos().y, windowWidth, windowHeight);
    viewManipulateRight = ImGui::GetWindowPos().x + windowWidth;
    viewManipulateTop = ImGui::GetWindowPos().y;

    ImGuizmo::DrawGrid(cameraView, cameraProjection, identityMatrix, 100.f);
    ImGuizmo::ViewManipulate(cameraView, camDistance, ImVec2(viewManipulateRight - 128, viewManipulateTop), ImVec2(128, 128), 0x10101010);
    //ImGuizmo::DrawCubes(cameraView, cameraProjection, &objectMatrix[0][0], totalCount);

    ImGuizmo::Manipulate(cameraView, cameraProjection, mCurrentGizmoOperation, mCurrentGizmoMode, predictedItem, NULL, NULL, NULL, NULL);

    for (int matId = 0; matId < count; matId++)
    {
        ImGuizmo::SetID(matId);

        ImGuizmo::Manipulate(cameraView, cameraProjection, mCurrentGizmoOperation, mCurrentGizmoMode, objectMatrix[matId], NULL, NULL, NULL, NULL);
        if (ImGuizmo::IsUsing())
        {
            lastUsing = matId;
        }
    }


    ImGui::End();
    ImGui::PopStyleColor(1);
}

void Dbg3DScene::Debug_Item1Matrix()
{
    static bool translate = true, rotate = false;

    ImGui::Begin("Item1 Matrix Manip");

    ImGui::SliderFloat4("1:", objectMatrix[0] + 0, -4, 4);
    ImGui::SliderFloat4("2:", objectMatrix[0] + 4, -4, 4);
    ImGui::SliderFloat4("3:", objectMatrix[0] + 8, -4, 4);
    ImGui::SliderFloat4("4:", objectMatrix[0] + 12, -4, 4);
    ImGui::SliderFloat4("1:", objectMatrix[1] + 0, -4, 4);
    ImGui::SliderFloat4("2:", objectMatrix[1] + 4, -4, 4);
    ImGui::SliderFloat4("3:", objectMatrix[1] + 8, -4, 4);
    ImGui::SliderFloat4("4:", objectMatrix[1] + 12, -4, 4);

    translate = ImGui::RadioButton("Translate", translate);
    rotate = ImGui::RadioButton("Rotate", rotate);
    if(translate) mCurrentGizmoOperation = ImGuizmo::TRANSLATE;
    if(rotate) mCurrentGizmoOperation = ImGuizmo::ROTATE;

    ImGui::End();
}

Dbg3DScene::Dbg3DScene() {
    fov = 27.f;
    viewWidth = 10.f;
    camYAngle = 165.f / 180.f * 3.14159f;
    camXAngle = 32.f / 180.f * 3.14159f;
    camDistance = 8.f;
    first_frame = true;
    lastUsing = 0;

    totalCount = 4;
}

void Dbg3DScene::SetGizmoAttitude(int index, const cv::Mat& rMat, const cv::Mat& tMat) {
    if (index >= objMatrixCapacity) return;

#if 0
    Eigen::Map<Eigen::Matrix3d> rMatEig((double*)rMat.data);
    Eigen::Map<Eigen::Vector3d> tMatEig((double*)tMat.data);

    tMatEig /= 1000;

    auto transformMat = InvertTransformMatrix(CreateTransformMatrix(rMatEig, tMatEig));
    transformMat.transposeInPlace(); // FIXME：上面的变换矩阵什么的是让ChatGPT写的，出来的代码不知道为什么是转置过的，所以要再转置一下

    // FIXME: 暂时没找到更合理的赋值方法。直接memcpy对拷会出现错误。
    for(int i = 0; i < 16; i++) {
        objectMatrix[index][i] = transformMat.coeff(i / 4, i % 4);
    }
#else
    // 怀疑ChatGPT的代码有问题，所以先只写tMat
    for(int i = 0; i < 3; i++)
        objectMatrix[index][12 + i] = tMat.at<double>(i) / 1000;
#endif
}

void Dbg3DScene::SetPredictedPosition(Eigen::VectorXd stateVec) {
    for(int i = 0; i < 3; i++)
        predictedItem[12 + i] = stateVec(i) / 1000;
}


