//
// Created by zhouhb on 23-3-19.
//

#ifndef JLUROBOVISION_DBG3DSCENE_H
#define JLUROBOVISION_DBG3DSCENE_H


#include <cmath>
#include <opencv2/core/mat.hpp>
#include "ImGuizmo/ImGuizmo.h"

class Dbg3DScene {
public:
    Dbg3DScene();
//    ~Dbg3DScene();
    void BeginFrame();
    void Frame(int count);
    void SetGizmoAttitude(int index, const cv::Mat& rMat, const cv::Mat& tMat);
    void SetPredictedPosition(Eigen::VectorXd stateVec);

private:
    // GUI code
    void EditTransform(float* cameraView, float* cameraProjection, float* matrix, bool editTransformDecomposition);

    void Debug_Item1Matrix();

    // Internal data
    static constexpr int objMatrixCapacity = 4;
    float objectMatrix[objMatrixCapacity][16] = {
            { 1.f, 0.f, 0.f, 0.f,
                    0.f, 1.f, 0.f, 0.f,
                    0.f, 0.f, 1.f, 0.f,
                    0.f, 0.f, 0.f, 1.f },

            { 1.f, 0.f, 0.f, 0.f,
                    0.f, 1.f, 0.f, 0.f,
                    0.f, 0.f, 1.f, 0.f,
                    2.f, 0.f, 0.f, 1.f },

            { 1.f, 0.f, 0.f, 0.f,
                    0.f, 1.f, 0.f, 0.f,
                    0.f, 0.f, 1.f, 0.f,
                    2.f, 0.f, 2.f, 1.f },

            { 1.f, 0.f, 0.f, 0.f,
                    0.f, 1.f, 0.f, 0.f,
                    0.f, 0.f, 1.f, 0.f,
                    0.f, 0.f, 2.f, 1.f }
    };
    float predictedItem[16] = {
            1.f, 0.f, 0.f, 0.f,
            0.f, 1.f, 0.f, 0.f,
            0.f, 0.f, 1.f, 0.f,
            0.f, 0.f, 0.f, 1.f
    };

    const float identityMatrix[16] =
            { 1.f, 0.f, 0.f, 0.f,
              0.f, 1.f, 0.f, 0.f,
              0.f, 0.f, 1.f, 0.f,
              0.f, 0.f, 0.f, 1.f };

    float cameraView[16] =
            { 1.f, 0.f, 0.f, 0.f,
              0.f, 1.f, 0.f, 0.f,
              0.f, 0.f, 1.f, 0.f,
              0.f, 0.f, 0.f, 1.f };

    float cameraProjection[16];
    float fov;
    float viewWidth; // for orthographic
    float camYAngle;
    float camXAngle;
    int lastUsing;
    int totalCount;

    bool first_frame;

    static constexpr int maxItems = 1;

    float camDistance;
    ImGuizmo::OPERATION mCurrentGizmoOperation = ImGuizmo::TRANSLATE;

    // Misc camera functions copied from the example
    void Perspective(float fovyInDegrees, float aspectRatio, float znear, float zfar, float* m16)
    {
        float ymax, xmax;
        ymax = znear * tanf(fovyInDegrees * 3.141592f / 180.0f);
        xmax = ymax * aspectRatio;
        Frustum(-xmax, xmax, -ymax, ymax, znear, zfar, m16);
    }

    void LookAt(const float* eye, const float* at, const float* up, float* m16);

    void Frustum(float left, float right, float bottom, float top, float znear, float zfar, float* m16)
    {
        float temp, temp2, temp3, temp4;
        temp = 2.0f * znear;
        temp2 = right - left;
        temp3 = top - bottom;
        temp4 = zfar - znear;
        m16[0] = temp / temp2;
        m16[1] = 0.0;
        m16[2] = 0.0;
        m16[3] = 0.0;
        m16[4] = 0.0;
        m16[5] = temp / temp3;
        m16[6] = 0.0;
        m16[7] = 0.0;
        m16[8] = (right + left) / temp2;
        m16[9] = (top + bottom) / temp3;
        m16[10] = (-zfar - znear) / temp4;
        m16[11] = -1.0f;
        m16[12] = 0.0;
        m16[13] = 0.0;
        m16[14] = (-temp * zfar) / temp4;
        m16[15] = 0.0;
    }

    void Cross(const float* a, const float* b, float* r)
    {
        r[0] = a[1] * b[2] - a[2] * b[1];
        r[1] = a[2] * b[0] - a[0] * b[2];
        r[2] = a[0] * b[1] - a[1] * b[0];
    }

    float Dot(const float* a, const float* b)
    {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

    void Normalize(const float* a, float* r)
    {
        float il = 1.f / (sqrtf(Dot(a, a)) + FLT_EPSILON);
        r[0] = a[0] * il;
        r[1] = a[1] * il;
        r[2] = a[2] * il;
    }
};


#endif //JLUROBOVISION_DBG3DSCENE_H

/*
    这是一个 C++ 类 Dbg3DScene 的头文件。此类的目的是提供一个 3D 调试界面，包含控制器以操作场景中的对象。该类似乎是基于 ImGuizmo 库构建的。
Dbg3DScene 类的公共接口包括以下成员函数：
    BeginFrame()：通过设置相机视图和投影矩阵准备场景进行渲染。
    Frame()：渲染场景的当前状态。

Dbg3DScene 类的私有数据成员包括：
    objectMatrix：一个 4x16 矩阵，用于表示场景中每个对象的变换矩阵。
    identityMatrix：一个标准的 4x4 单位矩阵，用于初始化矩阵。
    cameraView：相机的视图矩阵。
    cameraProjection：相机的投影矩阵。
    isPerspective：一个布尔标志，表示相机是否使用透视或正交投影。
    fov：透视投影的视角。
    viewWidth：正交投影的宽度。
    camYAngle：相机的垂直角度。
    camXAngle：相机的水平角度。
    lastUsing：表示上次操作的对象的索引。
    maxItems：可以添加到场景中的最大对象数。
    camDistance：相机与场景之间的距离。
    mCurrentGizmoOperation：正在对对象执行的当前操作（平移、旋转或缩放）。

此外，Dbg3DScene 类定义了两个私有辅助函数，用于设置相机投影矩阵：Frustum() 和 Perspective()。
 *
 * */
