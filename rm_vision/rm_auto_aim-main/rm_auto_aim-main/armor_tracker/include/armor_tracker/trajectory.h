#ifndef _TRAJECTORY_H
#define _TRAJECTORY_H
#include <cmath>
#include <iostream>
#include "math.h"

class PredictPitchXY
{
private:
    /**
     * @brief Set bullet speed
     * @param bulletSpeed: the speed of bullet(m/s)
     */
    double setBulletSpeed(float bulletSpeed);

    // 类成员就地初始化在c++11后开始支持，为保证扩展性就不用了;这里也没必要用初始化列表
    PredictPitchXY()
    {
        a = 0;
        theta_d = 0;
        X_d = 0;
        Y_d = 0;
        X_r = 0;
        Y_r = 0;
    }

    ~PredictPitchXY() = default;

    /**
     * @brief X、Y resistance(RK4)
     */
    double dropshotRK45();

    // void Xfunc(int countdie,double X_d,double Y_d,double theta,double v0_d);
    // void Yfunc(int countdie,double X_d,double Y_d,double theta,double v0_d);
    void InitPredictPitch(float bulletSpeedNow, double distance, float hrRec);

    // 单例模式
    static PredictPitchXY instance;
    PredictPitchXY(const PredictPitchXY &) = delete;
    PredictPitchXY &operator=(const PredictPitchXY &) = delete;

    double theta_d;                 // 迭代过程角度
    double v0_d;                    // 迭代过程速度
    double a;                       // 风阻带来的加速度
    double Y_d;                     // 迭代过程高度
    double X_d;                     // 迭代过程距离
    double X_r;                     // 目标的实际距离
    double Y_r;                     // 目标的实际高度
    const int step = 600;           // 迭代步长
    const int maxstep = 100;        // 迭代最大次数
    const double tolerance_v = 0.1; // 迭代容差，有待测试
    const double tolerance_theta = 0.1;
    const double tolerance_y = 0.01; // 迭代过程中允许y近似解的最大误差，注意与容许误差的区别
    const double smax = 5;           // 最大缩放因子
    const double smin = 2;           // 最小缩放因子
    const double safe = 1;           // 安全因子
    const double minerr_y = 0.01;    // y轴容许误差，y轴迭代后允许的最大误差

    double time_acc = 0;

    /**
     * @brief Init base parameter
     */
    static double v0now; // 输入弹速

    double theta;             // 角度
    const double k = 0.00556; // 阻力系数(大弹丸：0.00556、小弹丸：0.01903、发光大弹丸：0.00530)
    const double dt = 0.0005; // 每次仿真计算的步长
    const double g = 9.8;
    const double pi = 3.14159;

public:
    double operator()(float bulletSpeedNow, double distance, float hrRec)
    {
        instance.InitPredictPitch(bulletSpeedNow, distance, hrRec);
        theta = instance.dropshotRK45();
        if (theta == 0)
        {
            theta = atan(Y_r / X_r);
        } // theta为零，代表着第一次迭代就寄了，给出一个差不多的值（更好的办法是让外部读到一个状态直接）
        double pitch = theta * 180.0 / 3.1415;
        return pitch;
    }
    static PredictPitchXY &getinstance() { return instance; } // 返回单例(感觉用不着)

    //用于存储目标装甲板的信息
    struct tar_pos
    {
      float x;           //装甲板在世界坐标系下的x
      float y;           //装甲板在世界坐标系下的y
      float z;           //装甲板在世界坐标系下的z
      float yaw;         //装甲板坐标系相对于世界坐标系的yaw角
    };
    tar_pos tar_position[4];

    void GimbalControlTransform(float xw, float yw, float zw,
                               float vxw, float vyw, float vzw,float v_yaw,
                                float r1,float r2,float dz,
                                int id, float yaw, float *aim_x, float *aim_y, float *aim_z 
                                );
};
#endif