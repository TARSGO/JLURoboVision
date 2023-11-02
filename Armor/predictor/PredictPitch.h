//
// Created by zhouhb on 23-3-26.
//
#pragma once
#ifndef JLUROBOVISION_CALCULATEFIRINGSOLUTION_H
#define JLUROBOVISION_CALCULATEFIRINGSOLUTION_H


#include "General/General.h"
#include "../Armor/Armor.h"
#include "math.h"
#include <cmath>


class PredictPitch
{
public:
    /**
    * @brief Init all parameter
    */
    void InitPredictPitch(float bulletspeednow, double distance,float hrrec);


    /**
    * @brief Set bullet speed
    * @param bulletSpeed: the speed of bullet(m/s)
    */
    double setBulletSpeed(float bulletSpeed);

    /**
    * @brief time model
    */
    double timefd(double v0x, double t1);

    /**
    * @brief angle model
    */
    double anglefd(double t2, double theta);


    /**
    * @brief calculate time
    */
    void time(double v0x, double dr);

    /**
    * @brief calculate angle
    */
    void angle();
    /**
    * @brief used for hero dropshot
   */
    void dropshot();
    /**
    * @brief select algorithm
   */
    double selectalg(double pitch, double distance, float bulletspeednow,float hrrec);

private:
    double dk, vk, hk, dr;//dk、vk、hk、dr：迭代过程中的距离、速度、高度，与目标的实际距离（注意：长度单位均为m，角度均为角度值）
    double t, theta;//输出的t和角度（角度以弧度制给出）
    double v0,v0x;//发射速度，由从下位机接收到的速度给出
    double hr;//hr:目标高度与枪管的高度差（回头可以再测一下）

    const double k = 0.01903;//阻力系数
    const double dt = 0.0003;//每次仿真计算的步长
    const double g = 9.8;
    const double pi = 3.14159;
    double Y_r;//目标的实际高度
    double a;//风阻带来的加速度
    double Y_d;//枪管的实际高度
    double theta_d;//用于每次迭代的角度
    double X_r;//目标的实际距离
    double X_d;//枪管的实际长度
    double lasterr;//pid的上次误差
    double integral;//pid的积分项
    double v0now;//承装迭代时输入的v0
    double v0process;//每次迭代时使用的v0
};

#endif //JLUROBOVISION_CALCULATEFIRINGSOLUTION_H
