//
// Created by zhouhb on 23-3-26.
//

#include "PredictPitch.h"

void 

void PredictPitch::Init()
{
//    YAML::Node params = YAML::LoadFile("../General/config.yaml");

    cv::FileStorage file_settings( "../General/config.yaml", cv::FileStorage::READ);
    if (!file_settings.isOpened()) {
        std::cerr << "Failed to open YAML file in PredictPitch.cpp" << std::endl;
        return;
    }
    //简化模型
    file_settings["t"]>>t;
    file_settings["theta"]>>theta;//theta是两个模型共用的
    file_settings["dk"]>>dk;
    file_settings["v0x"]>>v0x;//v0在水平方向的分量
    //吊射
    file_settings["Y_r"]>>Y_r;
    file_settings["a"]>>a;
    file_settings["theta_d"]>>theta_d;
    file_settings["Y_d"]>>Y_d;
    file_settings["X_d"]>>X_d;
    file_settings.release();
}

void PredictPitch::InitPredictPitch(float bulletspeednow, double distance,float hrrec)
{

    hr=hrrec;
    dr = distance;//简化
    X_r = distance;//RK5

    v0now=setBulletSpeed(bulletspeednow);//承装输入的v0

}

//弹道模型部分 输入：弹速BULLET_SPEED 距离distance 输出：theta（即预测的pitch角）
double PredictPitch::timefd(double v0x, double t1)
{
    dk = 1.0 / k * log(k * v0x * t1 + 1.0);
    vk = v0x / (k * v0x * t1 + 1.0);
    return (dr - dk) / vk;
}
double PredictPitch::anglefd(double t2, double theta)
{
    hk = v0now * sin(theta) * t2 - 4.9 * t2 * t2;
    return hr - hk;//应当优化迭代条件,jian qiang guan gao du
}

void PredictPitch::time(double v0x, double dr)
{
    int i = 0;//记录迭代次数
    while (abs(dr - dk) > 0.01)
    {
        t = t + timefd(v0x, t);
        i++;
        if (i > 100)
        {
            break;
        }
    }
}

void PredictPitch::angle()
{
    int i = 0;//记录迭代次数
    double ek = 1;//每次运算后的反馈
    while (abs(ek) > 0.01)
    {
        v0x = v0now * cos(theta);
        time(v0x, dr);
        ek = anglefd(t, theta);
        i++;
        theta = theta + ek * 0.04;
        if (i > 400)
        {
            return;
        }
    }
}

double PredictPitch::setBulletSpeed(float bulletSpeed)
{
    if (bulletSpeed != 0) v0 = bulletSpeed;
    else v0 = 14.0;
    return v0;
}


void PredictPitch::dropshot()
{
    int count = 0;
    while (abs(Y_r - Y_d) > 0.01) {
        count = 0;
            if (Y_r - Y_d < 0)
            {
                theta = theta - abs(Y_r - abs(Y_d)) / fmax(Y_r, abs(Y_d)) * atan(abs(Y_r - Y_d) / X_d);
            }
            else
            {
                theta = theta + abs(Y_r - abs(Y_d)) / fmax(Y_r, abs(Y_d)) * atan(abs(Y_r - Y_d) / X_d);
            }
        //theta+=0.5*(abs(Y_r - abs(Y_d)) / max(Y_r, abs(Y_d)) * atan(abs(Y_r - Y_d) / X_d))
        theta_d = theta;
        X_d = 0.2 * cos(theta_d);
        Y_d = 0.42 - 0.2 * sin(theta_d);
        v0process =v0now ;
        while (abs(X_r - X_d) > 0.01) {
            a = k * v0process * v0process;
            v0process = v0process - a * dt;
            X_d = X_d + v0process * cos(theta_d) * dt;
            Y_d = Y_d + v0process * sin(theta_d) * dt;
            if ((cos(theta_d) * v0process) > 0)
                theta_d = atan((sin(theta_d) * v0process - g * dt) / (cos(theta_d) * v0process));

            if (cos(theta_d) <= 0.001)
                break;//发射的上升过程是不可能出现90度垂直的情况的
            if ((sin(theta_d) * v0process - g * dt) == 0)
                theta_d = 0;

            if ((sin(theta_d) * v0process - g * dt) < 0)
                theta_d = -atan(abs(sin(theta_d) * v0process - g * dt) / (cos(theta_d) * v0process));
            if ((count++) > 200) break;
        }
    }

}

double PredictPitchXY::dropshotRK45()
{
    double error = 0;
    theta = 0 * 3.14 / 180.0; // 初始化角度
    for (int i = 0; i < maxstep; i++)
    { // 设置最大步长
        // 初始化
        theta_d = theta;                   // 将theta赋给过程量
        X_d = 0.17 * cos(theta_d);         // TODO:枪管水平长度
        Y_d = 0.424 + 0.17 * sin(theta_d); // TODO:枪管垂直高度
        v0_d = v0now;
        // double x_step=(X_r-X_d)/step;//迭代步长
        double time_step = 0.0004; // 时间步长
        while (X_d < X_r)          // 迭代
        {
            // 计算各阶

            auto k1_v = (-k * pow(v0_d, 2) - g * sin(theta_d)) * time_step;
            auto k1_theta = (-g * cos(theta_d) / v0_d) * time_step; // 由于大弹丸很难产生马格努斯效应（横向摩擦轮怎么可能产生后向旋转），故忽略升力
            // 其实这里和公式是不完全相同的，未考虑t随步长的改变，因为t在推导过程中简化约去了

            auto k1_v_2 = v0_d + k1_v / 4.0;
            auto k1_theta_2 = theta_d + k1_theta / 4.0;

            auto k2_v = (-k * pow(k1_v_2, 2) - g * sin(k1_theta_2)) * time_step;
            auto k2_theta = (-g * cos(k1_theta_2) / k1_v_2) * time_step;
            auto k12_v_3 = v0_d + 3.0 / 32.0 * k1_v + 9.0 / 32.0 * k2_v;
            auto k12_theta_3 = theta_d + 3.0 / 32.0 * k1_theta + 9.0 / 32.0 * k2_theta;

            auto k3_v = (-k * pow(k12_v_3, 2) - g * sin(k12_theta_3)) * time_step;
            auto k3_theta = (-g * cos(k12_theta_3) / k12_v_3) * time_step;
            auto k123_v_4 = v0_d + 1932.0 / 2179.0 * k1_v - 7200.0 / 2179.0 * k2_v + 7296.0 / 2179.0 * k3_v;
            auto k123_theta_4 = theta_d + 1932.0 / 2179.0 * k1_theta - 7200.0 / 2179.0 * k2_theta + 7296.0 / 2179.0 * k3_theta;

            auto k4_v = (-k * pow(k123_v_4, 2) - g * sin(k123_theta_4)) * time_step;
            auto k4_theta = (-g * cos(k123_theta_4) / k123_v_4) * time_step;
            auto k1234_v_5 = v0_d + 439.0 / 216.0 * k1_v - 8.0 * k2_v + 3680.0 / 513.0 * k3_v - 845.0 / 4140.0 * k4_v;
            auto k1234_theta_5 = theta_d + 439.0 / 216.0 * k1_theta - 8.0 * k2_theta + 3680.0 / 513.0 * k3_theta - 845.0 / 4140.0 * k4_theta;

            auto k5_v = (-k * pow(k1234_v_5, 2) - g * sin(k1234_theta_5)) * time_step;
            auto k5_theta = (-g * cos(k1234_theta_5) / k1234_v_5) * time_step;
            auto k12345_v_6 = v0_d - 8.0 / 27.0 * k1_v + 2.0 * k2_v - 3544.0 / 2565.0 * k3_v + 1859.0 / 4104.0 * k4_v - 11.0 / 40.0 * k5_v;
            auto k12345_theta_6 = theta_d - 8.0 / 27.0 * k1_theta + 2.0 * k2_theta - 3544.0 / 2565.0 * k3_theta + 1859.0 / 4104.0 * k4_theta - 11.0 / 40.0 * k5_theta;

            auto k6_v = (-k * pow(k12345_v_6, 2) - g * sin(k12345_theta_6)) * time_step;
            auto k6_theta = (-g * cos(k12345_theta_6) / k12345_v_6) * time_step;

            auto vclass_5 = v0_d + 16.0 / 135.0 * k1_v + 6656.0 / 12825.0 * k3_v + 28561.0 / 56430.0 * k4_v - 9.0 / 50.0 * k5_v + 2.0 / 55.0 * k6_v;
            auto thetaclass_5 = theta_d + 16.0 / 135.0 * k1_theta + 6656.0 / 12825.0 * k3_theta + 28561.0 / 56430.0 * k4_theta - 9.0 / 50.0 * k5_theta + 2.0 / 55.0 * k6_theta;

            v0_d = vclass_5;
            theta_d = thetaclass_5;
            X_d += time_step * v0_d * cos(theta_d);
            Y_d += time_step * v0_d * sin(theta_d);
        }

        // 评估迭代结果，修正theta
        error = Y_r - Y_d; // error可以pid调节
        if (abs(error) < minerr_y)
        {
            std::cout << i << std::endl;
            std::cout << "..............................................." << std::endl;
            std::cout << "theta:" << theta * 180 / 3.1415 << std::endl;
            std::cout << "..............................................." << std::endl;
            return theta;
        } // 合适则输出本次迭代使用的theta
        else theta += atan((error) / X_r);

    }

    // 迭代失败则输出上次的迭代值(暂定为0)
    return 0;
}

double PredictPitch::selectalg(double pitch, double distance, float bulletspeednow,float hrrec)
{
    InitPredictPitch(bulletspeednow,distance,hrrec);
    if(distance<=7&&pitch>=-10)//fixme:利用角度和距离的约束决定采用哪个射击方案,回头需要实际测试进行微调*/
         angle();
    else
        dropshotRK45();

    /*if (abs(pitch - theta * 180.0 / 3.1415) > 20)
    {
        return pitch;
    }//简单滤个波*/
     pitch = -theta * 180.0 / 3.1415;//fixme:相当于直接把pitch接管了(修改：采用了与原值取平均的方式)，不过按理说也差不多，到时候输出的就是这个角，误差不大手描也好微调
    if (pitch >16)
    {
        pitch=16;
    }//简单滤个波
    return pitch;
}
