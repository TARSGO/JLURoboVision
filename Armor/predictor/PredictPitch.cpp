//
// Created by zhouhb on 23-3-26.
//

#include "PredictPitch.h"



void PredictPitch::InitPredictPitch(float bulletspeednow, double distance,float hrrec)
{
//    YAML::Node params = YAML::LoadFile("../General/config.yaml");

    cv::FileStorage file_settings( "../General/config.yaml", cv::FileStorage::READ);
    if (!file_settings.isOpened()) {
        std::cerr << "Failed to open YAML file in PredictPitch.cpp" << std::endl;
        return;
    }
    //简化模型
    //file_settings["hr"]>>hr;
    hr=hrrec;
    file_settings["t"]>>t;
    file_settings["theta"]>>theta;//theta是两个模型共用的
    file_settings["dk"]>>dk;
    dr = distance;
    file_settings["v0x"]>>v0x;//v0在水平方向的分量
    //吊射
    file_settings["Y_r"]>>Y_r;
    file_settings["a"]>>a;
    file_settings["theta_d"]>>theta_d;
    file_settings["Y_d"]>>Y_d;
    file_settings["X_d"]>>X_d;
    X_r = distance;//假设距离是这些，实际要根据电控传的东西
    file_settings.release();

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
    return hr - hk;
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
        ek = anglefd(t, theta);//FIXME:把它进一步封装一下
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
    else v0 = 15.7;
    return v0;
}

//迭代公式可以优化，当时写的时候只是为了让它收敛于目标点，效率未必多高
/*
*解释一下下面迭代theta的公式，Y_d是上次循环后X_d达到X_r时对应的高度，（计算它与目标值的误差并除最大值是为了提供一个在0到1之间的、与误差剧烈程度成正比的比例系数，理解为pid的p就行；后面那个计算可以作图理解一下
）
*/
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
        X_d = 0.24 * cos(theta_d);
        Y_d = 0.44 - 0.2 * sin(theta_d);
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

double PredictPitch::selectalg(double pitch, double distance, float bulletspeednow,float hrrec)
{
    InitPredictPitch(bulletspeednow,distance,hrrec);
    //if(distance<=7&&pitch>=-10)//fixme:利用角度和距离的约束决定采用哪个射击方案,回头需要实际测试进行微调*/
    //     angle();
    //else
         dropshot();

    /*if (abs(pitch - theta * 180.0 / 3.1415) > 20)
    {
        return pitch;
    }//简单滤个波*/
     pitch = -theta * 180.0 / 3.1415;
    if (pitch >16)
    {
        pitch=16;
    }//简单滤个波
    return pitch;
}
