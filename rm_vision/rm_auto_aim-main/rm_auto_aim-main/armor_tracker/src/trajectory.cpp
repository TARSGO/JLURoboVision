#include "armor_tracker/trajectory.h"

double PredictPitchXY::v0now;

PredictPitchXY PredictPitchXY::instance; // 一定要初始化静态变量

double PredictPitchXY::setBulletSpeed(float bulletSpeed)
{
    if (bulletSpeed != 0)
    {
        return bulletSpeed;
    }
    else
    {
        return 15.7;
    }
}

void PredictPitchXY::InitPredictPitch(float bulletSpeedNow, double distance, float hrRec)
{
    // 赋值初始量
    v0now = setBulletSpeed(bulletSpeedNow);
    X_r = distance;
    Y_r = hrRec;
}

// 8.27：应当以时间为单位作为步长，或者说我下面使用的步长正是时间而我理解为了距离（受到沈航代码先入为主的影响），所以才会出现角度变动如此之大和稳定的问题
double PredictPitchXY::dropshotRK45()
{
    double error = 0;
    theta = 15 * 3.14 / 180.0; // 初始化角度
    for (int i = 0; i < maxstep; i++)
    { // 设置最大步长
        // 初始化

        time_acc = 0;
        theta_d = theta;                   // 将theta赋给过程量
        X_d = 0.1 * cos(theta_d);         // 枪管水平长度
        Y_d = 0.1 * sin(theta_d); // 枪管垂直高度
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

            // 计算近似解以及相对误差
            // 注：计算相对误差采用五阶精度与四阶精度系数结果之差
            // auto vclass_4 = v0_d + 25.0 / 216.0 * k1_v + 1408.0 / 2565.0 * k3_v + 2197.0 / 4104.0 * k4_v - 1.0 / 5.0 * k5_v;
            // auto thetaclass_4 = theta_d + 25.0 / 216.0 * k1_theta + 1408.0 / 2565.0 * k3_theta + 2197.0 / 4104.0 * k4_theta - 1.0 / 5.0 * k5_theta;

            auto vclass_5 = v0_d + 16.0 / 135.0 * k1_v + 6656.0 / 12825.0 * k3_v + 28561.0 / 56430.0 * k4_v - 9.0 / 50.0 * k5_v + 2.0 / 55.0 * k6_v;
            auto thetaclass_5 = theta_d + 16.0 / 135.0 * k1_theta + 6656.0 / 12825.0 * k3_theta + 28561.0 / 56430.0 * k4_theta - 9.0 / 50.0 * k5_theta + 2.0 / 55.0 * k6_theta;

            // auto error_v = abs(vclass_5 - vclass_4);
            // auto error_theta = abs(thetaclass_5 - thetaclass_4);
            // auto error_y = tan(error_theta) * time_step;

            // std::cout << "error_v:" << error_v << std::endl;
            // std::cout << "error_theta:" << error_theta << std::endl;
            // std::cout << "error_y:" << error_y << std::endl;

            v0_d = vclass_5;
            theta_d = thetaclass_5;
            X_d += time_step * v0_d * cos(theta_d);
            Y_d += time_step * v0_d * sin(theta_d);
            time_acc += time_step;
        }

        // 评估迭代结果，修正theta
        error = Y_r - Y_d; // error可以pid调节
        if (abs(error) < minerr_y)
        {
            // std::cout << i << std::endl;
            // std::cout << "..............................................." << std::endl;
            // std::cout << "theta:" << theta * 180 / 3.1415 << std::endl;
            // std::cout << "..............................................." << std::endl;
            return theta;
        } // 合适则输出本次迭代使用的theta
        else
        {
            theta += atan((error) / X_r);
            // std::cout << "...................................................." << std::endl;
            // std::cout << "error:" << error << std::endl;
            // std::cout << "theta_d:" << theta_d * 180 / 3.1415 << endl;
            // std::cout << "...................................................." << std::endl;
        }
    }

    // 迭代失败则输出上次的迭代值(暂定为0)
    return 0;
}

//一些参数
// struct tar_pos tar_position[4];
float static_yaw = 0.000f; // One degree = 0.01745 Radian，增大往左
int bias_time = 0;//to be confirmed
void PredictPitchXY::GimbalControlTransform(float xw, float yw, float zw,
                            float vxw, float vyw, float vzw,float v_yaw,
                            float r1,float r2,float dz,
                            int id, float yaw, float *aim_x, float *aim_y, float *aim_z 
                            )
{
    // float s_static = 0.10; //枪口前推的距离
	  // float z_static = static_z;//0.08f;//0.16; //yaw轴电机到枪口水平面的垂直距离 // FIXME: 是电机到枪口还是相机到枪口？

    // 线性预测
    float timeDelay = bias_time/1000.0 + time_acc;
    // std::cout << "time_acc is" << time_acc << std::endl;
    float tar_yaw = 0 ;
    tar_yaw += v_yaw * timeDelay;

    //计算四块装甲板的位置
	int use_1 = 1;
	int i = 0;
    int idx = 0; // 选择的装甲板
    //armor_type = 1 为平衡步兵
    if (id == 1) {
        for (i = 0; i<2; i++) {
            float tmp_yaw = tar_yaw + i * 3.14;
            float r = r1;
            tar_position[i].x = xw - r*cos(tmp_yaw);
            tar_position[i].y = yw - r*sin(tmp_yaw);
            tar_position[i].z = zw;
            tar_position[i].yaw = tar_yaw + i * 3.14;
        }

        float yaw_diff_min = fabsf(yaw - tar_position[0].yaw);

        //因为是平衡步兵 只需判断两块装甲板即可
        float temp_yaw_diff = fabsf(yaw - tar_position[1].yaw);
        if (temp_yaw_diff < yaw_diff_min)
        {
            yaw_diff_min = temp_yaw_diff;
            idx = 1;
        }


    } else {

        for (i = 0; i<4; i++) {
            float tmp_yaw = tar_yaw + i * 3.14/2.0f;
            float r = use_1 ? r1 : r2;
            tar_position[i].x = xw - r*cos(tmp_yaw);
            tar_position[i].y = yw - r*sin(tmp_yaw);
            tar_position[i].z = use_1 ? zw : dz + zw;
            tar_position[i].yaw = tar_yaw + i * 3.14/2.0f;
            use_1 = !use_1;
        }

            //2种常见决策方案：
            //1.计算枪管到目标装甲板yaw最小的那个装甲板
            //2.计算距离最近的装甲板

            //计算距离最近的装甲板
        //	float dis_diff_min = sqrt(tar_position[0].x * tar_position[0].x + tar_position[0].y * tar_position[0].y);
        //	int idx = 0;
        //	for (i = 1; i<4; i++)
        //	{
        //		float temp_dis_diff = sqrt(tar_position[i].x * tar_position[0].x + tar_position[i].y * tar_position[0].y);
        //		if (temp_dis_diff < dis_diff_min)
        //		{
        //			dis_diff_min = temp_dis_diff;
        //			idx = i;
        //		}
        //	}
        //

            //计算枪管到目标装甲板yaw最小的那个装甲板
        float yaw_diff_min = fabsf(yaw - tar_position[0].yaw);
        for (i = 1; i<4; i++) {
            float temp_yaw_diff = fabsf(yaw - tar_position[i].yaw);
            if (temp_yaw_diff < yaw_diff_min)
            {
                yaw_diff_min = temp_yaw_diff;
                idx = i;
            }
        }

    }

	
    *aim_x =tar_position[idx].x + vxw * timeDelay;
    *aim_y =tar_position[idx].y + vyw * timeDelay;
    *aim_z =tar_position[idx].z + vzw * timeDelay;

   
  // *pitch =GimbalControlGetPitch(sqrt((*aim_x) * (*aim_x) + (*aim_y) * (*aim_y)) + s_static,
  //           tar_position[idx].z - z_static, st.current_v);
	//  *pitch = trajectory(15.7, distance, *aim_z);
 
	//*pitch = (float)(atan2(*aim_z, *aim_x));
    // *yaw = (float)(atan2(*aim_y, *aim_x)) + (static_yaw * (*aim_x) * 0.46);
}