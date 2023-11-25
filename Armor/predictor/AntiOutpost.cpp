#include "Armor.h"
#include "EKF.hpp"
void Outpost::AntiOutpostEKFInit()
{
  //f
  auto f = [](const Eigen::Matrix<double,9,1> & x,const Eigen::Matrix<double,1,1> & u)
  {
    Eigen::Matrix<double,9,1> x_pre = x;
    x_pre(0) = x(5) ;
    x_pre(1) = x(6) ;
    x_pre(2) = x(7) ;
    x_pre(3) += x(8) *u(0);
    return x_pre;
  };

  //J_f
  auto J_f = [](const Eigen::Matrix<double,1,1> & u)
  {
    double dt = u(0);
    Eigen::Matrix<double,9,9> F;
    F <<  1,   0,   0,   0,   0,   0,  0,   0,   0,
          0,   1,   0,   0,   0,   0,   0,  0,   0,
          0,   0,   1,   0,   0,   0,   0,   0,  0, 
          0,   0,   0,   1,   0,   0,   0,   0,   dt,
          0,   0,   0,   0,   1,   0,   0,   0,   0,
          0,   0,   0,   0,   0,   1,   0,   0,   0,
          0,   0,   0,   0,   0,   0,   1,   0,   0,
          0,   0,   0,   0,   0,   0,   0,   1,   0,
          0,   0,   0,   0,   0,   0,   0,   0,   1;
    return F;
  };  

  //h
  auto h = [](const Eigen::Matrix<double,9,1> & x_pre)
  {
    Eigen::Matrix<double,4,1> z;
    z(0) = x_pre(0) + x_pre(4) * cos(x_pre(3));
    z(1) = x_pre(1) + x_pre(4) * sin(x_pre(3));
    z(2) = x_pre(2);
    z(3) = x_pre(3);
    return z;
  };

  //J_h
  auto J_h = [](const Eigen::Matrix<double,9,1> & x_pre)
  {
    Eigen::Matrix<double,4,9> H;
    H <<  1,   0,   0,    x_pre(4)*sin(x_pre(3)),  -cos(x_pre(3)),  0,   0,  0,  0,
          0,   1,   0,   -x_pre(4)*cos(x_pre(3)),  -sin(x_pre(3)),  0,   0,  0,  0,
          0,   0,   1,   0,   0,   0,   0,   0,   0,
          0,   0,   0,   1,   0,   0,   0,   0,   0;
    return H;
  };

  //Q--由于我们采用恒速模型，故假定加速度=0且满足正态分布，故噪声主要来自于旋转中心平移加速度和旋转加速度，其对位移和速度的影响分别为1/2*dt^2*error和dt*error,故需要引入控制变量u
  auto Q = [](const Eigen::Matrix<double,1,1> & u)
  {
    constexpr double Q_xyz = 20;
    constexpr double Q_yaw = 100;
    constexpr double Q_r   = 800;//FIXME:ekf参数均来自RV，待测试
    Eigen::Matrix<double,9,9> q;
    double dt = u(0);
    double Exyz = 1/2*pow(dt,2)*Q_xyz;
    double Eyaw = 1/2*pow(dt,2)*Q_yaw;
    double Er = 1/2*pow(dt,2)*Q_r;
    double Ev_xyz = dt * Q_xyz;
    double Ev_yaw = dt * Q_yaw;
    q << Exyz*Exyz, 0, 0, 0, 0, Exyz*Ev_xyz, 0, 0, 0,
         0,Exyz*Exyz, 0, 0, 0, 0, Exyz*Ev_xyz, 0, 0,
         0, 0, Exyz*Exyz, 0, 0, 0, 0, Exyz*Ev_xyz, 0,
         0, 0, 0, Eyaw*Eyaw, 0, 0, 0, 0, Eyaw*Ev_yaw,
         0, 0, 0, 0, Er*Er, 0, 0, 0, 0,         
         Ev_xyz*Exyz, 0, 0, 0, 0, Ev_xyz*Ev_xyz, 0, 0, 0, 
         0, Ev_xyz*Exyz, 0, 0, 0, 0, Ev_xyz*Ev_xyz, 0, 0, 
         0, 0, Ev_xyz*Exyz, 0, 0, 0, 0, Ev_xyz*Ev_xyz, 0, 
         0, 0, 0, Ev_yaw*Eyaw, 0, 0, 0, 0, Ev_yaw*Ev_yaw;
    return q;
  };
  //R
  auto R = []()
  {
    constexpr double R_xyz = 0.05;
    constexpr double R_yaw = 0.02;
    Eigen::Matrix<double,4,4> r;
    r << R_xyz, 0, 0, 0,
         0, R_xyz, 0, 0,
         0, 0, R_xyz, 0,
         0, 0, 0, R_yaw;
    return r;
  };
  //P
  Eigen::Matrix<double,9,9> P;
  P.setIdentity();
  //X_post
  Eigen::Matrix<double,9,1> X_post;
  X_post << 0,0,0,0,553,0,0,0,0.4*360;//置R和Vyaw
  //CHI_Threshold
  double CHI_Threshold = 80;
  
  CV_EKF=EKF<double,9,4,1>(f,J_f,h,J_h,Q,R,P,X_post,CHI_Threshold);
}