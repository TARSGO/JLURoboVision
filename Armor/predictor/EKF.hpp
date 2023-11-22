#ifndef _EKF_
#define _EKF_
#include <Eigen/Dense>
#include <functional>
/*
反陀螺：
  X状态量：X_c ,Y_c ,Z_c ,V_x ,V_y ,V_z ,yaw ,v_yaw ,r
  Y观测量：X_r ,Y_r ,Z_r ,yaw  
  Z输入量：dt
*/
template <typename T, int X, int Y, int Z>
class EKF
{
public:
  EKF()=default;
  using fFunc   = std::function<Eigen::Matrix<T,X,1>(const Eigen::Matrix<T,X,1> &,Eigen::Matrix<T,Z,1> &)>;
  using hFunc   = std::function<Eigen::Matrix<T,Y,1>(const Eigen::Matrix<T,X,1> &)>;
  using QNoiseFunc = std::function<Eigen::Matrix<T,X,X>(const Eigen::Matrix<double,Z,1> & u)>;
  using RNoiseFunc = std::function<Eigen::Matrix<T,Y,Y>()>;

  //初始化
  explicit EKF(const fFunc & f, const hFunc & h, const QNoiseFunc & Q, const RNoiseFunc & R,const Eigen::Matrix<T,X,X> & P, const Eigen::Matrix<T,X,1> & X_post)
  : f(f),
    h(h),
    Q(Q),
    R(R),
    P_post(P),
    I(Eigen::Matrix<T,X,Y>::Identity()),
    X_post(Xpost.rows() ? Xpost : Eigen::Matrix<T,X,1>::Zero(n))
  {
  }

  void ResetEKF(const Eigen::Matrix<T,X,Y> & P, const Eigen::Matrix<T,X,1> & X_post)
  {
    P_post = P;
    X_post = X_post;
  }
  //输入量更新
  void updateU(const Eigen::Matrix<T,Z,1> u_input = 0){u = u_input;};
  //先验预测
  void pre_predict();
  //更新先验状态协方差
  void pre_P();
  //更新卡尔曼增益
  void Kk();
  //后验预测
  void post_predict();
  //更新后验状态协方差矩阵
  void post_P();
  //状态转移雅可比矩阵求解
  Eigen::Matrix<T,X,X> F(const Eigen::Matrix<T,X,1> & x); 
  //输入雅可比矩阵求解
  Eigen::Matrix<T,Y,X> H(const Eigen::Matrix<T,X,1> & x);

private:
  fFunc f;//状态转移矩阵
  hFunc h;//观测矩阵
  QNoiseFunc Q;//过程噪声协方差矩阵
  RNoiseFunc R;//测量噪声协方差矩阵
  Eigen::Matrix<T,X,X> P;//状态协方差矩阵
  Eigen::Matrix<T,X,Y> I;//单位矩阵
  Eigen::Matrix<T,X,1> X_pre;//先验状态
  Eigen::Matrix<T,X,1> X_post;//后验状态
  Eigen::Matrix<T,Z,1> u;//控制输入
};

#endif