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

  using fFunc   = std::function<Eigen::Matrix<T,X,1>(const Eigen::Matrix<T,X,1> &,const Eigen::Matrix<T,Z,1> &)>;
  using JfFunc  = std::function<Eigen::Matrix<T,X,X>(const Eigen::Matrix<T,Z,1> &)>;
  using hFunc   = std::function<Eigen::Matrix<T,Y,1>(const Eigen::Matrix<T,X,1> &)>;
  using JhFunc   = std::function<Eigen::Matrix<T,Y,X>(const Eigen::Matrix<T,X,1> &)>;
  using QNoiseFunc = std::function<Eigen::Matrix<T,X,X>(const Eigen::Matrix<double,Z,1> & u)>;
  using RNoiseFunc = std::function<Eigen::Matrix<T,Y,Y>()>;

  //初始化
  explicit EKF(const fFunc & f, const JfFunc & j_f, const hFunc & h, const JhFunc & j_h, const QNoiseFunc & Q, const RNoiseFunc & R,const Eigen::Matrix<T,X,X> & P, const Eigen::Matrix<T,X,1> & X_post)
  : f(f),
    J_f(j_f),
    h(h),
    J_h(j_h),
    update_Q(Q),
    update_R(R),
    P_post(P),
    I(Eigen::Matrix<T,X,X>::Identity()),
    X_post(X_post)
  {
  }
  //,,
  void ResetEKF(const Eigen::Matrix<T,X,X> & p, const Eigen::Matrix<T,X,1> & x_post)
  {
    P_post = p;
    X_post = x_post;
  }
  //输入量更新
  void updateUZ(const Eigen::Matrix<T,Z,1> u_input = Eigen::Matrix<T,Z,1>().setZero(), const Eigen::Matrix<T,Y,1> z_input = Eigen::Matrix<T,Y,1>().setZero()){u = u_input;z = z_input;}
  //预测
  void predict()
  {
    X_pre = f(X_post,u);
    Q = update_Q(u);
    F = J_f(u);
    P_pre = F * P_post * F.transpose() + Q;

  }
  //更新
  void update()
  {
    H = J_h(X_pre);
    R = update_R();
    Residual = z - h(X_pre);
    K = P_pre * H.transpose() * (H * P_pre * H.transpose() + R);
    X_post = X_pre + K * Residual;
    P_post = (I - K * H) * P_pre;
  }

private:
  Eigen::Matrix<T,X,1> X_pre;//先验状态
  Eigen::Matrix<T,X,1> X_post;//后验状态
  Eigen::Matrix<T,Z,1> u;//控制输入
  Eigen::Matrix<T,Y,1> z;//观测输入
  fFunc f;//非线性状态转移矩阵
  JfFunc J_f;//状态转移雅可比矩阵求解
  Eigen::Matrix<T,X,X> F;//线性化状态转移矩阵
  hFunc h;//非线性观测矩阵
  JhFunc J_h;//输入雅可比矩阵求解
  Eigen::Matrix<T,Y,X> H;//线性化观测矩阵
  QNoiseFunc update_Q;
  Eigen::Matrix<T,X,X> Q;//过程噪声协方差矩阵
  Eigen::Matrix<T,Y,1> Residual;//残差
  RNoiseFunc update_R;
  Eigen::Matrix<T,Y,Y> R;//测量噪声协方差矩阵
  Eigen::Matrix<T,X,Y> K;//卡尔曼增益
  Eigen::Matrix<T,X,X> P_pre;//状态协方差矩阵
  Eigen::Matrix<T,X,X> P_post;//状态协方差矩阵
  Eigen::Matrix<T,X,X> I;//单位矩阵

};

#endif