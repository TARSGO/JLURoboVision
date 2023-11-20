#ifndef _EKF_
#define _EKF_
#include <Eigen/Dense>

/*
反陀螺：
  状态量：X_c ,Y_c ,Z_c ,V_x ,V_y ,V_z ,yaw ,v_yaw ,r
  观测量：X_r ,Y_r ,Z_r ,yaw  
*/
template <typename T, int X, int Y>
class EKF
{
public:
  struct Initstream
  {
  Eigen::Matrix<T,X,X> f;//状态转移矩阵
  Eigen::Matrix<T,Y,X> h;//输入矩阵
  Eigen::Matrix<T,X,X> Q;//过程噪声协方差矩阵
  Eigen::Matrix<T,Y,Y> R;//测量噪声协方差矩阵
  Eigen::Matrix<T,X,Y> P;//状态协方差矩阵
  Eigen::Matrix<T,X,1> X_post;//后验状态
  }
  //初始化
  explicit EKF(const Initstream & state)
  : f(state.f),
    h(state.h),
    Q(state.Q),
    R(state.R),
    P_post(state.P),
    I(Eigen::Matrix<T,X,Y>::Identity()),
  //  X_pre(n),
    X_post(state.Xpost.rows() ? state.Xpost : Eigen::Matrix<T,X,1>::Zero(n))
  {
  }
  void ResetEKF(const Initstream & state)
  {
    f=state.f;
    h=state.h;
    Q=state.Q;
    R=state.R;
    P_post=state.P;
  }
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
  Eigen::MatrixXd F(const Eigen::MatrixXd & x); 
  //输入雅可比矩阵求解
  Eigen::MatrixXd H(const Eigen::MatrixXd & x);

private:

  Eigen::Matrix<T,X,X> f;//状态转移矩阵
  Eigen::Matrix<T,Y,X> h;//输入矩阵
  Eigen::Matrix<T,X,X> Q;//过程噪声协方差矩阵
  Eigen::Matrix<T,Y,Y> R;//测量噪声协方差矩阵
  Eigen::Matrix<T,X,Y> P;//状态协方差矩阵
  Eigen::Matrix<T,X,Y> I;//单位矩阵
  Eigen::Matrix<T,X,1> X_pre;//先验状态
  Eigen::Matrix<T,X,1> X_post;//后验状态
 
};

#endif