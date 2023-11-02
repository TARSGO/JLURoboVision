//
// Created by zhouhb on 23-3-31.
//

#include "kalman_filter.hpp"

KalmanFilter::KalmanFilter(const InitState state)
: F(state.F),
  H(state.H),
  Q(state.Q),
  R(state.R),
  P_post(state.P),
  n(state.F.rows()),
  I(Eigen::MatrixXd::Identity(n, n)),
  X_pre(n),
  X_post(state.Xpost.rows() ? state.Xpost : Eigen::VectorXd::Zero(n))
{
}

void KalmanFilter::init(const Eigen::VectorXd & x0) { X_post = x0; }

Eigen::VectorXd KalmanFilter::predict(double T)
{
    // Sample frequency
    F(0, 3) = F(1, 4) = F(2, 5) = T;

    X_pre = F * X_post;
    P_pre = F * P_post * F.transpose() + Q;

    // handle the case when there will be no measurement before the next predict
    X_post = X_pre;
    P_post = P_pre;

    return X_pre.col(0);
}

Eigen::MatrixXd KalmanFilter::update(const Eigen::VectorXd & Z)
{
    K = P_pre * H.transpose() * (H * P_pre * H.transpose() + R).inverse();
    X_post = X_pre + K * (Z - H * X_pre);
    P_post = (I - K * H) * P_pre;

    return X_post;
}


Eigen::VectorXd KalmanFilter::static_predict()
{
    auto X_pre = F * X_post;

    return X_pre.col(0);
}