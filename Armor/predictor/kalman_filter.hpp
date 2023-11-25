//
// Created by zhouhb on 23-3-31.
//
#ifndef JLUROBOVISION_KALMAN_H
#define JLUROBOVISION_KALMAN_H

#include <Eigen/Dense>

class KalmanFilter
{
public:
    struct InitState
    {
        Eigen::MatrixXd F;      // state transition matrix
        Eigen::MatrixXd H;      // measurement matrix
        Eigen::MatrixXd Q;      // process noise covariance matrix
        Eigen::MatrixXd R;      // measurement noise covariance matrix
        Eigen::MatrixXd P;      // error estimate covariance matrix
        Eigen::VectorXd Xpost;  // X post-update value, an initial value provided for stability
    };
    explicit KalmanFilter(KalmanFilter::InitState state);

  // Initialize the filter with a guess for initial states.
  void init(const Eigen::VectorXd & x0);

  // Computes a predicted state
  Eigen::VectorXd predict(double T);

  // Update the estimated state based on measurement
  Eigen::MatrixXd update(const Eigen::VectorXd & Z);

  // Compute a predicted state but DOESN'T MODIFY INTERNAL STATES
  Eigen::VectorXd static_predict();

private:
  // Invariant matrices
  Eigen::MatrixXd F, H, Q, R;

  // Priori error estimate covariance matrix
  Eigen::MatrixXd P_pre;
  // Posteriori error estimate covariance matrix
  Eigen::MatrixXd P_post;

  // Kalman gain
  Eigen::MatrixXd K;

  // System dimensions
  int n;

  // N-size identity
  Eigen::MatrixXd I;

  // Predicted state
  Eigen::VectorXd X_pre;
  // Updated state
  Eigen::VectorXd X_post;
};
#endif  // JLUROBOVISION_KALMAN_H
