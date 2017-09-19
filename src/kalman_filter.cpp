#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

void KalmanFilter::Init(
    const VectorXd& x,
    const MatrixXd& P) {
  x_ = x;
  P_ = P;
}

void KalmanFilter::Predict(
    const MatrixXd& F,
    const MatrixXd& Q) {
  x_ = F * x_;
  P_ = F * P_ * F.transpose() + Q;
}

void KalmanFilter::Update(
    const VectorXd& z,
    const MatrixXd& H,
    const MatrixXd& R) {
  VectorXd error = z - H * x_;
  UpdateEKF(error, H, R);
}

void KalmanFilter::UpdateEKF(
    const VectorXd& y,
    const MatrixXd& H,
    const MatrixXd& R) {
  MatrixXd Ht = H.transpose();
  MatrixXd S = H * P_ * Ht + R;
  MatrixXd K = P_ * Ht * S.inverse();

  x_ = x_ + (K * y);
  P_ = P_ - K * H * P_;
}
