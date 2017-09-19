#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"

class KalmanFilter {
 public:
  /**
   * Initializes the predicted state and covariance matrix.
   * @param x The initial state
   * @param P The initial covariance matrix
   */
  void Init(
      const Eigen::VectorXd& x,
      const Eigen::MatrixXd& P);

  /**
   * Predicts the state and the state covariance
   * using the process model
   * @param F The state transition matrix
   * @param Q The process covariance matrix
   */
  void Predict(
      const Eigen::MatrixXd& F,
      const Eigen::MatrixXd& Q);

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   * @param H The measurement matrix
   * @param R The measurement covariance matrix
   */
  void Update(
      const Eigen::VectorXd& z,
      const Eigen::MatrixXd& H,
      const Eigen::MatrixXd& R);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param y The difference between the predicted state and the measured state
   * @param H The measurement matrix
   * @param R The measurement covariance matrix
   */
  void UpdateEKF(
      const Eigen::VectorXd& y,
      const Eigen::MatrixXd& H,
      const Eigen::MatrixXd& R);

  /**
   * Returns the predicted state.
   */
  const Eigen::VectorXd& x() const { return x_; }

  /**
   * Returns the covariance matrix.
   */
  const Eigen::MatrixXd& P() const { return P_; }

 private:
  Eigen::VectorXd x_;
  Eigen::MatrixXd P_;
};

#endif  /* KALMAN_FILTER_H_ */
