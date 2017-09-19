#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "tools.h"

class FusionEKF {
 public:
  FusionEKF();
  ~FusionEKF() = default;

  // Runs the whole flow of the Kalman Filter from here.
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  // Returns the predicted state: (px, py, vx, vy).
  const VectorXd& x() const { return ekf_.x(); }

 private:
  bool is_initialized_;
  long long previous_timestamp_;
  Eigen::MatrixXd R_laser_;
  Eigen::MatrixXd R_radar_;
  Eigen::MatrixXd H_laser_;
  KalmanFilter ekf_;
};

#endif /* FusionEKF_H_ */
