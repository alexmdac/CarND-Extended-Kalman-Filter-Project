#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

FusionEKF::FusionEKF() {
  is_initialized_ = false;
  previous_timestamp_ = 0;

  // measurement covariance matrix - laser
  R_laser_ = MatrixXd(2, 2);
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  // measurement covariance matrix - radar
  R_radar_ = MatrixXd(3, 3);
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  // projects 4D state into 2D lidar observation space (i.e. x, y).
  H_laser_ = MatrixXd(2, 4);
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    VectorXd x(4);
    MatrixXd P = MatrixXd::Zero(4, 4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      double range = measurement_pack.raw_measurements_(0);
      double angle = measurement_pack.raw_measurements_(1);
      double range_rate = measurement_pack.raw_measurements_(2);

      x << range * cos(angle),
           range * sin(angle),
           range_rate * cos(angle),
           range_rate * sin(angle);

      P << 0.09, 0, 0, 0,
           0, 0.09, 0, 0,
           0, 0, 0.09, 0,
           0, 0, 0, 0.09;
    } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      x << measurement_pack.raw_measurements_(0),
           measurement_pack.raw_measurements_(1),
           0.,
           0.;

      P.topLeftCorner<2, 2>() = R_laser_;
      // Lidar doesn't measure velocity, so the initial estimate is very
      // uncertain.
      P(2, 2) = P(3, 3) = 100.;
    }

    ekf_.Init(x, P);
    is_initialized_ = true;
    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1.e6;
  previous_timestamp_ = measurement_pack.timestamp_;

  MatrixXd F = CalculateStateTransitionMatrix(dt);

  // "Use noise_ax = 9 and noise_ay = 9 for your Q matrix."
  MatrixXd Q = CalculateProcessCovarianceMatrix(dt, 9., 9.);

  ekf_.Predict(F, Q);

  /*****************************************************************************
   *  Update
   ****************************************************************************/
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    VectorXd polar = ConvertCartesianToPolar(ekf_.x());

    VectorXd y = measurement_pack.raw_measurements_ - polar;
    y(1) = NormalizeAngle(y(1));

    ekf_.UpdateEKF(
        y,
        CalculateJacobian(ekf_.x()),
        R_radar_);
  } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    ekf_.Update(
        measurement_pack.raw_measurements_,
        H_laser_,
        R_laser_);
  }
}
