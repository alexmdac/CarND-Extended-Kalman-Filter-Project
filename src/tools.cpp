#include <cmath>
#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;

VectorXd CalculateRMSE(
    const vector<VectorXd> &estimations,
    const vector<VectorXd> &ground_truth) {

  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  if (estimations.size() != ground_truth.size()
      || estimations.size() == 0) {
    cerr << "Invalid estimation or ground_truth data" << endl;
    return rmse;
  }

  for (int i = 0; i < estimations.size(); ++i) {
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  rmse = rmse / estimations.size();
  rmse = rmse.array().sqrt();

  return rmse;
}

MatrixXd CalculateJacobian(const VectorXd& x_state) {
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	float c1 = px*px + py*py;
	float c2 = sqrt(c1);
	float c3 = c1 *c2;
 
	MatrixXd Hj(3, 4);

	if (fabs(c1) < 0.0001) {
		cerr << "CalculateJacobian () - Error - Division by Zero" << endl;
		return Hj;
	}

	Hj << (px/c2),               (py/c2),               0,     0,
       -(py/c1),               (px/c1),               0,     0,
		    py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

	return Hj;
}

MatrixXd CalculateProcessCovarianceMatrix(
    double dt, double noise_ax, double noise_ay) {

  MatrixXd Q(4, 4);

  double dt2 = dt * dt;
  double dt3 = dt * dt2;
  double dt4 = dt * dt3;

  Q << dt4/4 * noise_ax, 0,                dt3/2 * noise_ax, 0,
       0,                dt4/4 * noise_ay, 0,                dt3/2 * noise_ay,
       dt3/2 * noise_ax, 0,                dt2 * noise_ax,   0,
       0,                dt3/2 * noise_ay, 0,                dt2 * noise_ay;

  return Q;
}

MatrixXd CalculateStateTransitionMatrix(double dt) {
  MatrixXd F(4, 4);

  F << 1, 0, dt, 0,
       0, 1, 0,  dt,
       0, 0, 1,  0,
       0, 0, 0,  1;

  return F;
}

VectorXd ConvertCartesianToPolar(const VectorXd& cartesian) {
  VectorXd polar(3);

  double px = cartesian(0);
  double py = cartesian(1);
  double vx = cartesian(2);
  double vy = cartesian(3);
  double s = sqrt(px*px + py*py);

  polar << s, atan2(py, px), (px*vx + py*vy)/s;
  return polar;
}

double NormalizeAngle(double angle) {
  // TODO: Make this more efficient.
  while (angle < M_PI) {
    angle += 2 * M_PI;
  }
  while (angle > M_PI) {
    angle -= 2 * M_PI;
  }
  return angle;
}
