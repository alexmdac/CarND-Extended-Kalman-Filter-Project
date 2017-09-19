#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

// Calculates RMSE.
VectorXd CalculateRMSE(
    const vector<VectorXd> &estimations,
    const vector<VectorXd> &ground_truth);

// Calculates the Jacobian.
MatrixXd CalculateJacobian(const VectorXd& x_state);

// Calculates the process covariance matrix Q.
MatrixXd CalculateProcessCovarianceMatrix(
    double dt, double noise_ax, double noise_ay);

// Calculates the state transition matrix F.
MatrixXd CalculateStateTransitionMatrix(double dt);

// Converts cartesian state (4d) to polar (3d).
VectorXd ConvertCartesianToPolar(const VectorXd& cartesian);

// Normalizes the angle so that it's in [-pi, pi].
double NormalizeAngle(double angle);

#endif  /* TOOLS_H_ */
