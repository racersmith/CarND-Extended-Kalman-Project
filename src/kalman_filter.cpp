#include "kalman_filter.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;  // State
  P_ = P_in;  // Covariance
  F_ = F_in;  // State transition
  H_ = H_in;  // Measurement matrix
  R_ = R_in;  // Measurement covariance
  Q_ = Q_in;  // Process covariance
}

void KalmanFilter::Predict() {
	// Prediction
	x_ = F_*x_;    // no external motion so u = 0
	P_ = F_*P_*F_.transpose() + Q_;
}

// Kalman filter update for laser measurement
void KalmanFilter::Update(const VectorXd &z) {
	// Measurement update
	VectorXd y = z - H_ * x_;
	MatrixXd Ht = H_.transpose();
	MatrixXd PHt = P_ * H_.transpose();
	MatrixXd S = H_ * PHt + R_;
	MatrixXd K = PHt * S.inverse();

	// New State
	x_ = x_ + K*y;
	int xn = x_.size();
	MatrixXd I = MatrixXd::Identity(xn, xn);
	P_ = (I - K*H_)*P_;
}

// Kalman filter update for radar measurement
void KalmanFilter::UpdateEKF(const VectorXd &z) {
	Tools tools;

	// Measurement update
	MatrixXd hx = tools.CartesianToPolar(x_);
	VectorXd y = z - hx;
	y = tools.NormalizeAngle(y);
	MatrixXd Ht = H_.transpose();
	MatrixXd PHt = P_ * H_.transpose();
	MatrixXd S = H_ * PHt + R_;
	MatrixXd K = PHt * S.inverse();

	// New state
	x_ = x_ + K*y;
	int xn = x_.size();
	MatrixXd I = MatrixXd::Identity(xn, xn);
	P_ = (I - K*H_)*P_;
}
