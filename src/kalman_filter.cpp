#include "kalman_filter.h"

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
	VectorXd y = z - H_*x_;
	MatrixXd S = H_*P_*H_.transpose() + R_;
	MatrixXd K = P_*H_.transpose()*S.inverse();

	// New State
	x_ = x_ + K*y;
	int xn = x_.size();
	MatrixXd I = MatrixXd::Identity(xn, xn);  // The all powerful identity matrix
	P_ = (I - K*H_)*P_;
}

// Kalman filter update for radar measurement
void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
}
