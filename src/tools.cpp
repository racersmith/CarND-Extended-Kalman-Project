#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::fpclassify;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth) {
	VectorXd rmse(4);
	rmse << 0, 0, 0, 0;

	int est_size = estimations.size();

	// Check for valid vectors
	if (est_size != ground_truth.size() || est_size == 0) {
		std::cout << "Invalid vectors passed to CalculateRMSE." << std::endl;
		std::cout << "  est size: " << est_size << std::endl;
		std::cout << "  truth size: " << ground_truth.size() << std::endl;
		return rmse;
	}
	else {
		// Calculate RMSE of valid vectors
		for (int i = 0; i < est_size; i++) {
			VectorXd residual = estimations[i] - ground_truth[i];
			residual = residual.array() * residual.array();
			rmse += residual;
		}
		rmse = rmse / est_size;
		rmse = rmse.array().sqrt();
		return rmse;
	}
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
	// Jacobian
	MatrixXd Hj(3, 4);

	// Pull parameters out of state
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	// Start pre-compute of repeating denominators
	float x2y2 = px*px + py*py;      // x^2+y^2

	// Check px*px + py*py == 0, set to small value to avoid divZero errors
	if (x2y2 < 0.0001) {
		x2y2 = 0.0001;
	}

	// Finish pre-compute
	float x2y2_05 = sqrt(x2y2);      // (x^2+y^2)^0.5
	float x2y2_15 = x2y2 * x2y2_05;  // (x^+y^2)^1.5

	// Compute the Jacobian matrix
	Hj << (px / x2y2_05), (py / x2y2_05), 0, 0,
		-(py / x2y2), (px / x2y2), 0, 0,
		(py*(vx*py - vy*px) / x2y2_15), (px*(px*vy - py*vx) / x2y2_15), (px / x2y2_05), (py / x2y2_05);

	return Hj;
}

MatrixXd Tools::CalculateCovarianceQ(const double & dt, const float & noise_ax, const float & noise_ay)
{
	MatrixXd Q(4, 4);

	// Calculate common parameters
	double dt2 = dt*dt;
	double dt3 = dt2*dt;
	double dt4 = dt3*dt;

	// Calculate process covariance matrix
	Q <<	dt4 / 4 * noise_ax, 0, dt3 / 2 * noise_ax, 0,
				0, dt4 / 4 * noise_ay, 0, dt3 / 2 * noise_ay,
				dt3 / 2 * noise_ax, 0, dt2*noise_ax, 0,
				0, dt3 / 2 * noise_ay, 0, dt2*noise_ay;

	return Q;
}

VectorXd Tools::CartesianToPolar(const VectorXd& x_state) {
	VectorXd hx_prime(3);
	
	// Pull parameters out of state
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);
	
	// Start pre-compute of repeating denominators
	float distance = sqrt(px*px + py*py);

	// Check for zero distance
	if (distance < 0.0001) {
		distance = 0.0001;
	}

	// intial bearing used if there is a domain error
	float bearing = 0.0;

	// update bearing if there is no domain error
	if (fpclassify(px) != FP_ZERO && fpclassify(py) != FP_ZERO) {
		bearing = atan2(py, px);
	}
	
	// radial velocity
	float distance_rate = (px*vx + py*vy) / distance;

	hx_prime << distance, bearing, distance_rate;

	return hx_prime;
}

VectorXd Tools::PolarToCartesian(const Eigen::VectorXd & z_radar)
{
	VectorXd result(4);

	// Pull parameters from measurment
	float distance = z_radar(0);
	float bearing = z_radar(1);
	float distance_rate = z_radar(2);

	// Calculate common parameters
	float cos_b = cos(bearing);
	float sin_b = sin(bearing);

	// Calculate Cartesian vector
	float px = distance*cos_b;
	float py = distance*sin_b;
	float vx = distance_rate*cos_b;
	float vy = distance_rate*sin_b;
	
	result << px, py, vx, vy;

	return result;
}

VectorXd Tools::NormalizeAngle(const Eigen::VectorXd& y) {
	VectorXd y_norm(3);

	double angle = y[1];
	
	while (angle > M_PI){
		angle -= 2*M_PI;
	}
	
	while (angle < -M_PI) {
		angle += 2*M_PI;
	}
	
	y_norm << y[0], angle, y[2];
	return y_norm;
}