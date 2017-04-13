#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

// Calculate the RMSE error over the test data for model evaluation.
// Returns a vector of resulting errors
VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
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

// Takes the current state vector and calculates the Jacobian
// matrix for use in the sensor update for radar.
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

VectorXd Tools::CartesianToPolar(const VectorXd& x_state) {
	VectorXd hx_prime(3,1);
	
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
	float radial_rate = (px*vx + py*vy) / distance;

	hx_prime << distance,
		bearing,
		radial_rate;

	return hx_prime;
}
