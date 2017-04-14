#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

class Tools {
public:
  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  /**
  * A helper method to calculate RMSE.
  */
  Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, const std::vector<Eigen::VectorXd> &ground_truth);

  /**
  * A helper method to calculate Jacobians.
  */
  Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);

	/**
	* A helper method to convert our Cartesian state to a polar measurement.
	*/
	Eigen::VectorXd CartesianToPolar(const Eigen::VectorXd& x_state);

	/**
	* A helper method to convert a polar measurment vector, z_radar, to a Cartesian vector
	*/
	Eigen::VectorXd PolarToCartesian(const Eigen::VectorXd& z_radar);

	/**
	* A helper method to calculate the covariance matrix Q for the 
	* given time step, dt and process noise constants noise_ax and noise_ay.
	*/
	Eigen::MatrixXd CalculateCovarianceQ(const float& dt, const float& noise_ax, const float& noise_ay);
};

#endif /* TOOLS_H_ */
