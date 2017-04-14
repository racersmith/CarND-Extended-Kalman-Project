#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
							0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
							0, 0.0009, 0,
							0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
	// Measurement matrix - Laser
	H_laser_ << 1, 0, 0, 0,
							0, 1, 0, 0;

	// Measurement matrix - Radar
	// This will be updated each time
	Hj_ <<	0, 0, 0, 0,
					0, 0, 0, 0,
					0, 0, 0, 0,
					0, 0, 0, 0;

	// Process noise
	noise_ax_ = 9.0;
	noise_ay_ = 9.0;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;


		// Create process covariance matrix
		ekf_.Q_ = MatrixXd(4, 4);
		// Create intial state transition matrix
		// this will be updated each pass with new dt
		ekf_.F_ = MatrixXd(4, 4);
		ekf_.F_ <<	1, 0, 1, 0,
								0, 1, 0, 1,
								0, 0, 1, 0,
								0, 0, 0, 1;

		// Create prediction covariance matrix
		ekf_.P_ = MatrixXd(4, 4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */

			// Set initial covariance
			// Initialize with higher confidence in velocity
			ekf_.P_ <<	1000, 0, 0, 0,
									0, 1000, 0, 0,
									0, 0, 10, 0,
									0, 0, 0, 10;
			
			// Extract measurement data
			double rho = measurement_pack.raw_measurements_[0];
			double phi = measurement_pack.raw_measurements_[1];
			double rho_dot = measurement_pack.raw_measurements_[2];
			VectorXd z(4, 1);
			z << rho, phi, rho_dot;
			
			// convert to cartesion coordinates
			ekf_.x_ << tools.PolarToCartesian(z);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state for a initial laser measurement.
      */
			
			// Set initial covariance
			// Initialize with higher confidence in position
			ekf_.P_ <<	10, 0, 0, 0,
									0, 10, 0, 0,
									0, 0, 10000, 0,
									0, 0, 0, 10000;
			
			// Extract laser data
			double x = measurement_pack.raw_measurements_[0];
			double y = measurement_pack.raw_measurements_[1];
			ekf_.x_ << x, y, 0.0, 0.0;
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

	// Calculate elapsed time
	double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 100000.0;
	previous_timestamp_ = measurement_pack.timestamp_;

	// Update state transition matrix with new dt
	ekf_.F_(0, 2) = dt;
	ekf_.F_(1, 3) = dt; 

	// Update the noise covariance matrix with new dt
	ekf_.Q_ = tools.CalculateCovarianceQ(dt, noise_ax_, noise_ay_);

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
  } else {
    // Laser updates
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
