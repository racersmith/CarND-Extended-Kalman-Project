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

	Tools tools;

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

	// Measurement matrix - Laser
	H_laser_ << 1, 0, 0, 0,
							0, 1, 0, 0;

	// Measurement matrix - Radar
	// This will be updated each time
	Hj_ <<	0, 0, 0, 0,
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
    // first measurement
    //cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

		previous_timestamp_ = measurement_pack.timestamp_;

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
			ekf_.P_ <<	10, 0, 0, 0,
									0, 10, 0, 0,
									0, 0, 0.5, 0,
									0, 0, 0, 0.5;
			
			// convert to cartesion coordinates
			ekf_.x_ = tools.PolarToCartesian(measurement_pack.raw_measurements_);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state for a initial laser measurement.
      */
			
			// Set initial covariance
			// Initialize with higher confidence in position
			ekf_.P_ <<	0.5, 0, 0, 0,
									0, 0.5, 0, 0,
									0, 0, 100, 0,
									0, 0, 0, 100;
			
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
	// Calculate elapsed time
	double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
	previous_timestamp_ = measurement_pack.timestamp_;

	// If the time step, dt, is small the change in the prediction will be small
	// and can be skipped.
	if (dt > 0.001) {
		// Update state transition matrix with new dt
		ekf_.F_(0, 2) = dt;
		ekf_.F_(1, 3) = dt; 

		// Update the noise covariance matrix with new dt
		ekf_.Q_ = tools.CalculateCovarianceQ(dt, noise_ax_, noise_ay_);
		
		ekf_.Predict();
	}
  

  /*****************************************************************************
   *  Update
   ****************************************************************************/
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
		
		// Calculate Jacobian
		ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
		
		// Set measurement covariance matrix for Radar
		ekf_.R_ = R_radar_;

		// Run Extended Kalman measurement update
		ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // Laser updates

		// Set measurement matrix
		ekf_.H_ = H_laser_;

		// set measurement covariance matrix for Laser
		ekf_.R_ = R_laser_;
		
		// Run Kalman measurement update
		ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  //cout << "x_ = " << ekf_.x_ << endl;
  //cout << "P_ = " << ekf_.P_ << endl;
}
