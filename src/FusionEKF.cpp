#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

const double epsilon = 0.00000001;

/**
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
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */


}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */
	 std::cout << measurement_pack.raw_measurements_ << std::endl;

    // first measurement
    //cout << "Kalman Filter Initialization " << endl;

    // set the state with the initial location and zero velocity


    previous_timestamp_ = measurement_pack.timestamp_;
    //is_initialized_ = true;
    //return;
	
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;
	std::cout << "dsfaaaa" << std::endl;
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
		std::cout << "dsfaaaa" << std::endl;
		double rho = measurement_pack.raw_measurements_[0]; // distance
		double theta = measurement_pack.raw_measurements_[1]; // bearing
		double rho_dot = measurement_pack.raw_measurements_[2]; // velocity
		std::cout << "dsfaaaa" << std::endl;
		double x = rho * cos(theta);
		double y = rho * sin(theta);
		double v_x = rho_dot * cos(theta);
		double v_y = rho_dot * sin(theta);
		ekf_.x_ << x, 
              y, 
              v_x, 
              v_y;
		
		
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
		std::cout << "dsfaaaa2" << std::endl;
		ekf_.x_ << measurement_pack.raw_measurements_[0], 
              measurement_pack.raw_measurements_[1], 
              0, 
              0;
    }
	std::cout << "dfsy" << std::endl;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }
  std::cout << "dfs" << std::endl;
  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates

  } else {
    // TODO: Laser updates

  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
