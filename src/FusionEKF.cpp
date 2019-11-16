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
  H_laser_ << 1, 0, 0, 0,
            0, 1, 0, 0;
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
   
   // the initial transition matrix F_
   ekf_.F_ = MatrixXd(4, 4);
   ekf_.F_ << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;

   // state covariance matrix P
   ekf_.P_ = MatrixXd(4, 4);
   ekf_.P_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;

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

    // first measurement
    cout << "Kalman Filter Initialization " << endl;

    // set the state with the initial location and zero velocity


    previous_timestamp_ = measurement_pack.timestamp_;
    //is_initialized_ = true;
    //return;
	
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
		double rho = measurement_pack.raw_measurements_[0]; // distance
		double theta = measurement_pack.raw_measurements_[1]; // bearing
		double rho_dot = measurement_pack.raw_measurements_[2]; // velocity

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
		ekf_.x_ << measurement_pack.raw_measurements_[0], 
              measurement_pack.raw_measurements_[1], 
              0, 
              0;
    }
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
   
   int noise_ax = 9;
   int noise_ay = 9;
      
   float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
   previous_timestamp_ = measurement_pack.timestamp_;
   
   float dt_2 = dt * dt;
   float dt_3 = dt_2 * dt;
   float dt_4 = dt_3 * dt;
   
   // Modify the F matrix so that the time is integrated
   ekf_.F_(0, 2) = dt;
   ekf_.F_(1, 3) = dt;
   
   // set the process covariance matrix Q
   ekf_.Q_ = MatrixXd(4, 4);
   ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
         0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
         dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
         0, dt_3/2*noise_ay, 0, dt_2*noise_ay;
		 
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
	ekf_.H_ = H_laser_;
	ekf_.R_ = R_laser_;
	ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
