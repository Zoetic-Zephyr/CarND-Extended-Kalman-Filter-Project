#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::cerr;
using std::vector;

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

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;
  
  // TODO: Finish initializing the FusionEKF.
  // measurement matrix - laser
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;


  // TODO: Set the process and measurement noises
  noise_ax_ = 9.0;
  noise_ay_ = 9.0;

  // * x will be populated below
  x = VectorXd(4);

  // * state covariance matrix P
  P = MatrixXd(4, 4);

  // * the initial transition matrix F
  F = MatrixXd(4, 4);

  // * process covariance matrix Q
  Q = MatrixXd (4, 4);

  

  // TODO: Task summary just for reference
  // 1. initialize variables and matrices (x, F, H_laser, H_jacobian, P, etc.)
  // 2. initialize the Kalman filter position vector with the first sensor measurements
  // 3. modify the F and Q matrices prior to the prediction step based on the elapsed time between measurements
  // 4. call the update step for either the lidar or radar sensor measurement. Because the update step for lidar and radar are slightly different, there are different functions for updating lidar and radar.

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
    cout << 'Kalman Filter Initializing...' << endl;

    // TODO: Initialize the state ekf_.x_ with the first measurement (from measurement data)
    //       if radar, need to convert to cartesian coordinates
    //       if lidar, no need to convert
    cout << 'Received the fist measurement.' << endl;
    cout << "Measurement is performed by sensor: "
         << ((measurement_pack.sensor_type_ == MeasurementPackage::RADAR) ? "radar " : "lidar ")
         << endl;

    double px;
    double py;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      double meas_rho = measurement_pack.raw_measurements_[0];
      double meas_phi = measurement_pack.raw_measurements_[1];
      px = meas_rho * cos(meas_phi);
      py = meas_rho * sin(meas_phi);

      // ! Although radar gives velocity data in the form of the range rate dot-ρ , a radar measurement does not contain enough information to determine the state variable velocities vx and vy. Therefore, vx and vy are set to be 0.
      x << px, py, 0.0, 0.0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
      px = measurement_pack.raw_measurements_[0];
      py = measurement_pack.raw_measurements_[1];

      x << px, py, 0.0, 0.0;
    }
    else {
      cerr << "Unknown sensor type of measurement; skipping initialization" << endl;
    }

    // TODO: Create the covariance matrix.
    P << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1000, 0,
         0, 0, 0, 1000;

    F << 1, 0, 1, 0,
         0, 1, 0, 1,
         0, 0, 1, 0,
         0, 0, 0, 1;

    // * At Δt=0, Q is all zeros by def, which is also Eigen's default values for matrix initialization
    ekf_.Init(x, P, F, H_laser_, R_laser_, R_radar_, Q);

    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  //  TODO: Update the state transition matrix F according to the new elapsed time.
  // Time is measured in seconds.
  // * 1. Compute the time elapsed between the current and previous measurements
  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  // * 2. Modify the F matrix so that the time is integrated
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;


  // TODO: Update the process noise covariance matrix.
  // Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
  // * 3. Set the process covariance matrix Q
  // * define dt_x to be used multiple times below 
  double dt_2 = dt * dt;
  double dt_3 = dt_2 * dt;
  double dt_4 = dt_3 * dt;

  double q11 = dt_4/4*noise_ax_;
  double q13 = dt_3/2*noise_ax_;

  double q22 = dt_4/4*noise_ay_;
  double q24 = dt_3/2*noise_ay_;

  double q31 = dt_3/2*noise_ax_;
  double q33 = dt_2*noise_ax_;

  double q42 = dt_3/2*noise_ay_;
  double q44 = dt_2*noise_ay_;

  // ! possible bug below line
  // ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << q11, 0, q13, 0,
            0, q22, 0, q24,
            q31, 0, q33, 0,
            0, q42, 0, q44;

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
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // TODO: Laser updates
    ekf_.Update(measurement_pack.raw_measurements_);

  }

  // print the output
  cout << "==========" << endl
            << "x_ = "      << endl << ekf_.x_ << endl
            << "-----"      << endl
            << "P_ = "      << endl << ekf_.P_ << endl
            << "==========" << endl            << endl;
}
