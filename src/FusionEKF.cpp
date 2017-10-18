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
  
  //measurement covariance matrix - radar
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
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
    // Initialize the state ekf_.x_ with the first measurement.
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;
    
    // Initialize Covariance Matrix ekf_.P_
    ekf_.P_ = MatrixXd(4,4);
    ekf_.P_ << 1,     0,     0,     0,
               0,     1,     0,     0,
               0,     0,    1000,   0,
               0,     0,     0 ,   1000;

    // Initialize timestamp
    previous_timestamp_ = measurement_pack.timestamp_;
      
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
        // Extract POLAR Measurements from RADAR readings
        double rho = measurement_pack.raw_measurements_(0);
        double phi = measurement_pack.raw_measurements_(1);
        double rhodot = measurement_pack.raw_measurements_(2);
        
        // Convert from POLAR to CARTESIAN
        double px = rho * cos(phi);
        double py = rho * sin(phi);
//        double vx = rhodot * cos(phi);
//        double vy = rhodot * sin(phi);
//        ekf_.x_ << px, py, vx, vy;
        
        // Initialize state ekf_.x_ with Measured RADAR Converted Values in CARTESIAN
        ekf_.x_ << px, py, 0, 0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
        // Initialize state ekf_.x_ with Measured LIDAR Values
        ekf_.x_ << measurement_pack.raw_measurements_(0), measurement_pack.raw_measurements_(1), 0, 0;
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
    
  // Calculate Delta Time (dt) to modify the F matrix for prediction
  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
    
  // Calculate F matrix
  ekf_.F_ = MatrixXd(4,4);
  ekf_.F_ <<    1,    0,    dt,    0,
                0,    1,    0,     dt,
                0,    0,    1,     0,
                0,    0,    0,     1;
  
  // Calculate Q Matrix for Process Noise Covariance
  double dt4 = pow(dt,4);
  double dt3 = pow(dt,3);
  double dt2 = pow(dt,2);
  
  ekf_.Q_ = MatrixXd(4,4);
  ekf_.Q_ <<    (dt4/4)*noise_ax,        0,        (dt3/2)*noise_ax,        0,
                       0,         (dt4/4)*noise_ay,       0,         (dt3/2)*noise_ay,
                (dt3/2)*noise_ax,        0,           dt2*noise_ax,         0,
                       0,         (dt3/2)*noise_ay,       0,            dt2*noise_ay;
  
  // Use the EKF
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
      ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
      ekf_.R_ = R_radar_;
      ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
      ekf_.H_ = H_laser_;
      ekf_.R_ = R_laser_;
      ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
