#include "kalman_filter.h"
#include <iostream>
#define PI 3.1415926
using namespace std;

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /*
    State Predictions for both RADAR and LIDAR
  */
    
    x_ = F_*x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /*
   Update the state by using Kalman Filter equations
   FOR LIDAR READINGS
  */
    // Calculate Error y and Kalman Gain K
    VectorXd y = z - H_*x_;
    MatrixXd S = H_ * P_ * H_.transpose() + R_;
    MatrixXd K = P_ * H_.transpose() * S.inverse();
    
    // CORRECTION
    x_= x_ + K*y;
    
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K*H_)*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /*
   Update the state by using Extended Kalman Filter equations
   FOR RADAR READINGS
  */
    // Get Predicted Position and Velocity: x and y
    double px = x_(0);
    double py = x_(1);
    double vx = x_(2);
    double vy = x_(3);
    double pxpyS = sqrt(pow(px,2) + pow(py,2));
    
    // Calculate PHI using X,Y Position and Ensure its within range -PI < phi < PI
    double norm_angle = atan2(py,px);
    while (norm_angle > PI){
        norm_angle -= 2*PI;
    }
    while (norm_angle < -PI){
        norm_angle += 2*PI;
    }
    
    // Correct for any Oscillations in Predicted PHI's sign due to RHO being close to 0
    if (fabs(z(1) - norm_angle) > 2*PI)
    {
        if (norm_angle < z(1)){norm_angle += 2*PI;}
        if (norm_angle > z(1)){norm_angle -= 2*PI;}
    }
    
    // Calculate Polar Values from Predicted Cartesian Values
    VectorXd h(3);
    h << pxpyS,
         norm_angle,
         (px*vx + py*vy)/pxpyS;
    
    // Calculate Error y and Kalman Gain K
    VectorXd y = z - h;
    MatrixXd S = H_ * P_ * H_.transpose() + R_;
    MatrixXd K = P_ * H_.transpose() * S.inverse();
    
    // CORRECTION
    x_= x_ + K*y;
    
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K*H_)*P_;
    
}
