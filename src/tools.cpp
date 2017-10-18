#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
    VectorXd rmse(4);
    rmse << 0,0,0,0;
    
    for (int i=0; i<estimations.size(); i++)
    {
        VectorXd diff = estimations[i] - ground_truth[i];
        diff = diff.array() * diff.array();
        rmse += diff;
    }
    
    rmse = rmse / estimations.size();
    rmse = rmse.array().sqrt();
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
    MatrixXd Hj(3,4);
    
    double px = x_state(0);
    double py = x_state(1);
    double vx = x_state(2);
    double vy = x_state(3);
    
    double pxpy2 = pow(px,2) + pow(py,2);
    if (fabs(pxpy2) < 0.0001){
        cout << "CalculateJacobian () - Error - Division by Zero" << endl;
        return Hj;
    }
        
    double pxpyS = sqrt(pxpy2);
    double pxpy32 = pxpy2 * pxpyS;
    
    Hj <<    px / pxpyS,                    py / pxpyS,                    0,                    0,
            -py / pxpy2,                    px / pxpy2,                    0,                    0,
         (py*(vx*py - vy*px))/pxpy32,   (px*(vy*px - vx*py))/pxpy32,    px / pxpyS,          py / pxpyS;

    return Hj;
}
