#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  using std::cout;
  using std::endl;

  // TODO: Calculate the RMSE here.
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // * 1. Check validity of input matrices
  if (estimations.size() == 0) {
    cout << 'Estimation.size() == 0' << endl;
    return rmse;
  } else if (estimations.size() != ground_truth.size()) {
    cout<< 'Unmatched sizes of estimations and ground_truth' << endl;
    return rmse;
  }
  
  // * 2. Accumulate squared residuals
  for (int i = 0; i < estimations.size(); i++) {
    // * each estimations/ground_truth element is a 1x4 vector containing px,py,vx,vy
    VectorXd residuals = estimations[i] - ground_truth[i];
    residuals = residuals.array() * residuals.array();
    rmse += residuals;
  }
  
  // * 3. Calculate mean
  rmse = rmse/rmse.size();

  // * 4. Calculate squared root
  rmse = rmse.array().sqrt();

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  using std::cout;
  using std::endl;
  // TODO: Calculate a Jacobian here.
  MatrixXd Hj(3,4);

  // * 1. Recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // * pre-calc values to save computation for later
  float q_sum = px*px + py*py; // * quadratic sum of px, py
  float g_mean = sqrt(q_sum); // * geometric mean of px, py
  float p = q_sum * g_mean; // * pow(q_sum, 3/2)

  if (fabs(q_sum) < 0.0001) {
    // * check division by zero
    cout << "CalculateJacobian () - Error - Division by Zero" << endl;
    
    return Hj;
  }

  // * 2. compute the Jacobian matrix
  Hj(0,0) = px/g_mean;
  Hj(0,1) = py/g_mean;
  Hj(0,2) = 0;
  Hj(0,3) = 0;
  
  Hj(1,0) = -py/q_sum;
  Hj(1,1) = px/q_sum;
  Hj(1,2) = 0;
  Hj(1,3) = 0;

  Hj(2,0) = py*(vx*py-vy*px)/p;
  Hj(2,1) = px*(vy*px-vx*py)/p;
  Hj(2,2) = px/g_mean;
  Hj(2,3) = py/g_mean;

  return Hj;
}
