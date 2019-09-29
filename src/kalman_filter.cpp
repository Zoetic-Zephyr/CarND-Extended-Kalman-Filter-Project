#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

Tools KalmanFilter::tools_ = Tools();

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in, MatrixXd &H_in,
                        MatrixXd &R_lidar_in, MatrixXd &R_radar_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_lidar = R_lidar_in;
  R_radar = R_radar_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  // TODO: predict the state
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  // TODO: update the state by using Kalman Filter equations
  // using equations for kalman filter
  VectorXd y = z - H_ * x_;

  // ! H_ and R_lidar
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_lidar;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_ * Ht * Si;

  // new state
  x_ = x_ + (K * y);
  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  // TODO: update the state by using Extended Kalman Filter equations
  MatrixXd Hj = tools_.CalculateJacobian(x_);

  // using equations for extended kalman filter
  // convert radar measurements from cartesian coordinates (x, y, vx, vy) to polar (rho, phi, rho_dot).
  VectorXd x_polar = tools_.ConvertFromCartesianToPolarCoords(x_);
  
  VectorXd y = z - x_polar;

  // normalize the angle between -pi to pi
  while(y(1) > M_PI){
    y(1) -= 2 * M_PI;
  }

  while(y(1) < -M_PI){
    y(1) += 2 * M_PI;
  }

  // ! Hj and R_radar
  MatrixXd Ht = Hj.transpose();
  MatrixXd S = Hj * P_ * Ht + R_radar;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_ * Ht * Si;

  // new state
  x_ = x_ + (K * y);
  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * Hj) * P_;
}
