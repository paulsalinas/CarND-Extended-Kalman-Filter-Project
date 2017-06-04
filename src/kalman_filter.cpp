#include "kalman_filter.h"
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(
    VectorXd &x_in,
    MatrixXd &P_in,
    MatrixXd &F_in,
    MatrixXd &H_in,
    MatrixXd &R_in,
    MatrixXd &Q_in)
{
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict()
{
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z)
{
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;
  
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z)
{
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  VectorXd z_pred = h(x_);
  
  VectorXd y = z - z_pred;
  
  // normalize!
  while (y(1) < -M_PI)
    y(1) += 2 * M_PI;
  
  while (y(1) > M_PI)
    y(1) -= 2 * M_PI;
  
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

VectorXd KalmanFilter::h(const VectorXd &x) {
  VectorXd z_pred(3);
  float px = x(0);
  float py = x(1);
  float vx = x(2);
  float vy = x(3);
  
  float rho = sqrt(px * px + py * py);
  
  float phi;
  if (fabs(px) < 0.0001) {
    phi = 0;
  } else {
    phi = atan2(py, px);
  }
  
  float rho_dot;
  if (fabs(rho) < 0.0001) {
     rho_dot = 0;
  } else {
    rho_dot = (px * vx + py * vy) / rho;
  }
 
  z_pred << rho, phi, rho_dot;  
  return z_pred;
}
