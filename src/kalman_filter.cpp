#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

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
  /**
   * TODO: predict the state
   */
  x_ = F_ * x_ ;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */

  VectorXd y = z - H_ * x_;       // comparing measurement with the prediction
  MatrixXd Ht = H_.transpose();   // taking transpose of H
  MatrixXd S = H_ * P_ * Ht + R_; // projecting measurement uncertainty into the meaurement space
  MatrixXd Si = S.inverse();      // taking inverse of S
  MatrixXd K =  P_ * Ht * Si;     // Kalman Gain

  x_ = x_ + (K * y);
  long int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;         // update our estimate and uncertainty
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  double px, py, vx, vy;
  px = x_(0); // position in x
  py = x_(1); // position in y
  vx = x_(2); // velocity in x
  vy = x_(3); // velocity in y

  double rho = sqrt(px * px + py * py);
  double phi = atan2(py, px);
  double rho_dot;

  if (fabs(rho) < 0.0001) { // checking division by 0
    rho_dot = 0;
  } else {
    rho_dot = (px*vx + py*vy)/rho;
  }

  VectorXd h(3);
  h << rho, phi, rho_dot;
  VectorXd y = z - h;             // comparing the measurement with the prediction
  MatrixXd Ht = H_.transpose();   // taking transpose of H
  MatrixXd S = H_ * P_ * Ht + R_; // projecting measurement uncertainty into the meaurement space
  MatrixXd Si = S.inverse();      // taking inverse of S
  MatrixXd K =  P_ * Ht * Si;     // Kalman Gain 
  
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;         // update our estimate and uncertainty
}
