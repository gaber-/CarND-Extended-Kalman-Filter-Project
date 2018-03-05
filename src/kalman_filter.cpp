#include "kalman_filter.h"
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
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
    MatrixXd H_t = H_.transpose();
    VectorXd y = z - H_ * x_;
    MatrixXd S = H_ * P_ * H_t + R_;
    MatrixXd K =  P_ * H_t * S.inverse();
    MatrixXd I = MatrixXd::Identity(4,4);

    //new state
    x_ = x_ + (K * y);
    P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  float x = x_(0);
  float y_ = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  float rho = sqrt(x*x + y_*y_);
  if (fabs(rho) < 0.00001){
      x +=0.001;
      y_ +=0.001;
      rho = sqrt(x*x + y_*y_);
  }
  float theta = atan2(y_, x);
  float rho_dot = (x*vx + y_*vy)/rho;
  VectorXd z_pred = VectorXd(3);
  z_pred << rho, theta, rho_dot;
  VectorXd y = z - z_pred;
  
  MatrixXd H_t = H_.transpose();
  MatrixXd S = H_ * P_ * H_t + R_;
  MatrixXd K =  P_ * H_t * S.inverse();
  MatrixXd I = MatrixXd::Identity(4,4);

  if (fabs(y(1)) > 1){ // apparently when y_ changes from positive to negative (but not vice/versa?) y(1) sign gets swapped
      y(1) = z(1)+theta;}

  //new state
  x_ = x_ + (K * y);
  P_ = (I - K * H_) * P_;

}
