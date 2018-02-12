#include "kalman_filter.h"
#include<iostream>

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
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
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

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  /*need to translate rectangular coordinate states into polar coordinates so that comparison with the radar
   sesnsor measurement z can be made (compute the difference y = z - h(x'), where y is the difference in
   polar coordinates, z is the radar measurement in polar coordinates, and h(x') is the function that will
   turn rectangular coordinates x' into polar coordinate equivalent (the sqrt(x^2 + y+2) for rho, arctan
   for angle etc.)
  */
  //declare variables to be used
  double rho = 0.0;
  double ang = 0.0;
  double rhodot = 0.0;

  //here goes the h function implementation
  //first of all, rho
  rho = sqrt(pow(x_[0],2)+pow(x_[1],2));
  //then the angle
  //check division by zero
  if(fabs(x_[0]) < 0.0001){
    cout << "Division by Zero on px', skipping this radar data sample" << endl;
    return; //exit the UpdateEKF function since there is no point using this data anymore (the x_[0] value could be written to a small number but not 0.. but losing one sensor data doesn't harm the result of the kalman filter, that's the beauty of it anyyway)
  }

  ang = atan2(x_[1],x_[0]); //ensure atan2 is used as opposed to atan
  //atan only computes first and fourth quadrant and will give out angles that will throw off the kalman filter predited states
  //somewhere midway where they start to diverge like crazy


  //Then the range rate, a.k.a radial velocity (change in range of rho)
  //check division by zero
  if(fabs(rho) < 0.0001){
    cout << "Division by Zero on rho, skipping this radar data sample" << endl;
    return;
  }

  rhodot = ((x_[0] * x_[2]) + (x_[1] * x_[3])) / rho;

  //update z_pred with the polar coordinate converted values before proceeding with the kalman filter steps

  VectorXd z_pred(3);
  z_pred << rho, ang, rhodot;
  VectorXd y = z - z_pred;
  //now ensure that the difference radian angle stays within -pi and pi (normalization)
  if(fabs(y[1]) > 2*M_PI){
    cout << "normalizing angle because the absolute value of z - z_pred exceeded 2*pi";
    y[1] = y[1] > 0 ? y[1] - 2*M_PI : y[1] + 2*M_PI;
  }

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
