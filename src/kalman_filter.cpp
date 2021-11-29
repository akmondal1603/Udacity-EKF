#include "kalman_filter.h"
#include <math.h>
#include<iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::endl;
using std::cout;

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
  x_=F_*x_;
  MatrixXd F_trans=F_.transpose();
  P_=F_*P_*F_trans+Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  VectorXd y=z-H_*x_;
  MatrixXd H_trans=H_.transpose();
  MatrixXd S=H_*P_*H_trans+R_;
  MatrixXd S_inv=S.inverse();
  MatrixXd K=P_*H_trans*S_inv;
  
  x_=x_+(K*y);
  long x_size = x_.size();
  MatrixXd I= MatrixXd::Identity(x_size, x_size);
  P_=(I-K*H_)*P_;
  
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  double px=x_(0);
  double py=x_(1);
  double vx=x_(2);
  double vy=x_(3);
  
  double rho=sqrt((px*px)+(py*py));
  double phi=atan2(py, px);
  double rho_dot=(px*vx+py*vy)/ rho;
  
  VectorXd h=VectorXd(3);
  if (rho< 0.0001)
  {
    cout << "ERROR: UpdateEKF () - Division by zero" << endl;
    h<<0,0,0;
  }
  
  h<<rho, phi, rho_dot;
  VectorXd y=z-h;
  
  if (y(1)>M_PI)
  {
    y(1) = y(1) - 2*M_PI;
  }
  else if (y(1)<-M_PI)
  {
    y(1) = y(1) + 2*M_PI;
  }
  
  MatrixXd H_trans=H_.transpose();
  MatrixXd S=H_*P_*H_trans+R_;
  MatrixXd S_inv=S.inverse();
  MatrixXd K=P_*H_trans*S_inv;
  
  x_=x_+(K*y);
  long x_size = x_.size();
  MatrixXd I= MatrixXd::Identity(x_size, x_size);
  P_=(I-K*H_)*P_;
}
