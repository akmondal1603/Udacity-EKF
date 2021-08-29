#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
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

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
 
  // noise 
  double noise_ax=9.0f;
  double noise_ay=9.0f;
  
  // Proess covarance matrix Q
  ekf_.Q_=MatrixXd(4,4);
 
  H_laser_<<1,0,0,0,
  			0,1,0,0;
  
  // Transition Matrix
  ekf_.F_=MatrixXd(4,4);
  ekf_.F_<< 1,0,0,0,
  			0,1,0,0,
  			0,0,1,0,
  			0,0,0,1;
  
  // State Co-variance matrix P
  ekf_.P_=MatrixXd(4,4);
  ekf_.P_<< 1,0,0,0,
  			0,1,0,0,
  			0,0,1000,0,
  			0,0,0,1000;


}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    //cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4); 
     
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      //cout << "EKF: Radar" << endl;
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      // set the state with the initial location and zero velocity
      // Asumption: theta is in radians
      // Asumption: not enouh information to initialize acceleration, because a_x and a_y are not known
    	ekf_.x_ << measurement_pack.raw_measurements_(0)*cos(measurement_pack.raw_measurements_(1)), 
              measurement_pack.raw_measurements_(0)*sin(measurement_pack.raw_measurements_(1)), 
              0, 
              0;

    	previous_timestamp_ = measurement_pack.timestamp_;
    	is_initialized_ = true;
        // print the output
  		//cout << "x_ = " << ekf_.x_ << endl;
  		//cout << "P_ = " << ekf_.P_ << endl;
    	return;

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      //cout << "EKF: Laser" << endl;
      // TODO: Initialize state.
      // set the state with the initial location and zero velocity
    	ekf_.x_ << measurement_pack.raw_measurements_(0), 
              measurement_pack.raw_measurements_(1), 
              0, 
              0;

    	previous_timestamp_ = measurement_pack.timestamp_;
    	is_initialized_ = true;
        // print the output
 		//cout << "x_ = " << ekf_.x_ << endl;
  		//cout << "P_ = " << ekf_.P_ << endl;
    	return;
      
    }
    is_initialized_ = true;
    ekf_.x_ << 1, 1, 1, 1;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  double dt = (measurement_pack.timestamp_ - previous_timestamp_)/ 1000000.0;
  previous_timestamp_=measurement_pack.timestamp_;
  
  double dt_2=dt*dt;
  double dt_3=dt_2*dt;
  double dt_4=dt_3*dt;
  double dt_4_4=dt_4/4;
  double dt_3_2=dt_3/2;
  /*
  ekf_.F_=MatrixXd(4,4);
  ekf_.F_<< 1,0,dt,0,
  			0,1,0,dt,
  			0,0,1,0,
  			0,0,0,1;
  // noise 
  double noise_ax=9.0f;
  double noise_ay=9.0f;
  
  // Proess covarance matri Q
  ekf_.Q_=MatrixXd(4,4);
  */
  
  // noise 
  double noise_ax=9.0f;
  double noise_ay=9.0f;
  
  ekf_.F_(0,2) = dt;
  ekf_.F_(1,3) = dt;
  ekf_.Q_<< dt_4_4*noise_ax, 0, dt_3_2*noise_ax, 0,
  			0, dt_4_4*noise_ay, 0, dt_3_2*noise_ay,
  			dt_3_2*noise_ax, 0, dt_2*noise_ax, 0,
  			0, dt_3_2*noise_ay, 0, dt_2*noise_ay;  

  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates
    Tools tools;
    Hj_=tools.CalculateJacobian(ekf_.x_);
    ekf_.H_=Hj_;
    ekf_.R_=R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // TODO: Laser updates
    ekf_.H_=H_laser_;
    ekf_.R_=R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);

  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
