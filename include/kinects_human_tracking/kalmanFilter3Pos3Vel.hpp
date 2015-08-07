#ifndef KALMAN_FILTER
#define KALMAN_FILTER

#include <iostream>
#include <Eigen/Dense>

/**
   Kalman Filter with state (x,y,z,vx,vy,vz) and measurement (x,y,z)
**/

using namespace std;

class KalmanFilter{

public:
  KalmanFilter();
  
  void init(Eigen::Vector3f jerk_std,
	  Eigen::Vector3f measur_std,
	  float delta_t,
	  Eigen::Matrix<float, 9,1> x_k1,
	  Eigen::Matrix<float, 9, 9> init_cov = Eigen::Matrix<float, 9, 9>::Zero());

  //if delta-t not known pass <=0
  void predict(float delta_t);
  
  void correct(Eigen::Vector3f z_k);
  
  //estimate new state vector given observation and dt
  void estimate(Eigen::Vector3f obs, float delta_t, Eigen::Matrix<float, 9, 1> &est);
 

private:
  Eigen::Matrix<float, 9,1> x_k_p_, x_k_n_; //previous and next state
  Eigen::Matrix<float, 9,1> x_k1_; // Xk-1
  Eigen::Matrix<float, 9, 9> A_, Q_, P_k_p_, P_k_n_, P_k1_;
  Eigen::Matrix3f R_, sigma_jerk_; //measurement and process noises
  Eigen::Matrix<float, 3, 9> H_;
  Eigen::Matrix<float, 9, 3> K_;
  float delta_t_;

  //Modifies the variables associated with change in delta-t
  void delta_change();
};
#endif
