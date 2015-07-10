#ifndef KALMAN_FILTER
#define KALMAN_FILTER

#include <iostream>
#include <Eigen/Dense>

/**
   Kalman Filter with state (x,y,vx,vy) and measurement (x,y)
**/

typedef Eigen::Matrix<float, 6, 1> State;
using namespace std;


class KalmanFilter{

public:
  //TODO: What parameters in constructor ?
  KalmanFilter();
  
  void init(Eigen::Vector2f jerk_std,
	  Eigen::Vector2f measur_std,
	  float delta_t,
	  Eigen::Matrix<float, 6,1> x_k1,
	  Eigen::Matrix<float, 6, 6> init_cov = Eigen::Matrix<float, 6, 6>::Zero());

  //if delta-t not known pass <=0
  void predict(float delta_t);
  
  void correct(Eigen::Vector2f z_k);
  
  //estimate new state vector given observation and dt
  void estimate(Eigen::Vector2f obs, float delta_t, Eigen::Matrix<float, 6, 1> &est);
 

private:
  Eigen::Matrix<float, 6,1> x_k_p_, x_k_n_; //previous and next state
  Eigen::Matrix<float, 6,1> x_k1_; // Xk-1
  Eigen::Matrix<float, 6, 6> A_, Q_, P_k_p_, P_k_n_, P_k1_;
  Eigen::Matrix2f R_, sigma_jerk_; //measurement and process noises
  Eigen::Matrix<float, 2, 6> H_;
  Eigen::Matrix<float, 6, 2> K_;
  float delta_t_;

  //Modifies the variables associated with change in delta-t
  void delta_change();
};
#endif
