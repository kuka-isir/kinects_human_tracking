#include <kinects_human_tracking/kalmanFilter.hpp>

/**
   Kalman Filter with state (x,y,vx,vy) and measurement (x,y)
**/

KalmanFilter::KalmanFilter(){}

void KalmanFilter::init(Eigen::Vector2f jerk_std, 
			Eigen::Vector2f measur_std,
			float delta_t,
			Eigen::Matrix<float, 6, 1> x_k1,
			Eigen::Matrix<float, 6, 6> init_cov)
{
  x_k_n_ =  x_k1;
  P_k_n_ = init_cov;

  H_.fill(0.);
  H_(0,0) = 1.;
  H_(1,1) = 1.;

  R_.fill(0.);
  R_(0,0) = pow(measur_std(0),2);
  R_(1,1) = pow(measur_std(1),2);

  sigma_jerk_ = Eigen::Matrix2f::Zero();
  sigma_jerk_(0,0) = pow(jerk_std(0),2);
  sigma_jerk_(1,1) = pow(jerk_std(1),2);

  delta_t_ = delta_t;
  delta_change();
}

void KalmanFilter::delta_change(){
  
  float dt2_2 = (pow(delta_t_,2))/2;
  
  //Recompute A, state transition
  A_ = Eigen::Matrix<float, 6, 6>::Identity();
  A_ << 1., 0., delta_t_, 0., dt2_2, 0.,
    0., 1., 0., delta_t_, 0., dt2_2,
    0., 0., 1., 0., delta_t_, 0.,
    0., 0., 0., 1., 0., delta_t_,
    0., 0., 0., 0., 1., 0.,
    0., 0., 0., 0., 0., 1.;
     
  //Recompute Q, covariance process noise
  Eigen::Matrix<float, 6, 2>  G;
  G.fill(0.);
  //Process noise only in the highest order term
  G(4,0) = delta_t_;
  G(5,1) = delta_t_;
  Q_ = G * sigma_jerk_ * G.transpose();
  
}

void KalmanFilter::predict(float delta_t){
  
  //if exact delta-t not known use default
  if (delta_t<=0)
    delta_t = delta_t_;
  
  if (delta_t!=delta_t_){
    delta_t_ = delta_t;
    delta_change();
  }

  //State prediction
  x_k_p_ = A_*x_k1_;
  
  //Error covariance prediction
  P_k_p_ = A_ * P_k1_ * A_.transpose() + Q_;
  
}

void KalmanFilter::correct(Eigen::Vector2f z_k){
  
  //Compute Kalman gain
  K_ = P_k_p_ * H_.transpose() * (H_ * P_k_p_ * H_.transpose() + R_).inverse();
  
  //Update estimate with observation
  x_k_n_ = x_k_p_ + K_* (z_k - H_ * x_k_p_);

  //Update error cov
  Eigen::Matrix<float, 6, 6> I6 = Eigen::MatrixXf::Identity(6,6);
  P_k_n_ = ( I6 - (K_ * H_)) * P_k_p_;
  
}

void KalmanFilter::estimate(Eigen::Vector2f obs, float delta_t, Eigen::Matrix<float, 6, 1> &est){
  
  //previous estimate is k-1th estimate now
  x_k1_ = x_k_n_;
  P_k1_ = P_k_n_;

  predict(delta_t);
  correct(obs);

  est = x_k_n_;
  
}