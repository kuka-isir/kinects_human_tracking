#include <kinects_human_tracking/kalmanFilter3Pos3Vel.hpp>

/**
   Kalman Filter with state (x,y,z,vx,vy,vz) and measurement (x,y,z)
**/

KalmanFilter::KalmanFilter(){}

void KalmanFilter::init(Eigen::Vector3f jerk_std, 
			Eigen::Vector3f measur_std,
			float delta_t,
			Eigen::Matrix<float, 9, 1> x_k1,
			Eigen::Matrix<float, 9, 9> init_cov)
{
  x_k_n_ =  x_k1;
  P_k_n_ = init_cov;

  H_.fill(0.);
  H_(0,0) = 1.;
  H_(1,1) = 1.;
  H_(2,2) = 1.;

  R_.fill(0.);
  R_(0,0) = pow(measur_std(0),2);
  R_(1,1) = pow(measur_std(1),2);
  R_(2,2) = pow(measur_std(2),2);

  sigma_jerk_ = Eigen::Matrix3f::Zero();
  sigma_jerk_(0,0) = pow(jerk_std(0),2);
  sigma_jerk_(1,1) = pow(jerk_std(1),2);
  sigma_jerk_(2,2) = pow(jerk_std(2),2);

  if (delta_t <=0)
    delta_t_ = 1/30; // We aim for 30 FPS
  else  
    delta_t_ = delta_t;
  
  delta_change();
}

void KalmanFilter::delta_change(){
  
  float dt2_2 = (pow(delta_t_,2))/2;
  
  //Recompute A, state transition
  A_ = Eigen::Matrix<float, 9, 9>::Identity();
  A_ << 1., 0.,   0.,     delta_t_,   0.,     0.,    dt2_2,     0.,     0.,
	0., 1.,   0.,       0.,     delta_t_, 0.,      0.,     dt2_2,   0.,
	0., 0.,   1.,       0.,       0.,   delta_t_,  0.,      0.,    dt2_2,
	0., 0.,   0.,       1.,       0.,     0.,    delta_t_,  0.,     0.,
	0., 0.,   0.,       0.,       1.,     0.,      0.,    delta_t_, 0.,
	0., 0.,   0.,       0.,       0.,     1.,      0.,      0.,     delta_t_,
	0., 0.,   0.,       0.,       0.,     0.,      1.,      0.,     0.,
	0., 0.,   0.,       0.,       0.,     0.,      0.,      1.,     0.,
	0., 0.,   0.,       0.,       0.,     0.,      0.,      0.,     1.;
     
  //Recompute Q, covariance process noise
  Eigen::Matrix<float, 9, 3>  G;
  G.fill(0.);
  //Process noise only in the highest order term
  G(6,0) = delta_t_;
  G(7,1) = delta_t_;
  G(8,2) = delta_t_;
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

void KalmanFilter::correct(Eigen::Vector3f z_k){
  
  //Compute Kalman gain
  K_ = P_k_p_ * H_.transpose() * (H_ * P_k_p_ * H_.transpose() + R_).inverse();
  
  //Update estimate with observation
  x_k_n_ = x_k_p_ + K_* (z_k - H_ * x_k_p_);

  //Update error cov
  Eigen::Matrix<float, 9, 9> I9 = Eigen::MatrixXf::Identity(9,9);
  P_k_n_ = ( I9 - (K_ * H_)) * P_k_p_;
  
}

void KalmanFilter::estimate(Eigen::Vector3f obs, float delta_t, Eigen::Matrix<float, 9, 1> &est){
  
  //previous estimate is k-1th estimate now
  x_k1_ = x_k_n_;
  P_k1_ = P_k_n_;

  predict(delta_t);
  correct(obs);

  est = x_k_n_;
  
}