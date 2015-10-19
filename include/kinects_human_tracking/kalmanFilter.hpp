//| This file is a part of the sferes2 framework.
//| Copyright 2015, ISIR / Universite Pierre et Marie Curie (UPMC)
//| Main contributor(s): Jimmy Da Silva, jimmy.dasilva@isir.upmc.fr
//|
//| This software is a computer program whose purpose is to facilitate
//| experiments in evolutionary computation and evolutionary robotics.
//|
//| This software is governed by the CeCILL license under French law
//| and abiding by the rules of distribution of free software. You
//| can use, modify and/ or redistribute the software under the terms
//| of the CeCILL license as circulated by CEA, CNRS and INRIA at the
//| following URL "http://www.cecill.info".
//|
//| As a counterpart to the access to the source code and rights to
//| copy, modify and redistribute granted by the license, users are
//| provided only with a limited warranty and the software's author,
//| the holder of the economic rights, and the successive licensors
//| have only limited liability.
//|
//| In this respect, the user's attention is drawn to the risks
//| associated with loading, using, modifying and/or developing or
//| reproducing the software by the user in light of its specific
//| status of free software, that may mean that it is complicated to
//| manipulate, and that also therefore means that it is reserved for
//| developers and experienced professionals having in-depth computer
//| knowledge. Users are therefore encouraged to load and test the
//| software's suitability as regards their requirements in conditions
//| enabling the security of their systems and/or data to be ensured
//| and, more generally, to use and operate it in the same conditions
//| as regards security.
//|
//| The fact that you are presently reading this means that you have
//| had knowledge of the CeCILL license and that you accept its terms.

#ifndef KALMAN_FILTER
#define KALMAN_FILTER

#include <iostream>
#include <Eigen/Dense>

/**
   Kalman Filter with state (x,y,vx,vy) and measurement (x,y)
**/

using namespace std;

class KalmanFilter{

public:
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
