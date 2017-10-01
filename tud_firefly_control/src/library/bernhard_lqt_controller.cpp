/*
 * Copyright 2017 Tjarko Möbius, RMR, TU Darmstadt, Germany
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "rotors_control/bernhard_lqt_controller.h"

namespace rotors_control {

BernhardLQTController::BernhardLQTController()
    : initialized_params_(false),
      controller_active_(false) {
  InitializeParameters();
}

BernhardLQTController::~BernhardLQTController() {}

void BernhardLQTController::InitializeParameters() {
  calculateAllocationMatrix(vehicle_parameters_.rotor_configuration_, &(controller_parameters_.allocation_matrix_));
  // To make the tuning independent of the inertia matrix we divide here.
  normalized_attitude_gain_ = controller_parameters_.attitude_gain_.transpose()
      * vehicle_parameters_.inertia_.inverse();
  // To make the tuning independent of the inertia matrix we divide here.
  normalized_angular_rate_gain_ = controller_parameters_.angular_rate_gain_.transpose()
      * vehicle_parameters_.inertia_.inverse();

  Eigen::Matrix4d I;
  I.setZero();
  I.block<3, 3>(0, 0) = vehicle_parameters_.inertia_;
  I(3, 3) = 1;
  angular_acc_to_rotor_velocities_.resize(vehicle_parameters_.rotor_configuration_.rotors.size(), 4);
  // Calculate the pseude-inverse A^{ \dagger} and then multiply by the inertia matrix I.
  // A^{ \dagger} = A^T*(A*A^T)^{-1}
  angular_acc_to_rotor_velocities_ = controller_parameters_.allocation_matrix_.transpose()
      * (controller_parameters_.allocation_matrix_
      * controller_parameters_.allocation_matrix_.transpose()).inverse() * I;
  initialized_params_ = true;
}

void BernhardLQTController::CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) const {
  assert(rotor_velocities);
  assert(initialized_params_);

  rotor_velocities->resize(vehicle_parameters_.rotor_configuration_.rotors.size());
  // Return 0 velocities on all rotors, until the first command is received.
  if (!controller_active_) {
    *rotor_velocities = Eigen::VectorXd::Zero(rotor_velocities->rows());
    return;
  }

  // read the current orientation:
	const Eigen::Matrix3d R_W_B = odometry_.orientation.toRotationMatrix();		// K_World <- K_Body
 	const Eigen::Matrix3d R_S_B = Eigen::Vector3d(1.0,-1.0,-1.0).asDiagonal();	// K_S <- K_Body   (K_S uses NED-convention)
		//printf("%f %f \n", R_W_B(1,0) , R_W_B(0,0) );
		//std::cout << R_W_B << std::endl << std::endl;

  // get the current state:
	const Eigen::Vector3d current_position_W         =         odometry_.position;
	const Eigen::Vector3d current_velocity_W         = R_W_B * odometry_.velocity;
	const Eigen::Vector3d current_kardan_angles      = Eigen::Vector3d(atan2(R_W_B(1,0),R_W_B(0,0) ),
	                                                                    asin(-R_W_B(2,0) ),
	                                                                   atan2( R_W_B(2,1),R_W_B(2,2) ) );
	const Eigen::Vector3d current_angular_velocity_S = R_S_B * odometry_.angular_velocity;

	Eigen::VectorXd current_state_nonconst(12);
	current_state_nonconst << current_position_W , current_velocity_W ,
					                  current_kardan_angles , current_angular_velocity_S; 

	const Eigen::Matrix<double, 12, 1> current_state = current_state_nonconst;
    //std::cout << "\n current_state:\n" << current_state << std::endl;
	

  // read the current time:
	ros::Time current_time = ros::Time::now();
	const double t = current_time.sec;


  // Calculate Inputs from controll law:
  ComputeSystemInputs_LQT( rotor_velocities, current_state, t );

  //ComputeSystemInputs_LQT_GS( rotor_velocities, current_state, t );
	

  // output in console
	std::cout << "\n----\n";

	std::cout << "\n current_time:\n"   << t << std::endl;
	std::cout << "\n current_input:\n"   << *rotor_velocities << std::endl;	

	std::cout << "\n [ x; y; z]:\n"      << current_position_W     << std::endl;
	std::cout << "\n [ vx; vy; vz]:\n"   << current_velocity_W     << std::endl;
	std::cout << "\n [ a; b; c] in °:\n" << 180/M_PI*current_kardan_angles << std::endl;
	std::cout << "\n omega:\n"           << current_angular_velocity_S     << std::endl;

	std::cout << "\n----\n";
  
}

//EigenOdometry defined in  rotors_contol/include/rotors_contol/common.h
void BernhardLQTController::SetOdometry(const EigenOdometry& odometry) {  
  odometry_ = odometry;
}

void BernhardLQTController::SetTrajectoryPoint(
    const mav_msgs::EigenTrajectoryPoint& command_trajectory) {
  command_trajectory_ = command_trajectory;
  controller_active_ = true;
}



// tracking of trajectories described by exogenous system using linear quadratic methods
void BernhardLQTController::ComputeSystemInputs_LQT(Eigen::VectorXd* rotor_velocities, 
                                                    const Eigen::VectorXd current_state,
                                                    const  double t ) const {

  // equilibrium point of linearization: x_R
  Eigen::VectorXd ref_state = Eigen::VectorXd::Zero(current_state.rows());
	ref_state.head(3) = Eigen::Vector3d(0,0,0);		//0,1,2
	ref_state(8)      = 0;                     		// instead of  "ref_state(8) = M_PI;"


  // equilibrium point of linearization: u_R
	double C__T  = vehicle_parameters_.rotor_configuration_.rotors[0].rotor_force_constant;
	double m__HR = vehicle_parameters_.mass_;				//Default: 1.56779; More accurate: 1.5432
	double g     = vehicle_parameters_.gravity_;

	double omega_Ri_R = 1.0 / C__T * sqrt(6.0) * sqrt(C__T * m__HR * g) / 6.0;
    //std::cout << "\n omega_Ri_R = "      << omega_Ri_R << std::endl << std::endl;

	const Eigen::VectorXd u_R = omega_Ri_R * Eigen::VectorXd::Ones(rotor_velocities->rows());

 
 
//*********** original contoller matrices - invarinat ***********************
 /*
//Return to stationary point - (case 2) zyx
  Eigen::MatrixXd K_matrix(6,12);
  Eigen::MatrixXd F_matrix(6,1);
  K_matrix << 
-4.9, -3.1,  12.9, -5.3, -3.8,  26.7, -4.1, -57.4,  31.4,  13.1,  27.8,  19.2, 
 0.2, -5.8,  12.9,  0.9, -6.2,  26.7,  4.1,  1.7,  59.1,  26.1,  0.1, -19.2, 
 5.1, -2.7,  12.9,  6.2, -2.3,  26.7, -4.1,  59.1,  27.8,  13.0, -27.7,  19.2, 
 4.9,  3.1,  12.9,  5.3,  3.8,  26.7,  4.1,  57.4, -31.4, -13.1, -27.8, -19.2, 
-0.2,  5.8,  12.9, -0.9,  6.2,  26.7, -4.1, -1.7, -59.1, -26.1, -0.1,  19.2, 
-5.1,  2.7,  12.9, -6.2,  2.3,  26.7,  4.1, -59.1, -27.8, -13.0,  27.7, -19.2;

  F_matrix << 
 27.7, 
 27.4, 
 38.5, 
 49.8, 
 50.0, 
 39.0;

  double exo_state = 1;
  */
//*********** end of original contoller matrices - invarinat *****************


//*********** dependent controller matrices - invariant
// your time variant reference: x_ref, y_ref, zref, alpha_ref, beta_ref, gamma_ref

Eigen::VectorXd your_ref(6);
//intialize here:   x_ref, y_ref, zref, alpha_ref, beta_ref, gamma_ref;

Eigen::VectorXd operating_point(6);
operating_point << 0.0, 0.0, 0.0,  0.0, 0.0, M_PI;


Eigen::MatrixXd O(6,1);
O = (your_ref - operating_point); // conversion vec -> mat !?

 
/*
	B, C,  D_d, E_d, S,  R, Q_y,  sLQB.Pi_x, sLQB.Pbar   exist in bsp_hexa_zyx.m  (case 2!)
    Abar  exist in the nested function  LQBernhard.m
*/

	Eigen::MatrixXd C(6,12);
	C <<	1, 0, 0,  0, 0, 0,  0, 0, 0,  0, 0, 0,
			0, 1, 0,  0, 0, 0,  0, 0, 0,  0, 0, 0,
			0, 0, 1,  0, 0, 0,  0, 0, 0,  0, 0, 0,
			0, 0, 0,  0, 0, 0,  1, 0, 0,  0, 0, 0,
			0, 0, 0,  0, 0, 0,  0, 1, 0,  0, 0, 0,
			0, 0, 0,  0, 0, 0,  0, 0, 1,  0, 0, 0; 
			
	Eigen::MatrixXd Q_y(6,6);
	Q_y <<	1, 0, 0,  0, 0, 0,
			0, 1, 0,  0, 0, 0,
			0, 0,10,  0, 0, 0,
			0, 0, 0,  1, 0, 0,
			0, 0, 0,  0, 1, 0,
			0, 0, 0,  0, 0, 1;
	Q_y *= 100;

	Eigen::MatrixXd D_d(6,1);
	D_d <<	0, 0, 0,  0, 0, 0;

Eigen::MatrixXd S_w(6,1);
S_w = C.transposeInPlace() * Q_y * (D_d - O);

	Eigen::MatrixXd Abar(12,12);
	Abar <<  0,         0,         0,    1.0000,         0,         0,         0,         0,         0,         0,         0,         0,
			 0,         0,         0,         0,    1.0000,         0,         0,         0,         0,         0,         0,         0,
			 0,         0,         0,         0,         0,    1.0000,         0,         0,         0,         0,         0,         0,
			 0,         0,         0,   -0.1703,         0,         0,         0,    9.8100,         0,         0,    0.0063,         0,
			 0,         0,         0,         0,   -0.1703,         0,         0,         0,   -9.8100,    0.0063,         0,         0,
		0.0000,    0.0000,   -0.4662,    0.0000,    0.0000,   -0.9656,    0.0000,    0.0000,   -0.0000,   -0.0000,    0.0000,    0.0000,
			 0,         0,         0,         0,         0,         0,         0,         0,         0,         0,         0,   -1.0000,
			 0,         0,         0,         0,         0,         0,         0,         0,         0,         0,   -1.0000,         0,
			 0,         0,         0,         0,         0,         0,         0,         0,         0,    1.0000,         0,         0,
	   -0.0386,    0.9945,    0.0000,   -0.0535,    1.3424,   -0.0000,   -0.0000,   -0.2901,  -10.1901,   -4.5130,   -0.0251,    0.0000,
		0.7531,    0.0293,   -0.0000,    1.0750,    0.0433,   -0.0000,    0.0000,    8.7744,   -0.2723,   -0.0098,   -4.1878,   -0.0000,
		0.0000,    0.0000,    0.0000,    0.0000,   -0.0000,    0.0000,    0.0373,    0.0000,    0.0000,    0.0000,   -0.0000,   -0.3000;
	
	double S = 0.0;
	
	
	Eigen::MatrixXd Pbar(12,12);
	Pbar <<	0.0163,    0.0000,    0.0000,    0.0119,   -0.0000,   -0.0000,    0.0000,    0.0556,    0.0016,    0.0004,   -0.0133,   -0.0000,  //Komma !!
			0.0000,    0.0153,    0.0000,    0.0002,    0.0104,   -0.0000,   -0.0000,    0.0024,   -0.0454,   -0.0100,   -0.0005,    0.0000,
			0.0000,    0.0000,    0.2071,   -0.0000,    0.0000,    0.2145,   -0.0000,   -0.0000,   -0.0000,   -0.0000,    0.0000,   -0.0000,
			0.0119,    0.0002,   -0.0000,    0.0125,    0.0001,   -0.0000,    0.0000,    0.0686,    0.0021,    0.0015,   -0.0152,   -0.0000,
		   -0.0000,    0.0104,    0.0000,    0.0001,    0.0104,   -0.0000,   -0.0000,    0.0034,   -0.0530,   -0.0107,   -0.0020,    0.0000,
		   -0.0000,   -0.0000,    0.2145,   -0.0000,   -0.0000,    0.4442,   -0.0000,   -0.0000,    0.0000,    0.0000,    0.0000,    0.0000,
			0.0000,   -0.0000,   -0.0000,    0.0000,   -0.0000,   -0.0000,    0.0805,    0.0000,    0.0000,    0.0000,   -0.0000,   -0.2684,
			0.0556,    0.0024,   -0.0000,    0.0686,    0.0034,   -0.0000,    0.0000,    0.4982,   -0.0029,    0.0029,   -0.1545,   -0.0000,
			0.0016,   -0.0454,   -0.0000,    0.0021,   -0.0530,    0.0000,    0.0000,   -0.0029,    0.3595,    0.1029,    0.0048,   -0.0000,
			0.0004,   -0.0100,   -0.0000,    0.0015,   -0.0107 ,   0.0000,    0.0000,    0.0029,    0.1029,    0.0455,    0.0002,   -0.0000,
		   -0.0133,   -0.0005,    0.0000,   -0.0152,   -0.0020 ,   0.0000,   -0.0000,   -0.1545,    0.0048,    0.0002,    0.0736,    0.0000,
		   -0.0000,    0.0000,   -0.0000,   -0.0000,    0.0000 ,   0.0000,   -0.2684,   -0.0000,   -0.0000,   -0.0000,    0.0000,    1.2649;
	Pbar *= 10000;
	
	Eigen::MatrixXd E_d(12,1);
	E_d <<	0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0;
		
//Pi_v = solvePi_v(Abar,S, Pbar,E_d,S_w) 			// Solve the sylvester equation
//Pi_v * S + (Abar.') * Pi_v = - (Pbar*E_d + S_w)	// In Matlab exists a function to do that, but in C++ ...   

// This is a problem, but in this case the equation simplifies:
// S=0  =>  Pi_v = - (Abar.')^-1 * (Pbar*E_d + S_w)
Pi_v = - Abar.transposeInPlace().inverse() * (Pbar*E_d + S_w)

	Eigen::MatrixXd R =  MatrixXd::Identity(6,6);

	Eigen::MatrixXd B(12,6);
	B <<   0.0,       0.0,       0.0,       0.0,       0.0,       0.0,
		   0.0,       0.0,       0.0,       0.0,       0.0,       0.0,
		   0.0,       0.0,       0.0,       0.0,       0.0,       0.0,
		   0.0,       0.0,       0.0,       0.0,       0.0,       0.0,
		   0.0 ,      0.0,       0.0,       0.0,       0.0,       0.0,
		0.0060 ,   0.0060,    0.0060,    0.0060,    0.0060,    0.0060,
		   0.0,       0.0,       0.0,       0.0,       0.0,       0.0,
		   0.0,       0.0,       0.0,       0.0,       0.0,       0.0,
		   0.0,       0.0,       0.0,       0.0,       0.0,       0.0,
		0.0287,    0.0575,    0.0287,   -0.0287,   -0.0575,   -0.0287,
		0.0377,       0.0,   -0.0377,   -0.0377,       0.0,    0.0377,
		0.0015,   -0.0015,    0.0015,   -0.0015,    0.0015,   -0.0015;
	
	
Eigen::MatrixXd F_matrix(6,1);
// F_matrix = - R.inverse() * B.transposeInPlace() * ( (Pbar - Palpha)*Pi_x + Pi_v );   simpification: alpha = 0  =>  Pbar = Palpha
F_matrix = - R.inverse() * B.transposeInPlace() * Pi_v;

 

  Eigen::MatrixXd K_matrix(6,12);
  K_matrix << 
-4.9, -3.1,  12.9, -5.3, -3.8,  26.7, -4.1, -57.4,  31.4,  13.1,  27.8,  19.2, 
 0.2, -5.8,  12.9,  0.9, -6.2,  26.7,  4.1,  1.7,  59.1,  26.1,  0.1, -19.2, 
 5.1, -2.7,  12.9,  6.2, -2.3,  26.7, -4.1,  59.1,  27.8,  13.0, -27.7,  19.2, 
 4.9,  3.1,  12.9,  5.3,  3.8,  26.7,  4.1,  57.4, -31.4, -13.1, -27.8, -19.2, 
-0.2,  5.8,  12.9, -0.9,  6.2,  26.7, -4.1, -1.7, -59.1, -26.1, -0.1,  19.2, 
-5.1,  2.7,  12.9, -6.2,  2.3,  26.7,  4.1, -59.1, -27.8, -13.0,  27.7, -19.2;

  double exo_state = 1;
  
  
	// Calculate the rotor velocities (inputs):
	*rotor_velocities = u_R - K_matrix * ( current_state - ref_state ) + F_matrix * exo_state;


}

}
