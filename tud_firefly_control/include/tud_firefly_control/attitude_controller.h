#ifndef FIREFLY_ATTITUDE_CONTROLLER_H
#define FIREFLY_ATTITUDE_CONTROLLER_H

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>

#include "tud_firefly_control/generic_pid.h"

#include "tud_firefly_control/tud_firefly_common.h"
#include <rotors_control/parameters_ros.h>

#include <fstream>
#include <iostream>
#include <ostream>

namespace rotors_control {

static const Eigen::Vector3d kDefaultAttitudeGainAttitudeControl  = Eigen::Vector3d(3, 3, 0.035);
static const Eigen::Vector3d kDefaultAngularRateGainAttitudeControl  = Eigen::Vector3d(0.52, 0.52, 0.025);

class  FireflyAttitudeControllerParameters{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FireflyAttitudeControllerParameters()
   : attitude_gain_(kDefaultAttitudeGainAttitudeControl ),
     angular_rate_gain_(kDefaultAngularRateGainAttitudeControl)
  {
    calculateAllocationMatrix(rotor_configuration_, &allocation_matrix_);
  }

Eigen::Matrix4Xd allocation_matrix_;
Eigen::Vector3d attitude_gain_;
Eigen::Vector3d angular_rate_gain_;
RotorConfiguration rotor_configuration_;
};

class FireflyAttitudeController {
 public:
  FireflyAttitudeController();
   ~FireflyAttitudeController();

  void InitializeParams(ros::NodeHandle& pnh);
  void reset();

  void SetOdometry(const EigenOdometry& odometry_msg);
  void SetError(const Eigen::Vector3d& p_error, const Eigen::Vector3d& vel_error);
  void SetCmd(const std::vector<double>& cmds);
  void SetTime(const ros::Time& t);
  void SetPeriod(const ros::Duration& dt);

  void CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities);
  void ControllerActivation(const bool& active);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 protected:
  void ComputeDesiredAcceleration(Eigen::Vector3d* acceleration);
  void ComputeDesiredAngularAcc(Eigen::Vector3d *angular_acceleration);

  FireflyAttitudeControllerParameters controller_parameters_;
  VehicleParameters vehicle_parameters_;

  GenericPID pidZ_;

  ros::Duration dt_;

  double gravity_N_;
  double cmd_yaw_rate_;
  double cmd_roll_;
  double cmd_pitch_;
  double cmd_twist_N_;
  double cmd_altitude_;

  double position_error_;
  double twist_error_;
  Eigen::Vector3d angle_error_;
  Eigen::Vector3d angular_rate_error_;

  Eigen::Matrix3d R_;
  double roll_;
  double pitch_;
  double yaw_;
  double load_factor_;

  bool initialized_params_;
  bool controller_active_;
  bool external_linear_error_;
  bool external_angular_error_;

  bool bool_test_;

  Eigen::Vector3d normalized_attitude_gain_;
  Eigen::Vector3d normalized_angular_rate_gain_;
  Eigen::MatrixX4d angular_acc_to_rotor_velocities_;
  Eigen::Matrix4Xd rotor_velocities_to_UAV_accelerations_;

  EigenOdometry odometry_;

};

}

#endif // FIREFLY_ATTITUDE_CONTROLLER_H
