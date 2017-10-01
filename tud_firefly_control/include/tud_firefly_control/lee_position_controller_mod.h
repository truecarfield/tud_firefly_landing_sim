#ifndef LEE_POSITION_CONTROLLER_MOD_H
#define LEE_POSITION_CONTROLLER_MOD_H

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <tf/transform_listener.h>

#include <tud_firefly_control/tud_firefly_common.h>
#include <rotors_control/parameters.h>

#include <tud_firefly_control/differential_tracker.h>
#include <tud_firefly_control/generic_pid.h>
#include <std_msgs/Float64MultiArray.h>

#include <fstream>
#include <iostream>
#include <ostream>

namespace rotors_control {

// Default values for the lee position controller and the Asctec Firefly.
static const Eigen::Vector3d kDefaultPositionGain = Eigen::Vector3d(6, 6, 6);
static const Eigen::Vector3d kDefaultVelocityGain = Eigen::Vector3d(4.7, 4.7, 4.7);
static const Eigen::Vector3d kDefaultAttitudeGain = Eigen::Vector3d(3, 3, 0.035);
static const Eigen::Vector3d kDefaultAngularRateGain = Eigen::Vector3d(0.52, 0.52, 0.025);
static const Eigen::Vector3d kDefaultPositionIntegralGain = Eigen::Vector3d(6, 6, 6);
static const Eigen::Vector3d kDefaultAttitudeIntegralGain = Eigen::Vector3d(6, 6, 6);

class LeePositionControllerModParameters {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LeePositionControllerModParameters()
      : position_gain_(kDefaultPositionGain),
        position_integral_gain_(kDefaultPositionIntegralGain),
        velocity_gain_(kDefaultVelocityGain),
        attitude_gain_(kDefaultAttitudeGain),
        attitude_integral_gain_(kDefaultAttitudeIntegralGain),
        angular_rate_gain_(kDefaultAngularRateGain) {
    calculateAllocationMatrix(rotor_configuration_, &allocation_matrix_);
  }

  Eigen::Matrix4Xd allocation_matrix_;
  Eigen::Vector3d position_gain_;
  Eigen::Vector3d position_integral_gain_;
  Eigen::Vector3d velocity_gain_;
  Eigen::Vector3d attitude_gain_;
  Eigen::Vector3d attitude_integral_gain_;
  Eigen::Vector3d angular_rate_gain_;
  RotorConfiguration rotor_configuration_;
};

class LeePositionControllerMod {
 public:
  LeePositionControllerMod();
  ~LeePositionControllerMod();

  void InitializeParameters(const ros::NodeHandle& pnh);
  void reset();
  void CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities);

  void SetTest(const bool& test_active);
  void SetCmd(const Eigen::Vector3d& cmd_position, const Eigen::Vector3d& cmd_velocity, const double& cmd_yaw);
  void SetCmdFromTwist(const geometry_msgs::Twist& msg);
  void SetError(const Eigen::Vector3d& position_error, const Eigen::Vector3d& twist_error, const double& yaw_error);
  void SetActive(const bool& active);
  void SetTime(const ros::Time& t);
  void SetPeriod(const ros::Duration& dt);
  void SetOdometry(const EigenOdometry& odometry);
  void SetTrajectoryPoint(
    const mav_msgs::EigenTrajectoryPoint& command_trajectory);

  LeePositionControllerModParameters controller_parameters_;
  VehicleParameters vehicle_parameters_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 protected:
  Eigen::Vector3d normalized_attitude_gain_;
  Eigen::Vector3d normalized_attitude_integral_gain_;
  Eigen::Vector3d normalized_angular_rate_gain_;
  Eigen::MatrixX4d angular_acc_to_rotor_velocities_;
  Eigen::MatrixX4d rotor_velocities_to_angular_acc_;

  Eigen::Vector3d angle_error_integral_;

  DifferentialTracker R_des_tracker_;
  DifferentialTracker angular_rate_des_tracker_;
  DifferentialTracker cmd_velocity_tracker_;
  DifferentialTracker cmd_position_tracker_;

  GenericPID pidZ_, pidVz_, pidYaw_, pidYawRate_;

  mav_msgs::EigenTrajectoryPoint command_trajectory_;
  Eigen::VectorXd cmd_position_;
  Eigen::VectorXd cmd_velocity_;
  Eigen::VectorXd cmd_accel_N_;
  double cmd_roll_;
  double cmd_pitch_;
  double cmd_yaw_;

  EigenOdometry odometry_;

  Eigen::Vector3d position_error_;
  Eigen::Vector3d velocity_error_;
  Eigen:: Vector3d angle_error_;
  Eigen::Vector3d angular_rate_error_;
  Eigen::Vector3d position_integral_error_;

  Eigen::Vector3d J_;

  Eigen::Matrix3d R_;
  double roll_;
  double pitch_;
  double yaw_;
  double load_factor_;
  double gravity_N_;
  double gg_N_;
  double PI_;

  // NMPC states
  std::vector<Eigen::Quaterniond> X1_;
  std::vector<Eigen::Vector3d> X2_;
  std::vector<Eigen::VectorXd> U_;
  //Eigen::Quaterniond X1_, X1_1_, X1_2_, X1_3_, X1_4_;
  //Eigen::Vector3d X2_, X2_1_, X2_2_, X2_3_, X2_4_;
  //Eigen::VectorXd U_, U_1_, U_2_, U_3_, U_4_;

  Eigen::VectorXd rotor_velocities_max_;
  Eigen::VectorXd rotor_velocities_;

  ros::Time t_;
  ros::Duration dt_;

  bool test_active_;
  bool initialized_params_;
  bool controller_active_;
  bool external_linear_error_;
  bool external_angular_error_;

  ros::Publisher test_position_pub_;
  std_msgs::Float64MultiArray test_position_msg_;

  void ComputeDesiredAngularAcc(const Eigen::Vector3d& acceleration,
                                Eigen::Vector3d* angular_acceleration);
  void ComputeDesiredAcceleration(Eigen::Vector3d* acceleration);
};
}

#endif // LEE_POSITION_CONTROLLER_MOD_H
