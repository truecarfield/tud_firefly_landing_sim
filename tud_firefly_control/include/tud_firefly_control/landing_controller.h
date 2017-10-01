#ifndef LANDING_CONTROLLER_H
#define LANDING_CONTROLLER_H

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <tf/transform_listener.h>

#include <acado/acado_toolkit.hpp>
#include <acado/acado_gnuplot.hpp>

#include <tud_firefly_control/tud_firefly_common.h>
#include <rotors_control/parameters.h>
#include <rotors_control/parameters_ros.h>

#include <tud_firefly_control/differential_tracker.h>
#include <tud_firefly_control/generic_pid.h>
#include <std_msgs/Float64MultiArray.h>
#include <boost/math/special_functions/sign.hpp>

#include <fstream>
#include <iostream>
#include <ostream>

namespace rotors_control {

// Default values for the landing controller and the Asctec Firefly.
static const Eigen::Vector3d kDefaultPositionGain = Eigen::Vector3d(6, 6, 6);
static const Eigen::Vector3d kDefaultVelocityGain = Eigen::Vector3d(4.7, 4.7, 4.7);
static const Eigen::Vector3d kDefaultAccelerationGain = Eigen::Vector3d(0, 0, 0);
static const Eigen::Vector3d kDefaultAttitudeGain = Eigen::Vector3d(3, 3, 0.035);
static const Eigen::Vector3d kDefaultAngularRateGain = Eigen::Vector3d(0.52, 0.52, 0.025);
static const Eigen::Vector3d kDefaultPositionIntegralGain = Eigen::Vector3d(0, 0, 0);
static const Eigen::Vector3d kDefaultAttitudeIntegralGain = Eigen::Vector3d(0, 0, 0);

class LandingControllerParameters {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LandingControllerParameters()
      : position_gain_(kDefaultPositionGain),
        position_integral_gain_(kDefaultPositionIntegralGain),
        velocity_gain_(kDefaultVelocityGain),
        acceleration_gain_(kDefaultAccelerationGain),
        attitude_gain_(kDefaultAttitudeGain),
        attitude_integral_gain_(kDefaultAttitudeIntegralGain),
        angular_rate_gain_(kDefaultAngularRateGain) {
    calculateAllocationMatrix(rotor_configuration_, &allocation_matrix_);
  }

  Eigen::Matrix4Xd allocation_matrix_;
  Eigen::Vector3d position_gain_;
  Eigen::Vector3d position_integral_gain_;
  Eigen::Vector3d velocity_gain_;
  Eigen::Vector3d acceleration_gain_;
  Eigen::Vector3d attitude_gain_;
  Eigen::Vector3d attitude_integral_gain_;
  Eigen::Vector3d angular_rate_gain_;
  RotorConfiguration rotor_configuration_;
};

class LandingController {
 public:
  LandingController();
  ~LandingController();

  void InitializeParameters(const ros::NodeHandle& pnh);
  void reset();
  void CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities);
  void PID(Eigen::VectorXd* rotor_velocities);
  void HYB(Eigen::VectorXd* rotor_velocities);
  void NMPC(Eigen::VectorXd* rotor_velocities);
  void BK(Eigen::VectorXd* rotor_velocities);
  void LQT(Eigen::VectorXd* rotor_velocities) const;
  void ComputeSystemInputs_LQT(Eigen::VectorXd* rotor_velocities,
                                                      const Eigen::VectorXd current_state,
                                                      const  double t ) const;
  void nmpcInit();

  void SetMaxTiltAngle(const double& max_tilt_angle);
  void SetExternalError(const bool& external_error);
  void SetTest(const bool& test_active); // for test
  void SetUseRdesTracker(const bool& use_R_des_tracker_);
  void SetUseRateTracker(const bool& use_rate_des_tracker_);
  void SetUseCmdVelTracker(const bool& use_cmd_velocity_tracker_);

  void SetActive(const bool& active);
  void SetTime(const ros::Time& t);
  void SetPeriod(const ros::Duration& dt);
  void SetOdometry(const EigenOdometry& odometry);

  // Set commands
  void SetCmdYaw(const double& cmd_yaw);
  void SetTrajectoryPoint(const mav_msgs::EigenTrajectoryPoint& command_trajectory);
  void SetTrajErrorCmdAccelYawPoint(const mav_msgs::EigenTrajectoryPoint& control_error);
  void SetCmdFromTwist(const geometry_msgs::Twist& msg);  // for test
  void GetRotorVelSqr(Eigen::VectorXd& rotor_velocities_sqr); // for test

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 protected:
  LandingControllerParameters controller_parameters_;
  VehicleParameters vehicle_parameters_;

  Eigen::Vector3d normalized_attitude_gain_;
  Eigen::Vector3d normalized_attitude_integral_gain_;
  Eigen::Vector3d normalized_angular_rate_gain_;
  Eigen::MatrixX4d angular_acc_to_rotor_velocities_;
  Eigen::MatrixX4d rotor_velocities_to_angular_acc_;

  Eigen::Vector3d angle_error_integral_;

  DifferentialTracker R_des_tracker_;
  DifferentialTracker angular_rate_des_tracker_;
  DifferentialTracker cmd_velocity_tracker_;
  DifferentialTracker position_error_tracker_;
  DifferentialTracker RPY_tracker_;

  // PID controllers
  GenericPID pidX_;
  GenericPID pidY_;
  GenericPID pidZ_;
  GenericPID pidRoll_;
  GenericPID pidPitch_;
  GenericPID pidYaw_;
  Eigen::Matrix3d c_;

  GenericPID pidVx_;
  GenericPID pidVy_;
  GenericPID pidVz_;
  GenericPID pidYawRate_;

  mav_msgs::EigenTrajectoryPoint command_trajectory_;
  Eigen::VectorXd cmd_position_;    // for test
  Eigen::VectorXd cmd_velocity_;     // for test

  double cmd_roll_;
  double cmd_pitch_;
  double cmd_yaw_;
  double cmd_yaw_rate_;
  Eigen::Vector3d ypsilon_;
  Eigen::Matrix3d cAngle_;
  Eigen::Matrix3d cRate_;
  Eigen::Matrix3d cIntAngle_;
  // double cmd_yaw_; // for test

  EigenOdometry odometry_;

  mav_msgs::EigenTrajectoryPoint traj_error_cmd_accel_yaw_;
  Eigen::Vector3d position_error_;
  Eigen::Vector3d velocity_error_;
  Eigen::Vector3d angle_error_;
  Eigen::Vector3d angle_error_int_;
  Eigen::Vector3d angular_rate_error_;
  Eigen::Vector3d position_integral_error_;

  Eigen::Matrix3d R_;
  double roll_;                        // for test
  double pitch_;                     // for test
  double yaw_;                       // for test
  double yaw_cor_;
  double load_factor_;
  double gravity_N_;
  double gg_N_;
  double PI_;
  double max_tilt_angle_;
  Eigen::Matrix3d W_;

  Eigen::Vector3d J_;

  Eigen::VectorXd rotor_velocities_max_;
  Eigen::VectorXd rotor_velocities_sqr_;

  ros::Time t_;
  ros::Duration dt_;

  bool pid_mode_;
  bool use_cmd_velocity_; // for test
  bool test_active_;
  bool initialized_params_;
  bool controller_active_;
  bool external_error_;

  bool use_R_des_tracker_;
  bool use_rate_des_tracker_;
  bool use_cmd_velocity_tracker_;
  bool use_position_error_tracker_;

  //nmpc attitude control
  ACADO::RealTimeAlgorithm* alg_;
  ACADO::Controller* nmpcController_;
  ACADO::DifferentialState r11_, r12_, r13_,
                                                       r21_, r22_, r23_,
                                                       r31_, r32_, r33_,
                                                       p_, q_, r_;
  ACADO::Control  w0w0_, w1w1_, w2w2_, w3w3_, w4w4_, w5w5_;
  ACADO::OCP* ocp_;
  ACADO::DifferentialEquation f_;

  ros::Publisher test_position_pub_;
  std_msgs::Float64MultiArray test_position_msg_;

  void bkAccel(Eigen::Vector3d* acceleration);

  void PIDAcceleration(Eigen::Vector3d* acceleration);
  void PIDAngularAcc(const Eigen::Vector3d& acceleration, Eigen::Vector3d* angular_acceleration);

  void ComputeDesiredAngularAcc(const Eigen::Vector3d& acceleration,
                                Eigen::Vector3d* angular_acceleration);

  void ComputeDesiredAcceleration(Eigen::Vector3d* acceleration);
};
}

#endif // LANDING_CONTROLLER_H
