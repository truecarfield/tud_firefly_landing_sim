#include "tud_firefly_control/lee_position_controller_mod.h"

namespace rotors_control {

static constexpr double kDefaulMaxRotVelocity = 838.0;

LeePositionControllerMod::LeePositionControllerMod()
    : initialized_params_(false),
      controller_active_(false),
      test_active_(false),
      external_linear_error_(false),
      gravity_N_(-9.80665),
      gg_N_(gravity_N_*gravity_N_),
      PI_(3.141592653589793)
{
  cmd_accel_N_.resize(3);
  cmd_position_.resize(3);
  cmd_velocity_.resize(3);

  cmd_position_.setZero();
  cmd_velocity_.setZero();
  cmd_accel_N_.setZero();

  X1_.resize(5);
  X2_.resize(5);
  U_.resize(5);

  rotor_velocities_.resize(6);
  rotor_velocities_.setZero();
  rotor_velocities_max_.resize(6);
  for (int i=0;i<6;i++)
  {
    rotor_velocities_max_(i) = kDefaulMaxRotVelocity;
  }
}

LeePositionControllerMod::~LeePositionControllerMod() {}

void LeePositionControllerMod::InitializeParameters(const ros::NodeHandle& pnh) {

  ros::NodeHandle nh;

  test_position_pub_= nh.advertise<std_msgs::Float64MultiArray>("test_position",1);
  test_position_msg_.data.resize(3);

  calculateAllocationMatrix(vehicle_parameters_.rotor_configuration_, &(controller_parameters_.allocation_matrix_));
  // To make the tuning independent of the inertia matrix we divide here.
  normalized_attitude_gain_ = controller_parameters_.attitude_gain_.transpose()
      * vehicle_parameters_.inertia_.inverse();
  normalized_attitude_integral_gain_ =  controller_parameters_.attitude_integral_gain_.transpose()
      * vehicle_parameters_.inertia_.inverse();
  // To make the tuning independent of the inertia matrix we divide here.
  normalized_angular_rate_gain_ = controller_parameters_.angular_rate_gain_.transpose()
      * vehicle_parameters_.inertia_.inverse();

  J_(0) = vehicle_parameters_.inertia_(0,0);
  J_(1) = vehicle_parameters_.inertia_(1,1);
  J_(2) = vehicle_parameters_.inertia_(2,2);

  Eigen::Matrix4d inertia_mass_matrix_;
  inertia_mass_matrix_.setZero();
  inertia_mass_matrix_.block<3, 3>(0, 0) = vehicle_parameters_.inertia_;
  inertia_mass_matrix_(3, 3) = vehicle_parameters_.mass_;
  rotor_velocities_to_angular_acc_.resize(4, 6);
  rotor_velocities_to_angular_acc_.block<3, 6>(0, 0) = vehicle_parameters_.inertia_.inverse() * controller_parameters_.allocation_matrix_.block<3, 6>(0, 0);
  rotor_velocities_to_angular_acc_.block<1, 6>(3, 0) = inertia_mass_matrix_.block<1, 6>(3, 0)/vehicle_parameters_.mass_;


  // Initialize the PID controllers
  pidZ_.initialize(ros::NodeHandle(pnh, "linear/z"));
  pidVz_.initialize(ros::NodeHandle(pnh, "linear/Vz"));
  pidYaw_.initialize(ros::NodeHandle(pnh, "angular/yaw"));
  pidYawRate_.initialize(ros::NodeHandle(pnh, "angular/r"));

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

  R_des_tracker_.init(pnh, "R_des");
  angular_rate_des_tracker_.init(pnh, "angular_rate_des");
  cmd_velocity_tracker_.init(pnh, "cmd_velocity");
  cmd_position_tracker_.init(pnh, "cmd_position");

  initialized_params_ = true;
 }

void LeePositionControllerMod::SetTime(const ros::Time& t) {
 t_ = t;
}

void LeePositionControllerMod::SetPeriod(const ros::Duration &dt)
{
  dt_ = dt;
}

void LeePositionControllerMod::SetCmd(const Eigen::Vector3d& cmd_position, const Eigen::Vector3d & cmd_velocity, const double& cmd_yaw)
{
  cmd_position_ = cmd_position;
  cmd_velocity_ = cmd_velocity;
  cmd_yaw_ = cmd_yaw;
}

void LeePositionControllerMod::SetCmdFromTwist(const geometry_msgs::Twist& msg)
{
  /* cmd_velocity_ << cos(yaw_)*msg.linear.x - sin(yaw_)*msg.linear.y,
                                      sin(yaw_)*msg.linear.x + cos(yaw_)*msg.linear.y,
                                      msg.linear.z;
  cmd_position_ += cmd_velocity_*dt_.toSec();
  cmd_yaw_ += msg.angular.z*dt_.toSec(); // */

  if (controller_active_)
  {
    cmd_position_ << 0.0, 0.0, 1.0;
    cmd_position_tracker_.diff(cmd_position_, cmd_velocity_, dt_.toSec());
    ROS_INFO_ONCE("t_an position_ is: [%lf]", t_.toSec());
  }
  else
  {
    cmd_position_ << 0.0, 0.0, 0.0;
  }

}

void LeePositionControllerMod::SetError(const Eigen::Vector3d& position_error, const Eigen::Vector3d& velocity_error, const double& yaw_error)
{
  position_error_ = position_error;
  velocity_error_ = velocity_error;
  angle_error_(2) = yaw_error;
}

void LeePositionControllerMod::reset() {}

void LeePositionControllerMod::CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) {
  assert(rotor_velocities);
  assert(initialized_params_);

  rotor_velocities->resize(vehicle_parameters_.rotor_configuration_.rotors.size());
  // Return 0 velocities on all rotors, until the first command is received.
  if (!controller_active_) {
    *rotor_velocities = Eigen::VectorXd::Zero(rotor_velocities->rows());
    return;
  }

  Eigen::Vector3d acceleration;
  ComputeDesiredAcceleration(&acceleration);

  Eigen::Vector3d angular_acceleration;
  ComputeDesiredAngularAcc(acceleration, &angular_acceleration);

  // Project thrust onto body z axis.
  // double thrust = -vehicle_parameters_.mass_ * acceleration.dot(odometry_.orientation.toRotationMatrix().col(2));

  Eigen::Vector3d accel;
  accel << 0.0, 0.0, -gravity_N_;
  double thrust = vehicle_parameters_.mass_ * (-gravity_N_ +  (gravity_N_ - acceleration(2))* load_factor_);

  Eigen::Vector4d angular_acceleration_thrust;
  angular_acceleration_thrust.block<3, 1>(0, 0) = angular_acceleration;
  angular_acceleration_thrust(3) = thrust;

  *rotor_velocities = angular_acc_to_rotor_velocities_ * angular_acceleration_thrust;
  *rotor_velocities = rotor_velocities->cwiseMax(Eigen::VectorXd::Zero(rotor_velocities->rows()));
  *rotor_velocities = rotor_velocities->cwiseSqrt();
  *rotor_velocities = rotor_velocities->cwiseMin(rotor_velocities_max_);
    rotor_velocities_ = *rotor_velocities;

  /* Eigen::MatrixXd test(6, 6);
  test = rotor_velocities->asDiagonal();
  Eigen::FullPivLU<Eigen::MatrixXd> test_(test);
  if (test_.rank() < 6)
  {
                    //"rotor_velocities has 0 element: "<<std::endl
                    <<*rotor_velocities<<std::endl
                     <<"corresponding accels are:"<<std::endl
                     <<angular_acceleration_thrust<<std::endl;
  } */
}

void  LeePositionControllerMod::SetActive(const bool& active) {
  controller_active_ = active;
}

void LeePositionControllerMod::SetTest(const bool &test_active)
{
  test_active_ = test_active;
}

void LeePositionControllerMod::SetOdometry(const EigenOdometry& odometry) {
  odometry_ = odometry;
  R_ = odometry_.orientation.toRotationMatrix();

  double w = odometry_.orientation.w();
  double x =  odometry_.orientation.x();
  double y =  odometry_.orientation.y();
  double z = odometry_.orientation.z();

  load_factor_ = 1. / (w*w - x*x - y*y + z*z);
  roll_  =  atan2(2.*y*z + 2.*w*x, z*z - y*y - x*x + w*w);
  pitch_ = -asin(2.*x*z - 2.*w*y);
  yaw_ =  atan2(2.*x*y + 2.*w*z, x*x + w*w - z*z - y*y);
}

void LeePositionControllerMod::SetTrajectoryPoint(
    const mav_msgs::EigenTrajectoryPoint& command_trajectory) {
  command_trajectory_ = command_trajectory;
  controller_active_ = true;
}

void LeePositionControllerMod::ComputeDesiredAcceleration(Eigen::Vector3d* acceleration) {
  assert(acceleration);

  test_position_msg_.data[0] = cmd_position_(0);
  test_position_msg_.data[1] = cmd_position_(1);
  test_position_msg_.data[2] = cmd_position_(2);
  test_position_pub_.publish(test_position_msg_);

  Eigen::Vector3d position_error,  velocity_error;
  Eigen::Vector3d velocity_W;
  if (external_linear_error_)
  {
    position_error = position_error_;
    velocity_error = velocity_error_;
  }
  else
  {
    //position_error = odometry_.position - command_trajectory_.position_W;
    position_error = odometry_.position - cmd_position_;
    // Transform velocity to world frame.
    velocity_W =  R_ * odometry_.velocity; // This odometry_.velocity is in {B}-frame
    // velocity_error = velocity_W - command_trajectory_.velocity_W;
    velocity_error = velocity_W - cmd_velocity_;

    Eigen::VectorXd cmd_velocity_f(3);
    cmd_velocity_tracker_.track(cmd_velocity_, cmd_velocity_f, cmd_accel_N_, dt_.toSec());
  }

  position_integral_error_ += position_error*dt_.toSec();

  Eigen::Vector3d e_3(Eigen::Vector3d::UnitZ());

  // *acceleration << 0.0,
  //                                  0.0,
  //                                 -pidZ_.update_f(cmd_velocity_(2), velocity_W(2), 0.0, dt_.toSec()) - vehicle_parameters_.gravity_*e_3(2);

  *acceleration = (position_error.cwiseProduct(controller_parameters_.position_gain_)
                               + position_integral_error_.cwiseProduct(controller_parameters_.position_integral_gain_)
                               + velocity_error.cwiseProduct(controller_parameters_.velocity_gain_)) / vehicle_parameters_.mass_
                                - vehicle_parameters_.gravity_ * e_3
                                - cmd_accel_N_;
  // */
}

// Implementation from the T. Lee et al. paper
// Control of complex maneuvers for a quadrotor UAV using geometric methods on SE(3)
void LeePositionControllerMod::ComputeDesiredAngularAcc(const Eigen::Vector3d& acceleration,
                                                     Eigen::Vector3d* angular_acceleration)
{
  assert(angular_acceleration);

  Eigen::Vector3d angle_error, angular_rate_error;

  double cmd_yaw = clampRotation(cmd_yaw_);

  // Calculate sin(roll)
  double sr = (-acceleration(0)*sin(cmd_yaw) + acceleration(1) *cos(cmd_yaw))
                           /sqrt( acceleration(0)*acceleration(0) + acceleration(1)*acceleration(1) + acceleration(2)*acceleration(2));

  /* std::cout<<"cmd_position_W z:"<<cmd_position_(2)
                   <<", real z:"<<odometry_.position(2)<<std::endl;
  std::cout<<"cmd_yaw:"<<cmd_yaw
                   <<", real yaw:"<<yaw_<<std::endl;
   //                <<", y:"<<-acceleration(1)
   //                <<", z:"<<-acceleration(2)<<std::endl; //*/

  if (sr > 1.0)
  {
    sr = 1.0;
  }
  else if (sr < -1.0)
  {
    sr = -1.0;
  }

  // */
  //cmd_roll_ = asin(sr);
  //cmd_pitch_ = atan2(-acceleration(0)*cos(cmd_yaw) - acceleration(1)*sin(cmd_yaw), -acceleration(2)); // In this one there is no Accel(2) part in the denominator

  // Get the desired rotation matrix.
  Eigen::Matrix3d R_des, dot_R_des_f;
  // Differential tracker R_des
  Eigen::VectorXd RPY_c(3), RPY_c_f(3), dot_RPY_c_f(3);

  if (test_active_)
  {
    cmd_roll_=0.0;
    cmd_pitch_=0.0;
    cmd_yaw = 0.6;
    ROS_INFO_ONCE("t_an attitude_ is: [%lf]", t_.toSec());

    RPY_c << cmd_roll_, cmd_pitch_, cmd_yaw; // it should be cmd_yaw_
    R_des_tracker_.track(RPY_c, RPY_c_f, dot_RPY_c_f, dt_.toSec());
  }
  else
  {
    cmd_roll_=0.0;
    cmd_pitch_=0.0;
    cmd_yaw = 0.0;

    dot_RPY_c_f.setZero();
  }


  R_des = Eigen::AngleAxisd(cmd_yaw, Eigen::Vector3d::UnitZ())                                  // yaw_des
               * Eigen::AngleAxisd(cmd_roll_, Eigen::Vector3d::UnitX())                                // roll_des
               * Eigen::AngleAxisd(cmd_pitch_, Eigen::Vector3d::UnitY());                            // pitch_des  */

  dot_R_des_f = Eigen::AngleAxisd(dot_RPY_c_f(2), Eigen::Vector3d::UnitZ())          // dot_yaw_des
                            * Eigen::AngleAxisd(dot_RPY_c_f(0), Eigen::Vector3d::UnitX())          // dot_roll_des
                            * Eigen::AngleAxisd(dot_RPY_c_f(1), Eigen::Vector3d::UnitY());         // dot_pitch_des  */

  // Get the desired rotation matrix. original
  /* Eigen::Vector3d b1_des;
  b1_des << cos(cmd_yaw), sin(cmd_yaw), 0;  // cmd_yaw_ should be defined globally

  Eigen::Vector3d b3_des;
  b3_des = -acceleration / acceleration.norm();

  Eigen::Vector3d b2_des;
  b2_des = b3_des.cross(b1_des);
  b2_des.normalize();

  R_des.col(0) = b2_des.cross(b3_des);
  R_des.col(1) = b2_des;
  R_des.col(2) = b3_des;  // */

  // NMPC attitude control

  // NMPC init
  X1_[0] = odometry_.orientation;
  X2_[0] = odometry_.angular_velocity;
  U_[0] = rotor_velocities_;


  // NMPC model

  // Solve dot_X_ = f(X_, U_)
  for (int i=1;i<5;i++)
  {
    // dot_R = R*angular_rate_skew_matrix
    double dq0 = 0.5*(-X1_[i].x() * X2_[i](0) - X1_[i].y() * X2_[i](1)  - X1_[i].z() * X2_[i](2));
    double dq1 = 0.5*(X1_[i].w() * X2_[i](0) + X1_[i].y() * X2_[i](2) - X1_[i].z() * X2_[i](1));
    double dq2 = 0.5*(X1_[i].w() * X2_[i](1) - X1_[i].x() * X2_[i](2) + X1_[i].z() * X2_[i](0));
    double dq3 = 0.5*(X1_[i].w() * X2_[i](2) + X1_[i].x() *X2_[i](1) - X1_[i].y() * X2_[i](0));
    X1_[i].w() = X1_[i-1].w() + dt_.toSec()*dq0;
    X1_[i].x() = X1_[i-1].x() + dt_.toSec()*dq1;
    X1_[i].y() = X1_[i-1].y() + dt_.toSec()*dq2;
    X1_[i].z() = X1_[i-1].z() + dt_.toSec()*dq3;
    // Normalise quaternion
    X1_[i].normalize();

    Eigen::Vector3d UAV_accel = rotor_velocities_to_angular_acc_*U_[i-1];

    X2_[i](0) = X2_[i-1](0) + X2_[i-1](1)*X2_[i-1](2)*(J_(1) - J_(2))/J_(0) + UAV_accel(0);
    X2_[i](1) = X2_[i-1](1) + X2_[i-1](2)*X2_[i-1](0)*(J_(2) - J_(0))/J_(0) + UAV_accel(1);
    X2_[i](2) = X2_[i-1](2) + X2_[i-1](0)*X2_[i-1](1)*(J_(0) - J_(1))/J_(0) + UAV_accel(2);
  }

  // Angle error according to lee et al.
  Eigen::Matrix3d angle_error_matrix = 0.5 * (R_des.transpose() * R_ - R_.transpose() * R_des);
  vectorFromSkewMatrix(angle_error_matrix, &angle_error);

  // TODO(burrimi) include angular rate references at some point.
  Eigen::Vector3d angular_rate_des(Eigen::Vector3d::Zero());
  Eigen::Matrix3d angular_rate_des_matrix;
  // angular_rate_des[2] = cmd_yaw_rate_;  // yaw_rate should be defined globally

  angular_rate_des_matrix = R_.transpose()*dot_R_des_f;
  vectorFromSkewMatrix(angular_rate_des_matrix, &angular_rate_des);

  Eigen::VectorXd angular_rate_des_f(3), dot_angular_rate_des_f(3);
  angular_rate_des_tracker_.track(angular_rate_des, angular_rate_des_f, dot_angular_rate_des_f, dt_.toSec());

  Eigen::Matrix3d angular_rate_matrix;
  skewMatrixFromVector(odometry_.angular_velocity, &angular_rate_matrix);

  angular_rate_error = odometry_.angular_velocity - R_.transpose() * R_des * angular_rate_des;
  //std::cout<<"angular error roll:"<<angle_error(0)<<", pitch:"<<angle_error(1)<<", yaw:"<<angle_error(2)<<std::endl;

  angle_error_integral_ += angle_error*dt_.toSec();

  *angular_acceleration = -1 * angle_error.cwiseProduct(normalized_attitude_gain_)
                                                  - angle_error_integral_.cwiseProduct(normalized_attitude_integral_gain_)
                                                 - angular_rate_error.cwiseProduct(normalized_angular_rate_gain_)
                                                + odometry_.angular_velocity.cross(odometry_.angular_velocity)
                                                 - (angular_rate_matrix*R_.transpose()*R_des*angular_rate_des - R_.transpose()*R_des*dot_angular_rate_des_f); // we don't need the inertia matrix here
}

}
