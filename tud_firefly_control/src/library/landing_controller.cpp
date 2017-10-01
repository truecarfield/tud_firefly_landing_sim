#include "tud_firefly_control/landing_controller.h"

namespace rotors_control {

static constexpr double kDefaulMaxRotVelocity = 838.0;

LandingController::LandingController()
    : initialized_params_(false),
      controller_active_(false),
      test_active_(false),
      external_error_(false),
      use_R_des_tracker_(false),
      use_rate_des_tracker_(false),
      use_cmd_velocity_tracker_(false),
      use_position_error_tracker_(false),
      cmd_yaw_(0.0),
      gravity_N_(-9.80665),
      gg_N_(gravity_N_*gravity_N_),
      max_tilt_angle_(0.5),
      PI_(3.141592653589793)
{
  cmd_yaw_ = 0.0; cmd_yaw_rate_ = 0.0;
  roll_ = 0.0; pitch_ = 0.0; yaw_ = 0.0;
  W_ <<1,      sin(roll_)*tan(pitch_),  cos(roll_)*tan(pitch_),
              0.0,  cos(roll_),                         -sin(roll_),
              0.0,  sin(roll_)/cos(pitch_),  cos(roll_)/cos(pitch_);

  cmd_position_.resize(3);
  cmd_velocity_.resize(3);
  // cmd_accel_N_.resize(3);

  cmd_position_.setZero();
  cmd_velocity_.setZero();
  // cmd_accel_N_.setZero();

  rotor_velocities_sqr_.resize(6);
  rotor_velocities_sqr_.setZero();
  rotor_velocities_max_.resize(6);
  for (int i=0;i<6;i++)
  {
    rotor_velocities_max_(i) = kDefaulMaxRotVelocity;
  }
}

LandingController::~LandingController() {}

void LandingController::InitializeParameters(const ros::NodeHandle& pnh) {
  // Read parameters from rosparam.
  GetRosParameter(pnh, "position_gain/x",
                  controller_parameters_.position_gain_.x(),
                  &controller_parameters_.position_gain_.x());
  GetRosParameter(pnh, "position_gain/y",
                  controller_parameters_.position_gain_.y(),
                  &controller_parameters_.position_gain_.y());
  GetRosParameter(pnh, "position_gain/z",
                  controller_parameters_.position_gain_.z(),
                  &controller_parameters_.position_gain_.z());
  GetRosParameter(pnh, "position_integral_gain/x",
                  controller_parameters_.position_integral_gain_.x(),
                  &controller_parameters_.position_integral_gain_.x());
  GetRosParameter(pnh, "position_integral_gain/y",
                  controller_parameters_.position_integral_gain_.y(),
                  &controller_parameters_.position_integral_gain_.y());
  GetRosParameter(pnh, "position_integral_gain/z",
                  controller_parameters_.position_integral_gain_.z(),
                  &controller_parameters_.position_integral_gain_.z());
  GetRosParameter(pnh, "velocity_gain/x",
                  controller_parameters_.velocity_gain_.x(),
                  &controller_parameters_.velocity_gain_.x());
  GetRosParameter(pnh, "velocity_gain/y",
                  controller_parameters_.velocity_gain_.y(),
                  &controller_parameters_.velocity_gain_.y());
  GetRosParameter(pnh, "velocity_gain/z",
                  controller_parameters_.velocity_gain_.z(),
                  &controller_parameters_.velocity_gain_.z());
  GetRosParameter(pnh, "acceleration_gain/x",
                  controller_parameters_.acceleration_gain_.x(),
                  &controller_parameters_.acceleration_gain_.x());
  GetRosParameter(pnh, "acceleration_gain/y",
                  controller_parameters_.acceleration_gain_.y(),
                  &controller_parameters_.acceleration_gain_.y());
  GetRosParameter(pnh, "acceleration_gain/z",
                  controller_parameters_.acceleration_gain_.z(),
                  &controller_parameters_.acceleration_gain_.z());
  GetRosParameter(pnh, "attitude_gain/x",
                  controller_parameters_.attitude_gain_.x(),
                  &controller_parameters_.attitude_gain_.x());
  GetRosParameter(pnh, "attitude_gain/y",
                  controller_parameters_.attitude_gain_.y(),
                  &controller_parameters_.attitude_gain_.y());
  GetRosParameter(pnh, "attitude_gain/z",
                  controller_parameters_.attitude_gain_.z(),
                  &controller_parameters_.attitude_gain_.z());
  GetRosParameter(pnh, "attitude_integral_gain/x",
                  controller_parameters_.attitude_integral_gain_.x(),
                  &controller_parameters_.attitude_integral_gain_.x());
  GetRosParameter(pnh, "attitude_integral_gain/y",
                  controller_parameters_.attitude_integral_gain_.y(),
                  &controller_parameters_.attitude_integral_gain_.y());
  GetRosParameter(pnh, "attitude_integral_gain/z",
                  controller_parameters_.attitude_integral_gain_.z(),
                  &controller_parameters_.attitude_integral_gain_.z());
  GetRosParameter(pnh, "angular_rate_gain/x",
                  controller_parameters_.angular_rate_gain_.x(),
                  &controller_parameters_.angular_rate_gain_.x());
  GetRosParameter(pnh, "angular_rate_gain/y",
                  controller_parameters_.angular_rate_gain_.y(),
                  &controller_parameters_.angular_rate_gain_.y());
  GetRosParameter(pnh, "angular_rate_gain/z",
                  controller_parameters_.angular_rate_gain_.z(),
                  &controller_parameters_.angular_rate_gain_.z());
  GetVehicleParameters(pnh, &vehicle_parameters_);

  calculateAllocationMatrix(vehicle_parameters_.rotor_configuration_, &(controller_parameters_.allocation_matrix_));
  // To make the tuning independent of the inertia matrix we divide here.
  normalized_attitude_gain_ = controller_parameters_.attitude_gain_.transpose()
      * vehicle_parameters_.inertia_.inverse();
  normalized_attitude_integral_gain_ =  controller_parameters_.attitude_integral_gain_.transpose()
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

  rotor_velocities_to_angular_acc_.resize(4, 6);
  rotor_velocities_to_angular_acc_.block<3, 6>(0, 0) = vehicle_parameters_.inertia_.inverse() * controller_parameters_.allocation_matrix_.block<3, 6>(0, 0);
  rotor_velocities_to_angular_acc_.block<1, 6>(3, 0) = I.block<1, 6>(3, 0)/vehicle_parameters_.mass_;

  // Whether use external control error source
  pnh.param("external_error", external_error_, external_error_);
  pnh.param("pid_mode", pid_mode_, pid_mode_);

  // Whether use differential trackers
  pnh.param("use_R_des_tracker", use_R_des_tracker_, false);
  pnh.param("use_rate_des_tracker", use_rate_des_tracker_, false);
  pnh.param("use_cmd_vel_tracker", use_cmd_velocity_tracker_, false);
  pnh.param("use_position_error_tracker", use_position_error_tracker_, false);
  R_des_tracker_.init(pnh, "R_des");
  angular_rate_des_tracker_.init(pnh, "angular_rate_des");
  RPY_tracker_.init(pnh, "RPY");
  //cmd_velocity_tracker_.init(pnh, "cmd_velocity");
  //position_error_tracker_.init(pnh, "position_error");

  //BK params;
  std::vector<double> cAngle(3,0), cRate(3,0), cIntAngle;
  pnh.getParam("c_angle", cAngle);
  pnh.getParam("c_int_angle", cIntAngle);
  pnh.getParam("c_rate", cRate);
  for (int i=0;i<3;i++)
  {
    cAngle_(i,i) = cAngle[i];
  }
  for (int i=0;i<3;i++)
  {
    cIntAngle_(i,i) = cIntAngle[i];
  }
  for (int i=0;i<3;i++)
  {
    cRate_(i) = cRate[i];
  }
  ypsilon_.setZero();

  // Initialize vector form of inertia
  J_(0) = vehicle_parameters_.inertia_(0,0);
  J_(1) = vehicle_parameters_.inertia_(1,1);
  J_(2) = vehicle_parameters_.inertia_(2,2);

  // Initialize the PID controllers
  pidX_.initialize(ros::NodeHandle(pnh, "linear/xy"));
  pidY_.initialize(ros::NodeHandle(pnh, "linear/xy"));
  pidZ_.initialize(ros::NodeHandle(pnh, "linear/z"));
  pidRoll_.initialize(ros::NodeHandle(pnh, "angular/roll"));
  pidPitch_.initialize(ros::NodeHandle(pnh, "angular/pitch"));
  pidYaw_.initialize(ros::NodeHandle(pnh, "angular/yaw"));
  double c = 10.0;
  pnh.param("pid_decouple_factor", c, 10.0);
  c_ << c, 0.0, 0.0,
             0.0, c, 0.0,
             0.0, 0.0, c;

  pidVx_.initialize(ros::NodeHandle(pnh, "linear/Vxy"));
  pidVy_.initialize(ros::NodeHandle(pnh, "linear/Vxy"));
  pidVz_.initialize(ros::NodeHandle(pnh, "linear/Vz"));
  pidYawRate_.initialize(ros::NodeHandle(pnh, "angular/r"));
  //nmpcInit();

  initialized_params_ = true;
}

void LandingController::SetExternalError(const bool& external_error)
{
  external_error_ = external_error;
}

void LandingController::SetTime(const ros::Time& t) {
 t_ = t;
}

void LandingController::SetPeriod(const ros::Duration &dt)
{
  dt_ = dt;
}

void LandingController::SetMaxTiltAngle(const double& max_tilt_angle)
{
  max_tilt_angle_ = max_tilt_angle;
}

void LandingController::SetUseRdesTracker(const bool& use_R_des_tracker)
{
  use_R_des_tracker_ = use_R_des_tracker;
}

void LandingController::SetUseRateTracker(const bool& use_rate_des_tracker)
{
  use_rate_des_tracker_ = use_rate_des_tracker;
}

void LandingController::SetUseCmdVelTracker(const bool& use_cmd_velocity_tracker)
{
  use_cmd_velocity_tracker_ = use_cmd_velocity_tracker;
}

void LandingController::SetCmdFromTwist(const geometry_msgs::Twist& msg)
{
                                        cmd_velocity_ << cos(yaw_)*msg.linear.x - sin(yaw_)*msg.linear.y,
                                                                            sin(yaw_)*msg.linear.x + cos(yaw_)*msg.linear.y,
                                                                            msg.linear.z; // */
  command_trajectory_.position_W += cmd_velocity_*dt_.toSec();
  if (use_cmd_velocity_)
  {
    command_trajectory_.velocity_W = cmd_velocity_;
  }
  command_trajectory_.setFromYawRate(msg.angular.z);
  double cmd_yaw = command_trajectory_.getYaw();
  cmd_yaw += msg.angular.z*dt_.toSec();
  command_trajectory_.setFromYaw(cmd_yaw); // */
}

void LandingController::SetCmdYaw(const double &cmd_yaw)
{
  cmd_yaw_ = cmd_yaw;
}

void LandingController::SetTrajErrorCmdAccelYawPoint(const mav_msgs::EigenTrajectoryPoint& traj_error_cmd_yaw)
{
  traj_error_cmd_accel_yaw_ = traj_error_cmd_yaw;
  // std::cout<<"traj_error_point z:"<<traj_error_cmd_yaw.position_W(2)<<std::endl;
}

void LandingController::reset() {}

void LandingController::nmpcInit() {
   f_ << dot(r11_) == r12_*r_ - r13_*q_;
   f_ << dot(r12_) == r13_*p_ - r11_*r_;
   f_ << dot(r13_) == r11_*q_ - r12_*p_;
   f_ << dot(r21_) == r22_*r_ - r23_*q_;
   f_ << dot(r22_) == r23_*p_ - r21_*r_;
   f_ << dot(r23_) == r21_*q_ - r22_*p_;
   f_ << dot(r31_) == r32_*r_ - r33_*q_;
   f_ << dot(r32_) == r33_*p_ - r31_*r_;
   f_ << dot(r33_) == r31_*q_ - r32_*p_;
   f_ << dot(p_) == (J_(1,1) - J_(2,2))*q_*r_/J_(0,0)+2.64405e-05*w0w0_+5.28809e-05*w1w1_+2.64405e-05*w2w2_-2.64405e-05*w3w3_-5.28809e-05*w4w4_-2.64405e-05*w5w5_;
   f_ << dot(q_) == (J_(2,2) - J_(0,0))*r_*p_/J_(1,1)-3.46831e-05*w0w0_-1.96101e-16*w1w1_+3.46831e-05*w2w2_+3.46831e-05*w3w3_-1.96101e-16*w4w4_-3.46831e-05*w5w5_;
   f_ << dot(r_) == (J_(0,0) - J_(1,1))*p_*q_/J_(2,2)-1.39997e-06*w0w0_+1.39997e-06*w1w1_-1.39997e-06*w2w2_+1.39997e-06*w3w3_-1.39997e-06*w4w4_-1.39997e-06*w5w5_;

   ocp_ = new ACADO::OCP(0.0, 0.05, 5);
   ocp_->subjectTo(f_);
   ocp_->subjectTo(0.0 <= w0w0_ <= 702244.0);
   ocp_->subjectTo(0.0 <= w1w1_ <= 702244.0);
   ocp_->subjectTo(0.0 <= w2w2_ <= 702244.0);
   ocp_->subjectTo(0.0 <= w3w3_ <= 702244.0);
   ocp_->subjectTo(0.0 <= w4w4_ <= 702244.0);
   ocp_->subjectTo(0.0 <= w5w5_ <= 702244.0);
   ocp_->subjectTo(-5.0 <= p_ <= 5.0);
   ocp_->subjectTo(-5.0 <= q_ <= 5.0);
   ocp_->subjectTo(-5.0 <= r_ <= 5.0);
 }

void LandingController::NMPC(Eigen::VectorXd* rotor_velocities)
{
 /* assert(rotor_velocities);
  assert(initialized_params_);

  rotor_velocities->resize(vehicle_parameters_.rotor_configuration_.rotors.size());
  // Return 0 velocities on all rotors, until the first command is received.
  if (!controller_active_) {
    *rotor_velocities = Eigen::VectorXd::Zero(rotor_velocities->rows());
    return;
  }

  Eigen::Vector3d acceleration;
  ComputeDesiredAcceleration(&acceleration);

  // Project thrust onto body z axis.
  double thrust = -vehicle_parameters_.mass_ * acceleration.dot(odometry_.orientation.toRotationMatrix().col(2));

  // Define angle error and its matrix form and angular rate error for this iteration.
  Eigen::Vector3d angle_error(Eigen::Vector3d::Zero()), angular_rate_error(Eigen::Vector3d::Zero());
  Eigen::Matrix3d angle_error_matrix;

  // Define desired angular rate for this iteration.
  Eigen::Matrix3d R_des;
  Eigen::Vector3d angular_rate_des(Eigen::Vector3d::Zero());
  Eigen::Matrix3d angular_rate_des_matrix;

  double cmd_yaw;
  if (external_error_)
  {
    cmd_yaw = clampRotation(cmd_yaw_);
  }
  else
  {
    cmd_yaw = clampRotation(command_trajectory_.getYaw());
  }

  //  std::cout<<"cmd_yaw"<<cmd_yaw<<std::endl;

  double cmd_yaw_fake = yaw_;

  // Get the desired rotation matrix. original
  Eigen::Vector3d b1_des;
  b1_des << cos(cmd_yaw), sin(cmd_yaw), 0;  // cmd_yaw_ should be defined globally

  Eigen::Vector3d b3_des;
  b3_des = -acceleration / acceleration.norm();

  Eigen::Vector3d b2_des;
  b2_des = b3_des.cross(b1_des);
  b2_des.normalize();

  R_des.col(0) = b2_des.cross(b3_des);
  R_des.col(1) = b2_des;
  R_des.col(2) = b3_des;

  // Define R_des differential tracker variables for this iteration.
  Eigen::VectorXd R_des_vec(9), R_des_vec_f(9), dot_R_des_vec_f(9);
  for (int i=0; i<3;i++)
  {
    for (int j=0; j<3;j++)
    {
      R_des_vec(3*i+j) = R_des(i, j);
    }
  }
  R_des_vec_f = R_des_vec;
  R_des_tracker_.track(R_des_vec, R_des_vec_f, dot_R_des_vec_f, dt_.toSec());

  Eigen::Matrix3d dot_R_des_f;
  for (int i=0; i<3;i++)
  {
    for (int j=0; j<3;j++)
    {
      dot_R_des_f(i, j) = dot_R_des_vec_f(3*i+j);
    }
  }
  angular_rate_des_matrix = R_des.transpose()*dot_R_des_f;
  vectorFromSkewMatrix(angular_rate_des_matrix, &angular_rate_des);

  Eigen::Matrix3d angular_rate_matrix;
  skewMatrixFromVector(odometry_.angular_velocity, &angular_rate_matrix);
  angular_rate_error = odometry_.angular_velocity - R_.transpose() * R_des * angular_rate_des;
  angle_error_integral_ += angle_error*dt_.toSec();

  *rotor_velocities = angular_acc_to_rotor_velocities_ * angular_acceleration_thrust;
  rotor_velocities_sqr_ = *rotor_velocities;
  *rotor_velocities = rotor_velocities->cwiseMax(Eigen::VectorXd::Zero(rotor_velocities->rows()));
  *rotor_velocities = rotor_velocities->cwiseSqrt();
  *rotor_velocities = rotor_velocities->cwiseMin(rotor_velocities_max_); //*/
}

void LandingController::HYB(Eigen::VectorXd *rotor_velocities)
{
  assert(rotor_velocities);
  assert(initialized_params_);

  rotor_velocities->resize(vehicle_parameters_.rotor_configuration_.rotors.size());
  // Return 0 velocities on all rotors, until the first command is received.
  if (!controller_active_) {
    *rotor_velocities = Eigen::VectorXd::Zero(rotor_velocities->rows());
    return;
  }

  Eigen::Vector3d acceleration;
  PIDAcceleration(&acceleration);

  Eigen::Vector3d angular_acceleration;
  ComputeDesiredAngularAcc(-acceleration, &angular_acceleration);

  // Project thrust onto body z axis.
  // double thrust = vehicle_parameters_.mass_ *((gravity_N_ - acceleration(2))*load_factor_ - gravity_N_);
  double thrust = vehicle_parameters_.mass_*acceleration.dot(odometry_.orientation.toRotationMatrix().col(2));

  Eigen::Vector4d angular_acceleration_thrust;
  angular_acceleration_thrust.block<3, 1>(0, 0) = angular_acceleration;
  angular_acceleration_thrust(3) = thrust;

  *rotor_velocities = angular_acc_to_rotor_velocities_ * angular_acceleration_thrust;
  rotor_velocities_sqr_ = *rotor_velocities;
  *rotor_velocities = rotor_velocities->cwiseMax(Eigen::VectorXd::Zero(rotor_velocities->rows()));
  *rotor_velocities = rotor_velocities->cwiseSqrt();
  *rotor_velocities = rotor_velocities->cwiseMin(rotor_velocities_max_);
}

void LandingController::PID(Eigen::VectorXd *rotor_velocities)
{
  assert(rotor_velocities);
  assert(initialized_params_);

  rotor_velocities->resize(vehicle_parameters_.rotor_configuration_.rotors.size());
  // Return 0 velocities on all rotors, until the first command is received.
  if (!controller_active_) {
    *rotor_velocities = Eigen::VectorXd::Zero(rotor_velocities->rows());
    return;
  }

  Eigen::Vector3d acceleration;
  PIDAcceleration(&acceleration);

  Eigen::Vector3d angular_acceleration;
  PIDAngularAcc(acceleration, &angular_acceleration);

  // Project thrust onto body z axis.
  // double thrust = vehicle_parameters_.mass_ *((gravity_N_ - acceleration(2))*load_factor_ - gravity_N_);
  double thrust = vehicle_parameters_.mass_*acceleration.dot(odometry_.orientation.toRotationMatrix().col(2));

  Eigen::Vector4d angular_acceleration_thrust;
  angular_acceleration_thrust.block<3, 1>(0, 0) = angular_acceleration;
  angular_acceleration_thrust(3) = thrust;

  *rotor_velocities = angular_acc_to_rotor_velocities_ * angular_acceleration_thrust;
  rotor_velocities_sqr_ = *rotor_velocities;
  *rotor_velocities = rotor_velocities->cwiseMax(Eigen::VectorXd::Zero(rotor_velocities->rows()));
  *rotor_velocities = rotor_velocities->cwiseSqrt();
  *rotor_velocities = rotor_velocities->cwiseMin(rotor_velocities_max_);
}

void LandingController::CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) {
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
  //double thrust = vehicle_parameters_.mass_ *((gravity_N_ - acceleration(2))*load_factor_ - gravity_N_);
  double thrust = -vehicle_parameters_.mass_ * acceleration.dot(odometry_.orientation.toRotationMatrix().col(2));

  Eigen::Vector4d angular_acceleration_thrust;
  angular_acceleration_thrust.block<3, 1>(0, 0) = angular_acceleration;
  angular_acceleration_thrust(3) = thrust;

  *rotor_velocities = angular_acc_to_rotor_velocities_ * angular_acceleration_thrust;
  rotor_velocities_sqr_ = *rotor_velocities;
  *rotor_velocities = rotor_velocities->cwiseMax(Eigen::VectorXd::Zero(rotor_velocities->rows()));
  *rotor_velocities = rotor_velocities->cwiseSqrt();
  *rotor_velocities = rotor_velocities->cwiseMin(rotor_velocities_max_);
}

void LandingController::GetRotorVelSqr(Eigen::VectorXd &rotor_velocities_sqr)
{
  rotor_velocities_sqr = rotor_velocities_sqr_;
}

void  LandingController::SetActive(const bool& active) {
  controller_active_ = active;
}

void LandingController::SetTest(const bool &test_active)
{
  test_active_ = test_active;
}

void LandingController::SetOdometry(const EigenOdometry& odometry) {
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

  W_ <<1,      sin(roll_)*tan(pitch_),  cos(roll_)*tan(pitch_),
              0.0,  cos(roll_),                         -sin(roll_),
              0.0,  sin(roll_)/cos(pitch_),  cos(roll_)/cos(pitch_);
  // */
}

void LandingController::SetTrajectoryPoint(
    const mav_msgs::EigenTrajectoryPoint& command_trajectory) {
  command_trajectory_ = command_trajectory;
  // std::cout<<"cmd yaw is:"<<command_trajectory.getYaw()<<std::endl;
  controller_active_ = true;
}

void LandingController::ComputeDesiredAcceleration(Eigen::Vector3d* acceleration) {
  assert(acceleration);
  Eigen::VectorXd position_error(3), position_error_f(3),  velocity_error(3);
  Eigen::VectorXd cmd_accel_N_(3);
  Eigen::Vector3d accel_unlimited;

  if (external_error_)
  {
    position_error = traj_error_cmd_accel_yaw_.position_W;
    // std::cout<<"poserror:"<<position_error<<std::endl;

    if (use_position_error_tracker_)
    {
      position_error_tracker_.track(position_error, position_error_f, velocity_error, dt_.toSec());
      position_error = position_error_f;
    }
    else
    {
      velocity_error = traj_error_cmd_accel_yaw_.velocity_W;
    }

   cmd_accel_N_ = traj_error_cmd_accel_yaw_.acceleration_W;
  }
  else
  {
    position_error = odometry_.position - command_trajectory_.position_W;
    position_error_tracker_.reportX(command_trajectory_.position_W); // test
    // Transform velocity to world frame.
    Eigen::Vector3d velocity_W =  R_ * odometry_.velocity; // This odometry_.velocity is in {B}-frame
    velocity_error = velocity_W - command_trajectory_.velocity_W;

    cmd_accel_N_ = command_trajectory_.acceleration_W;
  }

  Eigen::Vector3d e_3(Eigen::Vector3d::UnitZ());
  if (!pid_mode_)
  {
    // Lee position controller
    position_integral_error_ += position_error*dt_.toSec();
    accel_unlimited = (position_error.cwiseProduct(controller_parameters_.position_gain_)
                                 // + position_integral_error_.cwiseProduct(controller_parameters_.position_integral_gain_)
                                 + velocity_error.cwiseProduct(controller_parameters_.velocity_gain_)) / vehicle_parameters_.mass_
                                 - cmd_accel_N_.cwiseProduct(controller_parameters_.acceleration_gain_)
                                 - vehicle_parameters_.gravity_ * e_3; // */
  }
  else
  {
    // PID twist controller
   accel_unlimited << pidVx_.update_f(velocity_error(0), std::numeric_limits<double>::quiet_NaN(), dt_.toSec()); // - cmd_accel_N_(0),
                                         pidVy_.update_f(velocity_error(1), std::numeric_limits<double>::quiet_NaN(), dt_.toSec()); // - cmd_accel_N_(1),
                                         pidVz_.update_f(velocity_error(2), std::numeric_limits<double>::quiet_NaN(), dt_.toSec()) - vehicle_parameters_.gravity_*e_3(2); // - cmd_accel_N_(2);
  }
  double accel_max = 4.0;
  double accel_factor = fabs(accel_unlimited(0))/sqrt(accel_unlimited(0)*accel_unlimited(0)+accel_unlimited(1)*accel_unlimited(1));
  accel_factor = isnan(accel_factor) ? 1.0 : accel_factor;
  accel_unlimited(0) = boost::math::sign(accel_unlimited(0))*std::min(accel_max*accel_factor, fabs(accel_unlimited(0)));
  //accel_unlimited(0) = boost::math::sign(accel_unlimited(0))*std::min(4.0, fabs(accel_unlimited(0)));
  accel_unlimited(1) = boost::math::sign(accel_unlimited(1))*std::min(sqrt(accel_max*accel_max-accel_unlimited(0)*accel_unlimited(0)), fabs(accel_unlimited(1)));
  //std::cout<<"ax:"<<accel_unlimited(0)<<", ay:"<<accel_unlimited(1)<<", az:"<<accel_unlimited(2)<<std::endl;
  *acceleration = accel_unlimited;
  //Eigen::Vector3d temp(*acceleration);
  //std::cout<<"dt:"<<dt_.toSec()<<", error x:"<<velocity_error(0)<<", y:"<<velocity_error(1)<<", z:"<<velocity_error(2)<<", cmd Ax:"<<temp(0)<<", Ay:"<<temp(1)<<", Az:"<<temp(2)<<std::endl;
}

void LandingController::bkAccel(Eigen::Vector3d *acceleration)
{
  assert(acceleration);
  Eigen::VectorXd position_error(3), position_error_f(3),  velocity_error(3);
  Eigen::VectorXd cmd_accel_N_(3);

  if (external_error_)
  {
    position_error = traj_error_cmd_accel_yaw_.position_W;

    if (use_position_error_tracker_)
    {
      position_error_tracker_.track(position_error, position_error_f, velocity_error, dt_.toSec());
      position_error = position_error_f;
    }
    else
    {
      velocity_error = traj_error_cmd_accel_yaw_.velocity_W;
    }

    if (use_cmd_velocity_tracker_)
    {
      cmd_velocity_tracker_.diff(R_*odometry_.velocity - traj_error_cmd_accel_yaw_.velocity_W, cmd_accel_N_, dt_.toSec());
    }
    else
    {
      cmd_velocity_tracker_.reportX(R_*odometry_.velocity - traj_error_cmd_accel_yaw_.velocity_W);
      cmd_accel_N_ = traj_error_cmd_accel_yaw_.acceleration_W;
    }
  }
  else
  {
    position_error = odometry_.position - command_trajectory_.position_W;
    position_error_tracker_.reportX(command_trajectory_.position_W); // test
    // Transform velocity to world frame.
    Eigen::Vector3d velocity_W =  R_ * odometry_.velocity; // This odometry_.velocity is in {B}-frame
    velocity_error = velocity_W - command_trajectory_.velocity_W;

    if (use_cmd_velocity_tracker_)
    {
      cmd_velocity_tracker_.diff(command_trajectory_.velocity_W, cmd_accel_N_, dt_.toSec());
    }
    else
    {
      cmd_accel_N_ = command_trajectory_.acceleration_W;
    }
  }

  Eigen::Vector3d e_3(Eigen::Vector3d::UnitZ());
  // Lee position controller
  position_integral_error_ += position_error*dt_.toSec();
  *acceleration = (position_error.cwiseProduct(controller_parameters_.position_gain_)
                                 // + position_integral_error_.cwiseProduct(controller_parameters_.position_integral_gain_)
                                 + velocity_error.cwiseProduct(controller_parameters_.velocity_gain_)) / vehicle_parameters_.mass_
                                 - cmd_accel_N_.cwiseProduct(controller_parameters_.acceleration_gain_)
                                 - vehicle_parameters_.gravity_ * e_3; // */
}

void LandingController::BK(Eigen::VectorXd *rotor_velocities)
{
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
  //std::cout<<"ax:"<<acceleration(0)<<", ay:"<<acceleration(1)<<", az:"<<acceleration(2)<<std::endl;

  // Define angle error and its matrix form and angular rate error for this iteration.
  Eigen::Vector3d angle_error(Eigen::Vector3d::Zero()), angular_rate_error(Eigen::Vector3d::Zero());
  Eigen::Matrix3d angle_error_matrix;

  // Define desired angular rate for this iteration.
  Eigen::Matrix3d R_des;
  Eigen::Vector3d angular_rate_des(Eigen::Vector3d::Zero());
  Eigen::Matrix3d angular_rate_des_matrix;

  double cmd_yaw;
  if (external_error_)
  {
    cmd_yaw = clampRotation(cmd_yaw_);
  }
  else
  {
    cmd_yaw = clampRotation(command_trajectory_.getYaw());
  }

  //  std::cout<<"cmd_yaw"<<cmd_yaw<<std::endl;

  double cmd_yaw_fake = yaw_;
  // Calculate sin(roll)
  double sr = (-acceleration(0)*sin(cmd_yaw_fake) + acceleration(1) *cos(cmd_yaw_fake))
                           /sqrt(acceleration(0)*acceleration(0) + acceleration(1)*acceleration(1) + acceleration(2)*acceleration(2));

  if (sr > 1.0)
  {
    sr = 1.0;
  }
  else if (sr < -1.0)
  {
    sr = -1.0;
  }

  double cmd_roll_temp = asin(sr);
  double cmd_pitch_temp = atan2(-acceleration(0)*cos(cmd_yaw_fake) - acceleration(1)*sin(cmd_yaw_fake), -acceleration(2)); // In this one there is no Accel(2) part in the denominator

  //  /*
  double max_tilt_angle = 0.36 ;// max_tilt_angle_ ;
  double cmd_roll_limit = boost::math::sign(cmd_roll_temp)*std::min(fabs(cmd_roll_temp), max_tilt_angle);
  double temp_term = cos(max_tilt_angle)/cos(fabs(cmd_roll_limit));
  if (temp_term > 1.0)
  {temp_term = 1.0;}
  double max_pitch_temp = acos(temp_term);
  double cmd_pitch_limit = boost::math::sign(cmd_pitch_temp)*std::min(fabs(cmd_pitch_temp), max_pitch_temp);
  //cmd_roll_ = cmd_roll_limit;
  //cmd_pitch_ = cmd_pitch_limit;  // */

  cmd_roll_ = cmd_roll_temp;
  cmd_pitch_ = cmd_pitch_temp;
  // std::cout<<"cmd_roll:"<<cmd_roll_<<", pitch:"<<cmd_pitch_<<std::endl;

  // Get the desired rotation matrix. original
  /*
  Eigen::Vector3d b1_des;
  b1_des << cos(cmd_yaw), sin(cmd_yaw), 0;  // cmd_yaw_ should be defined globally

  Eigen::Vector3d b3_des;
  b3_des = -acceleration / acceleration.norm();

  Eigen::Vector3d b2_des;
  b2_des = b3_des.cross(b1_des);
  b2_des.normalize();

  R_des.col(0) = b2_des.cross(b3_des);
  R_des.col(1) = b2_des;
  R_des.col(2) = b3_des;  // */

  // Get the desired rotation matrix.
  R_des = Eigen::AngleAxisd(cmd_yaw, Eigen::Vector3d::UnitZ())                  // yaw_des
               * Eigen::AngleAxisd(cmd_roll_, Eigen::Vector3d::UnitX())                             // roll_des
               * Eigen::AngleAxisd(cmd_pitch_, Eigen::Vector3d::UnitY());                            // pitch_des  */

  // Define R_des differential tracker variables for this iteration.
  Eigen::VectorXd R_des_vec(9), R_des_vec_f(9), dot_R_des_vec_f(9);
  for (int i=0; i<3;i++)
  {
    for (int j=0; j<3;j++)
    {
      R_des_vec(3*i+j) = R_des(i, j);
    }
  }
  R_des_vec_f = R_des_vec;
  R_des_tracker_.track(R_des_vec, R_des_vec_f, dot_R_des_vec_f, dt_.toSec());

  Eigen::Matrix3d dot_R_des_f;
  for (int i=0; i<3;i++)
  {
    for (int j=0; j<3;j++)
    {
      dot_R_des_f(i, j) = dot_R_des_vec_f(3*i+j);
    }
  }
  //angular_rate_des_matrix = R_des.transpose()*dot_R_des_f;
  //vectorFromSkewMatrix(angular_rate_des_matrix, &angular_rate_des);

  // Angle error according to lee et al.
  angle_error_matrix = 0.5 * (R_des.transpose() * R_ - R_.transpose() * R_des);
  vectorFromSkewMatrix(angle_error_matrix, &angle_error);
  //std::cout<<"angle error x:"<<angle_error(0)<<", y:"<<angle_error(1)<<", z:"<<angle_error(2)<<std::endl;

  Eigen::VectorXd RPY(3), RPY_f(3), dot_RPY_f(3);
  RPY << roll_, pitch_, yaw_;
  RPY_tracker_.track(RPY, RPY_f, dot_RPY_f, dt_.toSec());
  dot_RPY_f << 0.0, 0.0, 0.0; // it seems that dot_RPY_f is zero then result is good

  angular_rate_des = W_.inverse()*(dot_RPY_f - cAngle_*angle_error);
  Eigen::VectorXd angular_rate_des_f(3), dot_angular_rate_des_f(3);

  //std::cout<<"rated x:"<<angular_rate_des(0)<<", y:"<<angular_rate_des(1)<<", z:"<<angular_rate_des(2)<<std::endl;
  angular_rate_des_tracker_.track(angular_rate_des, angular_rate_des_f, dot_angular_rate_des_f, dt_.toSec());
  //std::cout<<"ratedf x:"<<angular_rate_des_f(0)<<", y:"<<angular_rate_des_f(1)<<", z:"<<angular_rate_des_f(2)<<std::endl;

  angular_rate_error = odometry_.angular_velocity - angular_rate_des_f;
  ypsilon_ += (-cAngle_*ypsilon_ + W_*(angular_rate_des_f - angular_rate_des))*dt_.toSec();
  //std::cout<<"yp1:"<<ypsilon_(0)<<", yp2:"<<ypsilon_(1)<<", yp3:"<<ypsilon_(2)<<std::endl;

  Eigen::Vector3d angle_error_mod;
  angle_error_mod = angle_error - ypsilon_;
  angle_error_integral_ += angle_error*dt_.toSec();

  Eigen::Vector3d angular_accel_vec;
  angular_accel_vec = odometry_.angular_velocity.cross(odometry_.angular_velocity)
                                          + dot_angular_rate_des_f
                                          - W_.transpose()*angle_error_mod
                                          - cIntAngle_*W_.transpose()*angle_error_integral_
                                          - cRate_*angular_rate_error;
  // angular_accel_vec(2) = pidYawRate_.update_f(, odometry_.angular_velocity(2), 0.0, dt_.toSec());
  // std::cout<<"cAngle:"<<cAngle_(0,0)<<", cRate:"<<cRate_(0,0) << std::endl;
  // Project thrust onto body z axis.
  //double thrust = vehicle_parameters_.mass_ *((gravity_N_ - acceleration(2))*load_factor_ - gravity_N_);
  double thrust = -vehicle_parameters_.mass_ * acceleration.dot(odometry_.orientation.toRotationMatrix().col(2));

  Eigen::Vector4d angular_acceleration_thrust;
  angular_acceleration_thrust.block<3, 1>(0, 0) = angular_accel_vec;
  angular_acceleration_thrust(3) = thrust;
  //std::cout<<"ax_ang:"<<angular_accel_vec(0)<<", ay_ang:"<<angular_accel_vec(1)<<", az_ang"<<angular_accel_vec(2)<<", T:"<<thrust<<std::endl;

  *rotor_velocities = angular_acc_to_rotor_velocities_ * angular_acceleration_thrust;
  rotor_velocities_sqr_ = *rotor_velocities;
  *rotor_velocities = rotor_velocities->cwiseMax(Eigen::VectorXd::Zero(rotor_velocities->rows()));
  *rotor_velocities = rotor_velocities->cwiseSqrt();
  *rotor_velocities = rotor_velocities->cwiseMin(rotor_velocities_max_);
  /* Eigen::VectorXd rotor_vels(6);
  rotor_vels = *rotor_velocities;
  std::cout<<"w1:"<<rotor_vels(0)<<", w2:"<<rotor_vels(1)<<", w3:"<<rotor_vels(2)
                 <<", w4:"<<rotor_vels(3)<<", w5:"<<rotor_vels(4)<<", w6:"<<rotor_vels(5)<<std::endl; // */
}

void LandingController::LQT(Eigen::VectorXd* rotor_velocities) const {
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
  const Eigen::Vector3d current_kardan_angles      = Eigen::Vector3d(atan2(R_W_B(1,0),R_W_B(0,0)), asin(-R_W_B(2,0)), atan2(R_W_B(2,1), R_W_B(2,2)));
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
  ComputeSystemInputs_LQT(rotor_velocities, current_state, t);

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

// tracking of trajectories described by exogenous system using linear quadratic methods
void LandingController::ComputeSystemInputs_LQT(Eigen::VectorXd* rotor_velocities,
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
   //intialize here:
   your_ref << odometry_.position(0) - traj_error_cmd_accel_yaw_.position_W(0),
                        odometry_.position(1) - traj_error_cmd_accel_yaw_.position_W(1),
                        odometry_.position(2) - traj_error_cmd_accel_yaw_.position_W(2),
                        0.0, 0.0, cmd_yaw_;

   Eigen::VectorXd operating_point(6);
   operating_point << 0.0, 0.0, 0.0,  0.0, 0.0, M_PI;

   Eigen::MatrixXd O(6,1);
   O = your_ref - operating_point; // conversion vec -> mat !?


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
  Q_y <<	10, 0, 0,  0, 0, 0,
                0, 10, 0,  0, 0, 0,
                0, 0,10, 0, 0, 0,
                0, 0, 0,  1, 0, 0,
                0, 0, 0,  0, 1, 0,
                0, 0, 0,  0, 0, 10;
  Q_y *= 100;

  Eigen::MatrixXd D_d(6,1);
  D_d <<	0, 0, 0,  0, 0, 0;

  Eigen::MatrixXd S_w(12,1);
  //S_w = C.transposeInPlace()*Q_y*(D_d - O);
  S_w = C.transpose()*Q_y*(D_d - O);

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
  //Pi_v = - Abar.transposeInPlace().inverse() * (Pbar*E_d + S_w);
  Eigen::MatrixXd Pi_v(12, 1);
  Pi_v = (Abar.transpose()).inverse()*(- Pbar*E_d - S_w);

  Eigen::MatrixXd R =  Eigen::MatrixXd::Identity(6,6);

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
    // F_matrix = - R.inverse() * B.transposeInPlace() * Pi_v;
    F_matrix = - (R.inverse() * B.transpose() * Pi_v);


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

// Implementation from the T. Lee et al. paper
// Control of complex maneuvers for a quadrotor UAV using geometric methods on SE(3)
void LandingController::ComputeDesiredAngularAcc(const Eigen::Vector3d& acceleration,
                                                     Eigen::Vector3d* angular_acceleration)
{
  assert(angular_acceleration);

  // Define angle error and its matrix form and angular rate error for this iteration.
  Eigen::Vector3d angle_error(Eigen::Vector3d::Zero()), angular_rate_error(Eigen::Vector3d::Zero());
  Eigen::Matrix3d angle_error_matrix;

  // Define desired angular rate for this iteration.
  Eigen::Matrix3d R_des;
  Eigen::Vector3d angular_rate_des(Eigen::Vector3d::Zero());
  Eigen::Matrix3d angular_rate_des_matrix;

  double cmd_yaw;
  if (external_error_)
  {
    cmd_yaw = clampRotation(cmd_yaw_);
  }
  else
  {
    cmd_yaw = clampRotation(command_trajectory_.getYaw());
  }

  //  std::cout<<"cmd_yaw"<<cmd_yaw<<std::endl;

  double cmd_yaw_fake = yaw_;
  // Calculate sin(roll)
  double sr = (-acceleration(0)*sin(cmd_yaw_fake) + acceleration(1) *cos(cmd_yaw_fake))
                           /sqrt(acceleration(0)*acceleration(0) + acceleration(1)*acceleration(1) + acceleration(2)*acceleration(2));

  if (sr > 1.0)
  {
    sr = 1.0;
  }
  else if (sr < -1.0)
  {
    sr = -1.0;
  }

  double cmd_roll_temp = asin(sr);
  double cmd_pitch_temp = atan2(-acceleration(0)*cos(cmd_yaw_fake) - acceleration(1)*sin(cmd_yaw_fake), -acceleration(2)); // In this one there is no Accel(2) part in the denominator

  //  /*
  double max_tilt_angle = 0.36 ;// max_tilt_angle_ ;
  double cmd_roll_limit = boost::math::sign(cmd_roll_temp)*std::min(fabs(cmd_roll_temp), max_tilt_angle);
  double temp_term = cos(max_tilt_angle)/cos(fabs(cmd_roll_limit));
  if (temp_term > 1.0)
  {temp_term = 1.0;}
  double max_pitch_temp = acos(temp_term);
  double cmd_pitch_limit = boost::math::sign(cmd_pitch_temp)*std::min(fabs(cmd_pitch_temp), max_pitch_temp);
  //cmd_roll_ = cmd_roll_limit;
  //cmd_pitch_ = cmd_pitch_limit;  // */

  cmd_roll_ = cmd_roll_temp;
  cmd_pitch_ = cmd_pitch_temp;
  // std::cout<<"cmd_roll:"<<cmd_roll_<<", pitch:"<<cmd_pitch_<<std::endl;

  // for test
  /* if (t_.toSec() > 9.0)
  {cmd_roll_ = 0.3; cmd_pitch_ = 0.3; cmd_yaw = 0.3;}
  else if (t_.toSec() > 6.0)
  {cmd_roll_ = 0.3; cmd_pitch_ = 0.3; cmd_yaw = 0.0;}
  else if (t_.toSec() > 3.0)
  {cmd_roll_ = 0.0; cmd_pitch_ = 0.3; cmd_yaw = 0.0;} // */

  // Get the desired rotation matrix. original
  /*
  Eigen::Vector3d b1_des;
  b1_des << cos(cmd_yaw), sin(cmd_yaw), 0;  // cmd_yaw_ should be defined globally

  Eigen::Vector3d b3_des;
  b3_des = -acceleration / acceleration.norm();

  Eigen::Vector3d b2_des;
  b2_des = b3_des.cross(b1_des);
  b2_des.normalize();

  R_des.col(0) = b2_des.cross(b3_des);
  R_des.col(1) = b2_des;
  R_des.col(2) = b3_des;  // */

  // Get the desired rotation matrix.
  R_des = Eigen::AngleAxisd(cmd_yaw, Eigen::Vector3d::UnitZ())                              // yaw_des
               * Eigen::AngleAxisd(cmd_roll_, Eigen::Vector3d::UnitX())                             // roll_des
               * Eigen::AngleAxisd(cmd_pitch_, Eigen::Vector3d::UnitY());                         // pitch_des  */

  // Define R_des differential tracker variables for this iteration.
  Eigen::VectorXd R_des_vec(9), R_des_vec_f(9), dot_R_des_vec_f(9);
  for (int i=0; i<3;i++)
  {
    for (int j=0; j<3;j++)
    {
      R_des_vec(3*i+j) = R_des(i, j);
    }
  }
  R_des_vec_f = R_des_vec;
  R_des_tracker_.track(R_des_vec, R_des_vec_f, dot_R_des_vec_f, dt_.toSec());

  Eigen::Matrix3d dot_R_des_f;
  for (int i=0; i<3;i++)
  {
    for (int j=0; j<3;j++)
    {
      dot_R_des_f(i, j) = dot_R_des_vec_f(3*i+j);
    }
  }
  angular_rate_des_matrix = R_des.transpose()*dot_R_des_f;
  vectorFromSkewMatrix(angular_rate_des_matrix, &angular_rate_des);
  //angular_rate_des(2) = traj_error_cmd_accel_yaw_.getYawRate();
  //if (fabs(angular_rate_des(2)) > 0.5) {angular_rate_des(2) =0.5*boost::math::sign(angular_rate_des(2));}

  // Angle error according to lee et al.
  angle_error_matrix = 0.5 * (R_des.transpose() * R_ - R_.transpose() * R_des);
  vectorFromSkewMatrix(angle_error_matrix, &angle_error);

  Eigen::VectorXd angular_rate_des_f(3), dot_angular_rate_des_f(3);
  angular_rate_des_tracker_.track(angular_rate_des, angular_rate_des_f, dot_angular_rate_des_f, dt_.toSec());

  Eigen::Matrix3d angular_rate_matrix;
  skewMatrixFromVector(odometry_.angular_velocity, &angular_rate_matrix);
  angular_rate_error = odometry_.angular_velocity - R_.transpose() * R_des * angular_rate_des;
  angle_error_integral_ += angle_error*dt_.toSec();

  *angular_acceleration = -1 * angle_error.cwiseProduct(normalized_attitude_gain_)
                                                  // - angle_error_integral_.cwiseProduct(normalized_attitude_integral_gain_)
                                                  - angular_rate_error.cwiseProduct(normalized_angular_rate_gain_)
                                                  + odometry_.angular_velocity.cross(odometry_.angular_velocity);
                                                  - (angular_rate_matrix*R_.transpose()*R_des*angular_rate_des - R_.transpose()*R_des*dot_angular_rate_des_f); // we don't need the inertia matrix here
}

void LandingController::PIDAcceleration(Eigen::Vector3d* acceleration) {
  assert(acceleration);
  Eigen::VectorXd position_error(3), position_error_f(3),  velocity_error(3), cmd_Vel(3);
  Eigen::VectorXd cmd_accel_N_(3);

  if (external_error_)
  {
    position_error = traj_error_cmd_accel_yaw_.position_W;
    // std::cout<<"poserror:"<<position_error<<std::endl;

    if (use_position_error_tracker_)
    {
      position_error_tracker_.track(position_error, position_error_f, velocity_error, dt_.toSec());
      position_error = position_error_f;
    }
    else
    {
      velocity_error = traj_error_cmd_accel_yaw_.velocity_W;
    }

   cmd_accel_N_ = traj_error_cmd_accel_yaw_.acceleration_W;
  }
  else
  {
    position_error = odometry_.position - command_trajectory_.position_W;
    position_error_tracker_.reportX(command_trajectory_.position_W); // test
    // Transform velocity to world frame.
    Eigen::Vector3d velocity_W =  R_ * odometry_.velocity; // This odometry_.velocity is in {B}-frame
    velocity_error = velocity_W - command_trajectory_.velocity_W;

    cmd_accel_N_ = command_trajectory_.acceleration_W;
  }

  // std::cout<<"wtffffffffffffffffffffffffffffffff"<<std::endl;
  // PID position controller
  Eigen::Vector3d cmd_position = odometry_.position - position_error;
  Eigen::Vector3d accel_unlimited;
  Eigen::Vector3d velocity_W =  R_ * odometry_.velocity;
  Eigen::Vector3d e_3(Eigen::Vector3d::UnitZ());
  cmd_Vel(0) = pidX_.update_f(cmd_position(0),  odometry_.position(0), 0.0, dt_.toSec());// - cmd_accel_N_(0);
  cmd_Vel(1) = pidY_.update_f(cmd_position(1),  odometry_.position(1), 0.0, dt_.toSec());// - cmd_accel_N_(1);
  cmd_Vel(2) = pidZ_.update_f(cmd_position(2),  odometry_.position(2), 0.0, dt_.toSec()); // - cmd_accel_N_(2); //*/
  // std::cout<<"cmd_Vel x:"<<cmd_Vel(0)<<", y:"<<cmd_Vel(1)<<", z:"<<cmd_Vel(2)<<std::endl;

  // PID twist controller
  accel_unlimited(0) = pidVx_.update_f(cmd_Vel(0), velocity_W(0), 0.0, dt_.toSec()); //- cmd_accel_N_(0),
  accel_unlimited(1) = pidVy_.update_f(cmd_Vel(1), velocity_W(1), 0.0, dt_.toSec()); // - cmd_accel_N_(1),
  accel_unlimited(2) = pidVz_.update_f(cmd_Vel(2), velocity_W(2), 0.0, dt_.toSec()) + vehicle_parameters_.gravity_*e_3(2); // - cmd_accel_N_(2); //*/
  //std::cout<<"ax:"<<accel_unlimited(0)<<", ay:"<<accel_unlimited(1)<<", az:"<<accel_unlimited(2)<<std::endl;
  for (int i = 0;i<2;i++)
  {
    accel_unlimited(i) =boost::math::sign(accel_unlimited(i))*std::min(4.0, fabs(accel_unlimited(i)));
  }  // */
  *acceleration = accel_unlimited;
  //Eigen::Vector3d temp(*acceleration);
  //std::cout<<"dt:"<<dt_.toSec()<<", error x:"<<velocity_error(0)<<", y:"<<velocity_error(1)<<", z:"<<velocity_error(2)<<", cmd Ax:"<<temp(0)<<", Ay:"<<temp(1)<<", Az:"<<temp(2)<<std::endl;
}

void LandingController::PIDAngularAcc(const Eigen::Vector3d& acceleration,
                                                     Eigen::Vector3d* angular_acceleration)
{
  assert(angular_acceleration);

  // Define angle error and its matrix form and angular rate error for this iteration.
  Eigen::Vector3d angle_error(Eigen::Vector3d::Zero()), angular_rate_error(Eigen::Vector3d::Zero());
  Eigen::Matrix3d angle_error_matrix;

  // Define desired angular rate for this iteration.
  Eigen::Matrix3d R_des;
  Eigen::Vector3d angular_rate_des(Eigen::Vector3d::Zero());
  Eigen::Matrix3d angular_rate_des_matrix;

  double cmd_yaw;
  if (external_error_)
  {
    cmd_yaw = clampRotation(cmd_yaw_);
  }
  else
  {
    cmd_yaw = clampRotation(command_trajectory_.getYaw());
  }

  //  std::cout<<"cmd_yaw"<<cmd_yaw<<std::endl;

  double cmd_yaw_fake = yaw_;
  // Calculate sin(roll)
  double sr = (acceleration(0)*sin(cmd_yaw_fake) - acceleration(1) *cos(cmd_yaw_fake))
                           /sqrt(acceleration(0)*acceleration(0) + acceleration(1)*acceleration(1) + acceleration(2)*acceleration(2));

  if (sr > 1.0)
  {
    sr = 1.0;
  }
  else if (sr < -1.0)
  {
    sr = -1.0;
  }

  double cmd_roll_temp = asin(sr);
  double cmd_pitch_temp = atan2(acceleration(0)*cos(cmd_yaw_fake) + acceleration(1)*sin(cmd_yaw_fake), acceleration(2)); // In this one there is no Accel(2) part in the denominator

  //  /*
  double max_tilt_angle = 0.36 ;// max_tilt_angle_ ;
  double cmd_roll_limit = boost::math::sign(cmd_roll_temp)*std::min(fabs(cmd_roll_temp), max_tilt_angle);
  double temp_term = cos(max_tilt_angle)/cos(fabs(cmd_roll_limit));
  if (temp_term > 1.0)
  {temp_term = 1.0;}
  double max_pitch_temp = acos(temp_term);
  double cmd_pitch_limit = boost::math::sign(cmd_pitch_temp)*std::min(fabs(cmd_pitch_temp), max_pitch_temp);
  //cmd_roll_ = cmd_roll_limit;
  //cmd_pitch_ = cmd_pitch_limit;  // */

  cmd_roll_ = cmd_roll_temp;
  cmd_pitch_ = cmd_pitch_temp;
  // std::cout<<"cmd_roll:"<<cmd_roll_<<", pitch:"<<cmd_pitch_<<std::endl;

  // Get the desired rotation matrix. original
  /*
  Eigen::Vector3d b1_des;
  b1_des << cos(cmd_yaw), sin(cmd_yaw), 0;  // cmd_yaw_ should be defined globally

  Eigen::Vector3d b3_des;
  b3_des = acceleration / acceleration.norm();

  Eigen::Vector3d b2_des;
  b2_des = b3_des.cross(b1_des);
  b2_des.normalize();

  R_des.col(0) = b2_des.cross(b3_des);
  R_des.col(1) = b2_des;
  R_des.col(2) = b3_des;  // */

  // for test
  /* if (t_.toSec() > 9.0)
  {cmd_roll_ = 0.3; cmd_pitch_ = 0.3; cmd_yaw = 0.3;}
  else if (t_.toSec() > 6.0)
  {cmd_roll_ = 0.3; cmd_pitch_ = 0.3; cmd_yaw = 0.0;}
  else if (t_.toSec() > 3.0)
  {cmd_roll_ = 0.0; cmd_pitch_ = 0.3; cmd_yaw = 0.0;} // */

  // Get the desired rotation matrix.
  R_des = Eigen::AngleAxisd(cmd_yaw, Eigen::Vector3d::UnitZ())                  // yaw_des
               * Eigen::AngleAxisd(cmd_roll_, Eigen::Vector3d::UnitX())                             // roll_des
               * Eigen::AngleAxisd(cmd_pitch_, Eigen::Vector3d::UnitY());                            // pitch_des  */

  // Define R_des differential tracker variables for this iteration.
  Eigen::VectorXd R_des_vec(9), R_des_vec_f(9), dot_R_des_vec_f(9);
  for (int i=0; i<3;i++)
  {
    for (int j=0; j<3;j++)
    {
      R_des_vec(3*i+j) = R_des(i, j);
    }
  }
  R_des_vec_f = R_des_vec;
  R_des_tracker_.track(R_des_vec, R_des_vec_f, dot_R_des_vec_f, dt_.toSec());

  Eigen::Matrix3d dot_R_des_f;
  for (int i=0; i<3;i++)
  {
    for (int j=0; j<3;j++)
    {
      dot_R_des_f(i, j) = dot_R_des_vec_f(3*i+j);
    }
  }
  angular_rate_des_matrix = R_des.transpose()*dot_R_des_f;
  vectorFromSkewMatrix(angular_rate_des_matrix, &angular_rate_des);
  //angular_rate_des(2) = traj_error_cmd_accel_yaw_.getYawRate();
  //if (fabs(angular_rate_des(2)) > 0.5) {angular_rate_des(2) =0.5*boost::math::sign(angular_rate_des(2));}

  // Angle error according to lee et al.
  angle_error_matrix = 0.5 * (R_des.transpose() * R_ - R_.transpose() * R_des);
  vectorFromSkewMatrix(angle_error_matrix, &angle_error);

  Eigen::VectorXd angular_rate_des_f(3), dot_angular_rate_des_f(3);
  angular_rate_des_tracker_.track(angular_rate_des, angular_rate_des_f, dot_angular_rate_des_f, dt_.toSec());

  Eigen::Matrix3d angular_rate_matrix;
  skewMatrixFromVector(odometry_.angular_velocity, &angular_rate_matrix);
  angular_rate_error = odometry_.angular_velocity - R_.transpose() * R_des * angular_rate_des;
  angle_error_integral_ += angle_error*dt_.toSec();

  Eigen::Vector3d angular_accel_star;
  angular_accel_star << pidRoll_.update_f(cmd_roll_, roll_, 0.0, dt_.toSec()),
                                              pidPitch_.update_f(cmd_pitch_, pitch_, 0.0, dt_.toSec()),
                                              pidYaw_.update_f(cmd_yaw, yaw_, 0.0, dt_.toSec()); //*/

  *angular_acceleration = odometry_.angular_velocity.cross(odometry_.angular_velocity) - c_*odometry_.angular_velocity + c_*angular_accel_star;
  /* *angular_acceleration = -1 * angle_error.cwiseProduct(normalized_attitude_gain_)
                                                  // - angle_error_integral_.cwiseProduct(normalized_attitude_integral_gain_)
                                                  - angular_rate_error.cwiseProduct(normalized_angular_rate_gain_)
                                                  + odometry_.angular_velocity.cross(odometry_.angular_velocity);
                                                  - (angular_rate_matrix*R_.transpose()*R_des*angular_rate_des - R_.transpose()*R_des*dot_angular_rate_des_f); // we don't need the inertia matrix here */
}

}
