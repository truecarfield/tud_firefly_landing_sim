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
      gravity_N_(-9.80665),
      gg_N_(gravity_N_*gravity_N_),
      max_tilt_angle_(0.5),
      PI_(3.141592653589793)
{
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

  // Initialize the backstepping controller.
  ypsilon_.setZero();
  cAngle_ = normalized_attitude_gain_.asDiagonal();
  cIntAngle_ = normalized_attitude_integral_gain_.asDiagonal();
  cRate_ = normalized_angular_rate_gain_.asDiagonal();

  std::cout<<"cAngle:"<<std::endl
                   <<cAngle_<<std::endl
                   <<"cIntAngle:"<<std::endl
                   <<cIntAngle_<<std::endl
                   <<"cRate:"<<std::endl
                   <<cRate_<<std::endl;

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
  cmd_velocity_tracker_.init(pnh, "cmd_velocity");
  position_error_tracker_.init(pnh, "position_error");

  // Initialize vector form of inertia
  J_(0) = vehicle_parameters_.inertia_(0,0);
  J_(1) = vehicle_parameters_.inertia_(1,1);
  J_(2) = vehicle_parameters_.inertia_(2,2);

  // Initialize the PID controllers
  pidX_.initialize(ros::NodeHandle(pnh, "linear/xy"));
  pidY_.initialize(ros::NodeHandle(pnh, "linear/xy"));
  pidZ_.initialize(ros::NodeHandle(pnh, "linear/z"));
  pidYaw_.initialize(ros::NodeHandle(pnh, "angular/yaw"));

  pidVx_.initialize(ros::NodeHandle(pnh, "linear/Vxy"));
  pidVy_.initialize(ros::NodeHandle(pnh, "linear/Vxy"));
  pidVz_.initialize(ros::NodeHandle(pnh, "linear/Vz"));
  pidYawRate_.initialize(ros::NodeHandle(pnh, "angular/r"));

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

void LandingController::SetTrajErrorCmdAccelYawPoint(const mav_msgs::EigenTrajectoryPoint& traj_error_cmd_yaw)
{
  traj_error_cmd_accel_yaw_ = traj_error_cmd_yaw;
  // std::cout<<"traj_error_point z:"<<traj_error_cmd_yaw.position_W(2)<<std::endl;
}

void LandingController::reset()
{
  traj_error_cmd_accel_yaw_.position_W.setZero();
  traj_error_cmd_accel_yaw_.velocity_W.setZero();
  traj_error_cmd_accel_yaw_.angular_velocity_W.setZero();
  traj_error_cmd_accel_yaw_.setFromYaw(yaw_);
  traj_error_cmd_accel_yaw_.acceleration_W.setZero();

  R_des_tracker_.reset();
  angular_rate_des_tracker_.reset();

  cmd_roll_ = 0.0;
  cmd_yaw_ = 0.0;
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
  reset();
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
  if (!pid_mode_)
  {
    // Lee position controller
    position_integral_error_ += position_error*dt_.toSec();
    *acceleration = (position_error.cwiseProduct(controller_parameters_.position_gain_)
                                 // + position_integral_error_.cwiseProduct(controller_parameters_.position_integral_gain_)
                                 + velocity_error.cwiseProduct(controller_parameters_.velocity_gain_)) / vehicle_parameters_.mass_
                                 - cmd_accel_N_.cwiseProduct(controller_parameters_.acceleration_gain_)
                                 - vehicle_parameters_.gravity_ * e_3; // */
  }
  else
  {
    // PID twist controller
    *acceleration << pidX_.update_f(velocity_error(0), std::numeric_limits<double>::quiet_NaN(), 0.0, dt_.toSec()); // - cmd_accel_N_(0),
                                      pidY_.update_f(velocity_error(1), std::numeric_limits<double>::quiet_NaN(), 0.0, dt_.toSec()); // - cmd_accel_N_(1),
                                      pidZ_.update_f(velocity_error(2), std::numeric_limits<double>::quiet_NaN(), 0.0, dt_.toSec()) - vehicle_parameters_.gravity_*e_3(2); // - cmd_accel_N_(2);

   // Eigen::Vector3d temp(*acceleration);
   // std::cout<<"dt:"<<dt_.toSec()<<", error x:"<<position_error(0)<<", y:"<<position_error(1)<<", z:"<<position_error(2)<<", cmd Ax:"<<temp(0)<<", Ay:"<<temp(1)<<", Az:"<<temp(2)<<std::endl;
  }
}

// Implementation from the T. Lee et al. paper
// Control of complex maneuvers for a quadrotor UAV using geometric methods on SE(3)
void LandingController::ComputeDesiredAngularAcc(const Eigen::Vector3d& acceleration,
                                                     Eigen::Vector3d* angular_acceleration)
{
  assert(angular_acceleration);

  // Define angle error and its matrix form and angular rate error for this iteration.
  Eigen::Vector3d angle_error, angular_rate_error, angle_error_mod;
  //Eigen::Matrix3d angle_error_matrix;

  // Define desired angular rate for this iteration.
  //Eigen::Matrix3d R_des;
  Eigen::VectorXd angular_rate_des(3), angular_rate_cmd(3), dot_angular_rate_cmd(3);

  double cmd_yaw;
  if (external_error_)
  {
    cmd_yaw = clampRotation(traj_error_cmd_accel_yaw_.getYaw());
  }
  else
  {
    cmd_yaw = clampRotation(command_trajectory_.getYaw());
  }

  double cmd_yaw_fake = cmd_yaw;
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

  /*
  double cmd_roll_limit = boost::math::sign(cmd_roll_temp)*std::min(fabs(cmd_roll_temp), max_tilt_angle_);
  double temp_term = cos(max_tilt_angle_)/cos(fabs(cmd_roll_limit));
  if (temp_term > 1.0)
  {temp_term = 1.0;}
  double max_pitch_temp = acos(temp_term);
  double cmd_pitch_limit = boost::math::sign(cmd_pitch_temp)*std::min(fabs(cmd_pitch_temp), max_pitch_temp);
  cmd_roll_ = cmd_roll_limit;
  cmd_pitch_ = cmd_pitch_limit;
  //cmd_roll_ = cmd_roll_temp;
  //cmd_pitch_ = cmd_pitch_temp;
  if (test_active_)
  {
    cmd_roll_ = 0.3;
    cmd_pitch_ = 0.0;
  }
  else
  {
    cmd_roll_ = 0.0;
    cmd_pitch_ = 0.0;
  }

  //R_des = Eigen::AngleAxisd(cmd_yaw_fake, Eigen::Vector3d::UnitZ())                      // yaw_des
  //             * Eigen::AngleAxisd(cmd_roll_, Eigen::Vector3d::UnitX())                                // roll_des
  //             * Eigen::AngleAxisd(cmd_pitch_, Eigen::Vector3d::UnitY());                            // pitch_des  */

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

  // Angle error according to lee et al.
  //angle_error_matrix = 0.5 * (R_des.transpose() * R_ - R_.transpose() * R_des);
  //vectorFromSkewMatrix(angle_error_matrix, &angle_error);

  // Define R_des differential tracker variables for this iteration.
  Eigen::VectorXd RPY(3), dot_RPY(3), RPY_c(3), RPY_c_f(3), dot_RPY_c_f(3);
  RPY << roll_, pitch_, yaw_;
  dot_RPY = W_*odometry_.angular_velocity;

  RPY_c << cmd_roll_, cmd_pitch_, cmd_yaw;
  RPY_c_f << 0.0, 0.0, 0.0;

  angle_error = RPY - RPY_c;
  R_des_tracker_.track(RPY_c, RPY_c_f, dot_RPY_c_f, dt_.toSec());

  angular_rate_des = W_.inverse()*(dot_RPY - cAngle_*angle_error);
  angular_rate_cmd << 0.0, 0.0, 0.0;
  angular_rate_des_tracker_.track(angular_rate_des, angular_rate_cmd, dot_angular_rate_cmd, dt_.toSec());

  angular_rate_error = odometry_.angular_velocity - angular_rate_cmd;
  ypsilon_ += (-cAngle_*ypsilon_ + W_*(angular_rate_cmd - angular_rate_des))*dt_.toSec();

  angle_error_mod = angle_error - ypsilon_;
  angle_error_integral_ += angle_error*dt_.toSec();

  Eigen::Vector3d angular_accel_vec;
  angular_accel_vec = odometry_.angular_velocity.cross(odometry_.angular_velocity)
                                          + dot_angular_rate_cmd
                                          - W_.transpose()*angle_error_mod
                                          // - cIntAngle_*W_.transpose()*angle_error_integral_
                                          - cRate_*angular_rate_error;
  angular_accel_vec(2) = pidYawRate_.update_f(dot_RPY_c_f(2), odometry_.angular_velocity(2), 0.0, dt_.toSec());

  *angular_acceleration = angular_accel_vec;
}

}
