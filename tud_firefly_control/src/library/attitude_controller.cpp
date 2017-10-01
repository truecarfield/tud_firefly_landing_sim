#include "tud_firefly_control/attitude_controller.h"

namespace rotors_control{

FireflyAttitudeController::FireflyAttitudeController()
  : gravity_N_(-9.80665),
    controller_active_(false),
    external_linear_error_(false),
    external_angular_error_(false),
    bool_test_ (false)
{}

FireflyAttitudeController::~FireflyAttitudeController() {
}

void FireflyAttitudeController::InitializeParams(ros::NodeHandle& pnh) {
  // Read parameters from rosparam.
  pidZ_.initialize(ros::NodeHandle(pnh, "linear/z"));
  GetRosParameter(pnh, "attitude_gain/x",
                  controller_parameters_.attitude_gain_.x(),
                  &controller_parameters_.attitude_gain_.x());
  GetRosParameter(pnh, "attitude_gain/y",
                  controller_parameters_.attitude_gain_.y(),
                  &controller_parameters_.attitude_gain_.y());
  GetRosParameter(pnh, "attitude_gain/z",
                  controller_parameters_.attitude_gain_.z(),
                  &controller_parameters_.attitude_gain_.z());
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

  normalized_attitude_gain_ = controller_parameters_.attitude_gain_.transpose()
      * vehicle_parameters_.inertia_.inverse();
  // To make the tuning independent of the inertia matrix we divide here.
  normalized_angular_rate_gain_ = controller_parameters_.angular_rate_gain_.transpose()
      * vehicle_parameters_.inertia_.inverse();

  // Read inertial.
  Eigen::Matrix4d I;
  I.setZero();
  I.block<3, 3>(0, 0) = vehicle_parameters_.inertia_;
  I(3, 3) = 1;
  // angular_acc_to_rotor_velocities_.resize(vehicle_parameters_.rotor_configuration_.rotors.size(), 4);
  angular_acc_to_rotor_velocities_.resize(6, 4);
  // Calculate the pseude-inverse A^{ \dagger} and then multiply by the inertia matrix I.
  // A^{ \dagger} = A^T*(A*A^T)^{-1}
  angular_acc_to_rotor_velocities_ = controller_parameters_.allocation_matrix_.transpose()
      * (controller_parameters_.allocation_matrix_
      * controller_parameters_.allocation_matrix_.transpose()).inverse() * I;

  initialized_params_ = true;
}

void FireflyAttitudeController::ControllerActivation(const bool &active)
{
  controller_active_ = active;
}

void FireflyAttitudeController::SetPeriod(const ros::Duration& dt)
{
  dt_ = dt;
}

void FireflyAttitudeController::SetCmd(const std::vector<double>& cmds)
{
 if (!cmds.size() == 4)
 {
   ROS_ERROR("Command size for attitude controller is not 4, no commands will be integrated!");
   return;
 }
 cmd_pitch_ = cmds[0];
 cmd_roll_ = -cmds[1];
 cmd_twist_N_ = cmds[2];
 cmd_yaw_rate_ = cmds[3];
}

void FireflyAttitudeController::SetError(const Eigen::Vector3d& p_error, const Eigen::Vector3d& vel_error)
{
  position_error_ = p_error(2);
  twist_error_ = (R_*vel_error)(2);
}

void FireflyAttitudeController::SetOdometry(const EigenOdometry& odometry) {
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

void FireflyAttitudeController::CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) {
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
  ComputeDesiredAngularAcc(&angular_acceleration);

  // Project thrust onto body z axis.
  double thrust = vehicle_parameters_.mass_*acceleration.dot(odometry_.orientation.toRotationMatrix().col(2));

  Eigen::Vector4d angular_acceleration_thrust;
  angular_acceleration_thrust.block<3, 1>(0, 0) = angular_acceleration;
  angular_acceleration_thrust(3) = thrust;

  *rotor_velocities = angular_acc_to_rotor_velocities_ * angular_acceleration_thrust;
  *rotor_velocities = rotor_velocities->cwiseMax(Eigen::VectorXd::Zero(rotor_velocities->rows()));
  *rotor_velocities = rotor_velocities->cwiseSqrt();

  Eigen::MatrixXd test(6, 6);
  test = rotor_velocities->asDiagonal();
  Eigen::FullPivLU<Eigen::MatrixXd> test_(test);
  if (test_.rank() < 6)
  {
    std::cout<<"rotor_velocities has 0 element: "<<std::endl
                    <<*rotor_velocities<<std::endl
                     <<"corresponding accels are:"<<std::endl
                     <<angular_acceleration_thrust<<std::endl;
  }
}

void FireflyAttitudeController::ComputeDesiredAcceleration(Eigen::Vector3d* acceleration) {
  assert(acceleration);
   double error_z_N, error_Vz_N;
   if (external_linear_error_)
   {
     error_z_N = position_error_;
     error_Vz_N = twist_error_;
   }
   else
   {
     cmd_altitude_ += cmd_twist_N_*dt_.toSec(); // which means cmds(3) is still the linear velocity in z direction
     if (cmd_altitude_ < 0.0)
     {cmd_altitude_ = 0.0;}
     std::cout<<"cmd_twist_z:"<<cmd_twist_N_<<", cmd altitude:"<<cmd_altitude_<<std::endl;
     error_z_N = cmd_altitude_ - odometry_.position(2);
     //Eigen::Vector3d odom_Vz_N = R*odometry_.velocity;
     error_Vz_N = cmd_twist_N_ - (R_*odometry_.velocity)(2);
     //error_Vz_N = cmd_twist_N - odom_Vz_N(2);
   }

  *acceleration << 0.0, 0.0, pidZ_.update(error_z_N, error_Vz_N, 0.0, dt_.toSec()) - gravity_N_;
}

// Implementation from the T. Lee et al. paper
// Control of complex maneuvers for a quadrotor UAV using geometric methods on SE(3)
void FireflyAttitudeController::ComputeDesiredAngularAcc(Eigen::Vector3d* angular_acceleration) {
  assert(angular_acceleration);

  // get desired rotation matrix
  Eigen::Vector3d angle_error, angular_rate_error;

  if (external_angular_error_)
  {
    angle_error = angle_error_;
    angular_rate_error = angular_rate_error_;
  }
  else
  {
    Eigen::Matrix3d R_des;
    R_des = Eigen::AngleAxisd(yaw_, Eigen::Vector3d::UnitZ())                       // yaw_des = yaw
                  * Eigen::AngleAxisd(cmd_roll_, Eigen::Vector3d::UnitX())             // roll_des
                  * Eigen::AngleAxisd(cmd_pitch_, Eigen::Vector3d::UnitY());         // pitch_des

    // angle error according to lee et al.
    Eigen::Matrix3d angle_error_matrix = 0.5 * (R_des.transpose() * R_ - R_.transpose() * R_des);
    angle_error << angle_error_matrix(2, 1),  // inverse skew operator
                                  angle_error_matrix(0, 2),
                                  0;                                                // angle_error_matrix(1,0);

    Eigen::Vector3d angular_rate_des(Eigen::Vector3d::Zero());
    angular_rate_des[2] = cmd_yaw_rate_;
    angular_rate_error = odometry_.angular_velocity - R_des.transpose() * R_ * angular_rate_des;
  }
  *angular_acceleration = -1 * angle_error.cwiseProduct(normalized_attitude_gain_)
                           - angular_rate_error.cwiseProduct(normalized_angular_rate_gain_)
                           + odometry_.angular_velocity.cross(odometry_.angular_velocity); // we don't need the inertia matrix here
}

}
