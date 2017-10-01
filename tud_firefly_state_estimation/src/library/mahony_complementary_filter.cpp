#include "tud_firefly_state_estimation/mahony_complementary_filter.h"

namespace RobotLocalization {

MahonyCompFilter::MahonyCompFilter():
  gravity_(9.80665),
  gg_(gravity_*gravity_),
  twoKp1_(2.0),
  twoKp2_(2.0),
  twoKi1_(0.1),
  twoKi2_(0.1),
  q_norm(0.0),
  q0(1.0),
  q1(0.0),
  q2(0.0),
  q3(0.0),
  dq0(0.0),
  dq1(0.0),
  dq2(0.0),
  dq3(0.0),
  state_initialized_(false),
  enable_mag_(true)
{
   gyrBias_.setZero();
   mag_norm_b_.setZero();
   acc_norm_b_.setZero();
}

void MahonyCompFilter::sendTest(Eigen::Vector3d& g_b_)
{
  g_b_ = gyrBias_;
}

MahonyCompFilter::~MahonyCompFilter() {}

void MahonyCompFilter::initParams(
                                          const Eigen::Vector4d& k,
                                          const Eigen::Vector3d& tauGyr,
                                          const Eigen::Vector3d& tauAccel,
                                          const Eigen::Vector3d& tauMag,
                                          const std::vector<int>& tauGyrConfig,
                                          const std::vector<int>& tauAccelConfig,
                                          const std::vector<int>& tauMagConfig)
{
  twoKp1_ = 2*k(0);
  twoKi1_ = 2*k(1);
  twoKp2_ = 2*k(2);
  twoKi2_ = 2*k(3);

  gyrTracker_.init(tauGyr.size(), tauGyr, tauGyrConfig);
  accelTracker_.init(tauAccel.size(), tauAccel, tauAccelConfig);
  magnTracker_.init(tauMagConfig.size(), tauMag, tauMagConfig);
}

void MahonyCompFilter::initState(double Au, double Av, double Aw, double Mu, double Mv, double Mw)
{
  double initialRoll, initialPitch, initialYaw;
  double cr, sr, cp, sp, cy, sy;                                 // cos(roll), sin(roll), cos(pitch), sin(pitch), cos(yaw), sin(yaw)
  double Mx, My;                                                       // Magnetic field in {N} frame

  initialRoll = asin(-Av/gravity_);
  initialPitch = asin(Au/gravity_);

  cr = cos(initialRoll);
  sr = sin(initialRoll);
  cp = cos(initialPitch);
  sp = sin(initialPitch);

  Mx = Mu * cp + Mv * sr * sp + Mw * cr * sp;
  My = Mv * cr - Mw * sr;

  if (enable_mag_)
  {
    initialYaw = atan2(- My, Mx); // ?
  }
  else
  {
    initialYaw = 0.0;
  }
  cy = cos(initialYaw);
  sy = sin(initialYaw);

  q0 = cr * cp * cy + sr * sp * sy;
  q1 = sr * cp * cy - cr * sp * sy;
  q2 = cr * sp * cy + sr * cp * sy;
  q3 = cr * cp * sy - sr * sp * cy;

  // auxillary variables to reduce number of repeated operations, for 1st pass
  q0q0 = q0 * q0;
  q0q1 = q0 * q1;
  q0q2 = q0 * q2;
  q0q3 = q0 * q3;
  q1q1 = q1 * q1;
  q1q2 = q1 * q2;
  q1q3 = q1 * q3;
  q2q2 = q2 * q2;
  q2q3 = q2 * q3;
  q3q3 = q3 * q3;
  load_factor_ = 1. / (q0q0 - q1q1 - q2q2 + q3q3);
}

void MahonyCompFilter::update(const Eigen::Vector3d& accData,
                                                                 const Eigen::Vector3d& gyrData,
                                                                 const Eigen::Vector3d& magData,
                                                                 const double& dt,
                                                                 Eigen::Quaterniond& filteredData)
{
  if (!state_initialized_)
  {
    initState(accData(0), accData(1), accData(2), magData(0), magData(1), magData(2));
    state_initialized_ = true;
  }

  // Normalized magnetometer measurement
  double magNorm = magData.norm();
  if (!magNorm == 0.0 && enable_mag_)
  {
    // HALF of the magnetic field in {B}-frame implied with estimated orientation, HALF is for saving calculation resources
    double half_mag_norm_u_hat_, half_mag_norm_v_hat_, half_mag_norm_w_hat_;
    // Normalized magnetic field in estimated N-frame
    double mag_norm_x_, mag_norm_y_, mag_norm_z_, mag_norm_xy_;

    mag_norm_b_ = magData/magNorm;
    // Reference direction of Earth's magnetic field
    mag_norm_x_ = 2.0*(mag_norm_b_(0)*(0.5 - q2q2 - q3q3)
                                          + mag_norm_b_(1)*( q1q2 - q0q3)
                                          + mag_norm_b_(2)*(q1q3 + q0q2));

    mag_norm_y_ = 2.0*(mag_norm_b_(0)*(q1q2 + q0q3)
                                          + mag_norm_b_(1)*(0.5 - q1q1 - q3q3)
                                          + mag_norm_b_(2)*(q2q3 - q0q1));

    mag_norm_z_ = 2.0* mag_norm_b_(0)*(q1q3 - q0q2) +
                                    2.0* mag_norm_b_(1)*(q2q3 + q0q1) +
                                    2.0* mag_norm_b_(2)*(0.5 - q1q1 - q2q2);

    // This step seems like force the estimated mag field in x direction in {N}-frame
    mag_norm_xy_ = sqrt(mag_norm_x_*mag_norm_x_ + mag_norm_y_*mag_norm_y_);
    // HALF of estimated direction of magnetic field in {B}-frame
    half_mag_norm_u_hat_ = mag_norm_xy_ * (0.5 - q2q2 - q3q3) + mag_norm_z_ * (q1q3 - q0q2);
    half_mag_norm_v_hat_ = mag_norm_xy_ * (q1q2 - q0q3) + mag_norm_z_  * (q0q1 + q2q3);
    half_mag_norm_w_hat_ = mag_norm_xy_ * (q0q2 + q1q3) + mag_norm_z_ * (0.5 - q1q1 - q2q2);

    // Error is sum of cross product between estimated direction and measured direction of field vectors
    half_em_b_(0) = (mag_norm_b_(1) * half_mag_norm_w_hat_ - mag_norm_b_(2) * half_mag_norm_v_hat_);
    half_em_b_(1) = (mag_norm_b_(2) * half_mag_norm_u_hat_ - mag_norm_b_(0) * half_mag_norm_w_hat_);
    half_em_b_(2) = (mag_norm_b_(0) * half_mag_norm_v_hat_ - mag_norm_b_(1) * half_mag_norm_u_hat_);
  }

  // Normalized accelerometer measurement
  double aa_u_ = accData(0)*accData(0),
                 aa_v_ = accData(1)*accData(1),
                 aa_w_ = gg_ - aa_u_ - aa_v_;
  if (aa_w_ < 0.0) {aa_w_ = 0.0;}
  // double acc_w_ = (load_factor_ > 0.0) ? -sqrt(aa_w_) : sqrt(aa_w_);
  double acc_w_ = -sqrt(aa_w_);
  double accNorm = sqrt(aa_u_ + aa_v_ + aa_w_);
  if (!accNorm == 0.0)
  {
    // HALF of the gravity acceleration in {B}-frame implied with estimated orientation
    double half_g_norm_u_hat_, half_g_norm_v_hat_, half_g_norm_w_hat_;
    acc_norm_b_(0) = accData(0)/accNorm;
    acc_norm_b_(1) = accData(1)/accNorm;
    acc_norm_b_(2) = acc_w_/accNorm;

    // Half of estimated direction of gravity
    // Notice that the "-" is necessary! because the gravity is downwards and our coordinate is East North Up
    half_g_norm_u_hat_ = q0q2 - q1q3;                                // - (q1q3 - q0q2)
    half_g_norm_v_hat_ = - q0q1 - q2q3;                              // - (q0q1 + q2q3)
    half_g_norm_w_hat_ = 0.5 - q0q0 - q3q3;                      // - (q0q0 + q3q3 - 0.5)

    // Error is sum of cross product between estimated direction and measured direction of field vectors
    // /*
    half_ea_b_(0) = acc_norm_b_(1) * half_g_norm_w_hat_ - acc_norm_b_(2) * half_g_norm_v_hat_;
    half_ea_b_(1) = acc_norm_b_(2) * half_g_norm_u_hat_ - acc_norm_b_(0) * half_g_norm_w_hat_;
    half_ea_b_(2) = acc_norm_b_(0)* half_g_norm_v_hat_ - acc_norm_b_(1) * half_g_norm_u_hat_;
    // */
  }

  //std::cout<<"2kp2:"<<twoKp2_<<"2ki2:"<<twoKi2_<<std::endl;
  gyrBias_ += half_ea_b_*twoKi2_ + half_em_b_*twoKi1_ ;

  gyrCor_ = gyrData + gyrBias_  + half_ea_b_*twoKp2_ + half_em_b_*twoKp1_;

  // Time derivative of quaternion. q_dot = 0.5*q\otimes omega.
  //! q_k = q_{k-1} + dt*\dot{q}
  //! \dot{q} = 0.5*q \otimes P(\omega)
  dq0 = 0.5*(-q1 * gyrCor_(0) - q2 * gyrCor_(1)  - q3 * gyrCor_(2));
  dq1 = 0.5*(q0 * gyrCor_(0) + q2 * gyrCor_(2) - q3 * gyrCor_(1));
  dq2 = 0.5*(q0 * gyrCor_(1) - q1 * gyrCor_(2) + q3 * gyrCor_(0));
  dq3 = 0.5*(q0 * gyrCor_(2) + q1 *gyrCor_(1) - q2 * gyrCor_(0));

  q0 += dt*dq0;
  q1 += dt*dq1;
  q2 += dt*dq2;
  q3 += dt*dq3;

  // Normalise quaternion
  q_norm = 1.0/sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 *= q_norm;
  q1 *= q_norm;
  q2 *= q_norm;
  q3 *= q_norm;

  // Auxiliary variables to avoid repeated arithmetic
  q0q0 = q0 * q0;
  q0q1 = q0 * q1;
  q0q2 = q0 * q2;
  q0q3 = q0 * q3;
  q1q1 = q1 * q1;
  q1q2 = q1 * q2;
  q1q3 = q1 * q3;
  q2q2 = q2 * q2;
  q2q3 = q2 * q3;
  q3q3 = q3 * q3;
  load_factor_ = 1. / (q0q0 - q1q1 - q2q2 + q3q3);

  filteredData.w() = q0;
  filteredData.x() = q1;
  filteredData.y() = q2;
  filteredData.z() = q3;
}

void MahonyCompFilter::reset()
{
  gyrBias_ << 0.0, 0.0, 0.0;
}

}
