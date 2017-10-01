#ifndef MAHONY_COMPLEMENTARY_FILTER_H
#define MAHONY_COMPLEMENTARY_FILTER_H

#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <tud_firefly_control/tud_firefly_common.h>
#include <tud_firefly_control/differential_tracker.h>

namespace RobotLocalization{

class  MahonyCompFilter {
public:
  MahonyCompFilter();
  ~MahonyCompFilter();

  void reset();

  //! @brief Main function of the filter
  //!
  //! @param[in] accData - Accelerometer measurement
  //! @param[in] gyrData - Gyroscope measurement
  //! @param[in] magData - Magnetometer measurement
  //! @param[in] dt - Time from last update to this one
  //! @param[out] filteredData - Filtered UAV orientation expressed in quaternion
  //!
  void update(const Eigen::Vector3d& accData,
                           const Eigen::Vector3d& gyrData,
                           const Eigen::Vector3d& magData,
                           const double& dt,
                           Eigen::Quaterniond& filteredData);

  //! @brief Initialize the parameters
  //!
  //! @param[in] k - The four coefficients of the filter Kp1, Ki1, Kp2, Ki2
  //! @param[in] tauGyr - Time constant of gyroscope differential tracker
  //! @param[in] tauAccel - Time constant of accelerometer differential tracker
  //! @param[in] tauMag - Time constant of magnetometer differential tracker
  //! @param[in] tauGyr - Configuration of gyroscope differential tracker
  //! @param[in] tauAccel - Configuration of accelerometer differential tracker
  //! @param[in] tauMag - Configuration of magnetometer differential tracker
  //!
  void initParams(const Eigen::Vector4d& k,
                                  const Eigen::Vector3d& tauGyr,
                                  const Eigen::Vector3d& tauAccel,
                                  const Eigen::Vector3d& tauMag,
                                  const std::vector<int>& tauGyrConfig,
                                  const std::vector<int>& tauAccelConfig,
                                  const std::vector<int>& tauMagConfig);

  //! @brief Initialize the filtered state
  //!
  //! @param[in] Au - Linear acceleration in U axis;
  //! @param[in] Av - Linear acceleration in V axis;
  //! @param[in] Aw - Linear acceleration in W axis;
  //! @param[in] Mu - Magnetic field in U axis;
  //! @param[in] Mv - Magnetic field in V axis;
  //! @param[in] Mw - Magnetic field in W axis;
  void initState(double Au, double Av, double Aw, double Mu, double Mv, double Mw);

  // Function for test
  void sendTest(Eigen::Vector3d& g_b_);

protected:
  //! @brief Gravity const
  //!
  double gravity_;

  //! @brief gravity*gravity_
  //!
  double gg_;

  //! @brief 3x3 identity matrix
  //!
  Eigen::Matrix3d Identity_;

  //! @brief HALF of the orientation error for magnetometer measurement correction
  //!
  Eigen::Vector3d half_em_b_;

  //! @brief HALF of the orientation error for accelerometer measurement correction
  //!
  Eigen::Vector3d half_ea_b_;

  //! @brief The gyroscope measurement correction factor that intagrates Ki part of the correction
  //!
  Eigen::Vector3d gyrBias_;

  //! @brief Corrected gyroscope measurment
  //!
  Eigen::Vector3d gyrCor_;

  //! @brief PI term coefficients
  //!
  double twoKp1_, twoKi1_, twoKp2_, twoKi2_;

  //! @brief Indices whether the filter state is initiated
  //!
  bool state_initialized_;

  //! @brief Indices whether magnetometer measurement is being implemented for the filter
  //!
  bool enable_mag_;

  //! @brief Differential tracker for gyroscope data
  //!
  rotors_control::DifferentialTracker gyrTracker_;

  //! @brief Differential tracker for accelerometer data
  //!
  rotors_control::DifferentialTracker accelTracker_;

  //! @brief Differential tracker for magnetometer data
  //!
  rotors_control::DifferentialTracker magnTracker_;

  //! @brief Accelerometer data filtered by low pass
  //!
  Eigen::VectorXd acc_lp_;

  //! @brief Gyroscope data filtered by low pass
  //!
  Eigen::VectorXd gyr_lp_;

  //! @brief Magnetometer data filtered by low pass
  //!
  Eigen::VectorXd mag_lp_;

  //! @brief Magnetometer normalized data
  //!
  Eigen::Vector3d mag_norm_b_;

  //! @brief Accelerometer normalized data
  //!
  Eigen::Vector3d acc_norm_b_;

  //! @brief filtered orientation data, and its multiplicated values
  //!
  double q0, q1, q2, q3;

  //! @brief Norm of the filtered orientation data
  //!
  double q_norm;

  //! @brief Time derivative of the filtered orientation data derived from the filter
  //!
  double dq0, dq1, dq2, dq3;

  //! @brief Multiplications of each two quaternion element for saving calculation resources
  //!
  double q0q0, q0q1, q0q2, q0q3;
  double q1q1, q1q2, q1q3;
  double q2q2, q2q3;
  double q3q3;

  //! @brief Load factor of the UAV to indice whether it is fliped over or not
  //!
  double load_factor_;
};

}

#endif // MAHONY_COMPLEMENTARY_FILTER_H
