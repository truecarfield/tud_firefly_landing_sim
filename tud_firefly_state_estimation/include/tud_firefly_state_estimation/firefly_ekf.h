#ifndef FIREFLY_EKF_LANDING
#define FIREFLY_EKF_LANDING

#include <ros/ros.h>
#include <Eigen/Eigen>

#include <rotors_control/common.h>
#include <rotors_control/parameters.h>

#include "tud_firefly_state_estimation/firefly_filter_base.h"

#include <fstream>
#include <vector>
#include <set>
#include <queue>

namespace RobotLocalization
{

//! @brief Extended Kalman filter class
//!
//! Implementation of an extended Kalman filter (EKF). This
//! class derives from FilterBase and overrides the predict()
//! and correct() methods in keeping with the discrete time
//! EKF algorithm.
//!
class FireflyEkf: public FireflyFilterBase
{
  public:
    //! @brief Constructor for the Ekf class
    //!
    //! @param[in] args - Generic argument container (not used here, but
    //! needed so that the ROS filters can pass arbitrary arguments to
    //! templated filter types).
    //!
    explicit FireflyEkf(std::vector<double> args = std::vector<double>());

    //! @brief Destructor for the Ekf class
    //!
    ~FireflyEkf();

    //! @brief Carries out the correct step in the predict/update cycle.
    //!
    //! @param[in] measurement - The measurement to fuse with our estimate
    //!
    void correct(const Measurement &measurement);

    //! @brief Carries out the predict step in the predict/update cycle.
    //!
    //! Projects the state and error rmatrices forward using a model of
    //! the vehicle's motion.
    //!
    //! @param[in] referenceTime - The time at which the prediction is being made
    //! @param[in] delta - The time step over which to predict.
    //!
    void predict(const double referenceTime, const double delta);
    //!
    //! \brief Initiate the control signal with the control effort signal from the controller
    //! \param referenceTime - The time at which the prediction is being made
    //! \param predictionDelta - The amount of time over which we are carrying out our prediction
    //!
    void PrepareControl(const double referenceTime, const double predictionDelta);

    int measurementSeq_;

    std::deque<Eigen::VectorXd> actuator_commands_;
};

}  // namespace RobotLocalization

#endif
