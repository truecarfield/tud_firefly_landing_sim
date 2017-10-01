#include "tud_firefly_state_estimation/firefly_ekf.h"
#include "tud_firefly_state_estimation/firefly_filter_common.h"
#include <mav_msgs/TorqueThrust.h>

namespace RobotLocalization
{
  FireflyEkf::FireflyEkf(std::vector<double>) :
    FireflyFilterBase()  // Must initialize filter base!
  {
  }

  FireflyEkf::~FireflyEkf()
  {
  }

  void FireflyEkf::correct(const Measurement &measurement)
  {
    FB_DEBUG("---------------------- FireflyEkf::correct ----------------------\n" <<
             "State is:\n" << state_ << "\n"
             "Topic is:\n" << measurement.topicName_ << "\n"
             "measurement is:\n" << measurement.measurement_ << "\n"
             "measurement topic name is:\n" << measurement.topicName_ << "\n\n"
             "measurement covariance is:\n" << measurement.covariance_ << "\n");

    // We don't want to update everything, so we need to build matrices that only update
    // the measured parts of our state vector. Throughout prediction and correction, we
    // attempt to maximize efficiency in Eigen.
    Measurement measurementPseudo;
    measurementPseudo = measurement;
    // std::cout<<"measurement Vx: "<<measurement.measurement_(6)<<", Vy: "<<measurement.measurement_(7)<<", Vz: "<<measurement.measurement_(8)<<std::endl;

    if (measurementPseudo.topicName_=="_acceleration" && !xyOdomAvailable_)
    {
      //       // Pseudo linear twist
      double roll = state_(StateMemberRoll);
      double pitch = state_(StateMemberPitch);
      double yaw = state_(StateMemberYaw);
      double rollVel = state_(StateMemberVroll);
      double pitchVel = state_(StateMemberVpitch);
      double yawVel = state_(StateMemberVyaw);
      double uPseudo = xyOdomPseudo_(3);
      double vPseudo = xyOdomPseudo_(4);
      double wPseudo = xyOdomPseudo_(5);

      double sp = ::sin(pitch);
      double cp = ::cos(pitch);
      double tp = ::tan(pitch);

      double sr = ::sin(roll);
      double cr = ::cos(roll);

      double sy = ::sin(yaw);
      double cy = ::cos(yaw);

      if (estimatingXY_)
      {
        /* xyOdomPseudo_(3)  += (controlAcceleration_(ControlMemberAu) + gravity_*sp)*delta_;
        xyOdomPseudo_(4) += (controlAcceleration_(ControlMemberAv) - gravity_*cp*sr)*delta_;
        xyOdomPseudo_(5)  += (controlAcceleration_(ControlMemberAw)  - gravity_*cp*cr)*delta_; // */

        // it turns out that the drift and bias has to been identified so that the blind mode can be implemented
        xyOdomPseudo_(3) += (yawVel*vPseudo - pitchVel*wPseudo + measurementPseudo.measurement_(12))*delta_;
        xyOdomPseudo_(4) += (rollVel*wPseudo - yawVel*uPseudo + measurementPseudo.measurement_(13))*delta_;
        xyOdomPseudo_(5) += (pitchVel*uPseudo - rollVel*vPseudo + measurementPseudo.measurement_(14))*delta_; // */
        xyOdomPseudo_(0) += (cy*cp*uPseudo + (cy*sp*sr - sy*cr)*vPseudo + (cy*sp*cr + sy*sr)*wPseudo)*delta_;
        xyOdomPseudo_(1) += (sy*cp*uPseudo + (sy*sp*sr + cy*cr)*vPseudo + (sy*sp*cr - cy*sr)*wPseudo)*delta_;
        xyOdomPseudo_(2) += (-sp*uPseudo + cp*sr*vPseudo + cp*cr*wPseudo)*delta_;
      }
      else
      {
        xyOdomPseudo_(0) = 0.0;
        xyOdomPseudo_(1) = 0.0;
        xyOdomPseudo_(3) = 0.0;
        xyOdomPseudo_(4) = 0.0;
        xyOdomPseudo_(5) = 0.0;
      }
      // std::cout<<"blind Au:"<<measurementPseudo.measurement_(12)<<", Av:"<<measurementPseudo.measurement_(13)<<", Aw:"<<measurementPseudo.measurement_(14)<<std::endl;
       //std::cout<<"blind Vx:"<<xyOdomPseudo_(3)<<", Vy:"<<xyOdomPseudo_(4)<<", Vz:"<<xyOdomPseudo_(5)<<std::endl;

      measurementPseudo.measurement_(0) = xyOdomPseudo_(0);
      measurementPseudo.measurement_(1) = xyOdomPseudo_(1);
      measurementPseudo.measurement_(2) = xyOdomPseudo_(2);
      measurementPseudo.updateVector_[0] = 1;
      measurementPseudo.updateVector_[1] = 1;
      measurementPseudo.updateVector_[2] = 1;
    }

    // First, determine how many state vector values we're updating
    std::vector<size_t> updateIndices;
    for (size_t i = 0; i < measurementPseudo.updateVector_.size(); ++i)
    {
      if (measurementPseudo.updateVector_[i])
      {
        // Handle nan and inf values in measurementPseudos
        if (std::isnan(measurementPseudo.measurement_(i)))
        {
          FB_DEBUG("Value at index " << i << " was nan. Excluding from update.\n");
        }
        else if (std::isinf(measurementPseudo.measurement_(i)))
        {
          FB_DEBUG("Value at index " << i << " was inf. Excluding from update.\n");
        }
        else
        {
          updateIndices.push_back(i);
        }
      }
    }

    FB_DEBUG("Update indices are:\n" << updateIndices << "\n");

    size_t updateSize = updateIndices.size();

    // Now set up the relevant matrices
    Eigen::VectorXd stateSubset(updateSize);                              // x (in most literature)
    Eigen::VectorXd measurementSubset(updateSize);                        // z
    Eigen::MatrixXd measurementCovarianceSubset(updateSize, updateSize);  // R
    Eigen::MatrixXd stateTomeasurementSubset(updateSize, state_.rows());  // H
    Eigen::MatrixXd kalmanGainSubset(state_.rows(), updateSize);          // K
    Eigen::VectorXd innovationSubset(updateSize);                         // z - Hx

    stateSubset.setZero();
    measurementSubset.setZero();
    measurementCovarianceSubset.setZero();
    stateTomeasurementSubset.setZero();
    kalmanGainSubset.setZero();
    innovationSubset.setZero();

    // Now build the sub-matrices from the full-sized matrices
    for (size_t i = 0; i < updateSize; ++i)
    {
      measurementSubset(i) = measurementPseudo.measurement_(updateIndices[i]);
      stateSubset(i) = state_(updateIndices[i]);

      for (size_t j = 0; j < updateSize; ++j)
      {
        // measurementCovarianceSubset(i, j) = measurementPseudo.covariance_(updateIndices[i], updateIndices[j]);
        measurementCovarianceSubset(i, j) = (useInitMeasNoiseCovariance_) ?
              measurementNoiseCovariance_(updateIndices[i], updateIndices[j]) : measurementPseudo.covariance_(updateIndices[i], updateIndices[j]);
      }

      // Handle negative (read: bad) covariances in the measurementPseudo. Rather
      // than exclude the measurementPseudo or make up a covariance, just take
      // the absolute value.
      if (measurementCovarianceSubset(i, i) < 0)
      {
        FB_DEBUG("WARNING: Negative covariance for index " << i <<
                 " of measurementPseudo (value is" << measurementCovarianceSubset(i, i) <<
                 "). Using absolute value...\n");

        measurementCovarianceSubset(i, i) = ::fabs(measurementCovarianceSubset(i, i));
      }

      // If the measurementPseudo variance for a given variable is very
      // near 0 (as in e-50 or so) and the variance for that
      // variable in the covariance matrix is also near zero, then
      // the Kalman gain computation will blow up. Really, no
      // measurementPseudo can be completely without error, so add a small
      // amount in that case.
      if (measurementCovarianceSubset(i, i) < 1e-9)
      {
        FB_DEBUG("WARNING: measurementPseudo had very small error covariance for index " << updateIndices[i] <<
                 ". Adding some noise to maintain filter stability.\n");

        measurementCovarianceSubset(i, i) = 1e-9;
      }
    }

    // The state-to-measurementPseudo function, h, will now be a measurement_size x full_state_size
    // matrix, with ones in the (i, i) locations of the values to be updated
    for (size_t i = 0; i < updateSize; ++i)
    {
      stateTomeasurementSubset(i, updateIndices[i]) = 1;
    }

    FB_DEBUG("Current state subset is:\n" << stateSubset <<
             "\nmeasurementPseudo subset is:\n" << measurementSubset <<
             "\nmeasurementPseudo covariance subset is:\n" << measurementCovarianceSubset <<
             "\nState-to-measurementPseudo subset is:\n" << stateTomeasurementSubset << "\n");

    // (1) Compute the Kalman gain: K = (PH') / (HPH' + R)
    Eigen::MatrixXd pht = estimateErrorCovariance_ * stateTomeasurementSubset.transpose();
    Eigen::MatrixXd hphrInv  = (stateTomeasurementSubset * pht + measurementCovarianceSubset).inverse();
    kalmanGainSubset.noalias() = pht * hphrInv;

    innovationSubset = (measurementSubset - stateSubset);

    // Wrap angles in the innovation
    for (size_t i = 0; i < updateSize; ++i)
    {
      if (updateIndices[i] == StateMemberRoll  ||
          updateIndices[i] == StateMemberPitch ||
          updateIndices[i] == StateMemberYaw)
      {
        while (innovationSubset(i) < -PI)
        {
          innovationSubset(i) += TAU;
        }

        while (innovationSubset(i) > PI)
        {
          innovationSubset(i) -= TAU;
        }
      }
    }

    // (2) Check Mahalanobis distance between mapped measurementPseudo and state.
    // if (checkMahalanobisThreshold(innovationSubset, hphrInv, measurementPseudo.mahalanobisThresh_))
    {
      // (3) Apply the gain to the difference between the state and measurementPseudo: x = x + K(z - Hx)
       /*
      if (measurementPseudo.topicName_=="imu0_pose")
      {
        std::cout<<"dim is:"<<measurementSubset.size()<<std::endl
                         <<"roll_meas: "<<measurementSubset(0)
                         <<", pitch_meas: "<<measurementSubset(1)
                         <<", yaw_meas: "<<measurementSubset(2)<<std::endl;
      } // */

      state_.noalias() += kalmanGainSubset * innovationSubset;
      /*
      if (measurementPseudo.topicName_=="imu0_pose")
      {
        std::cout<<"roll_state: "<<state_(3)
                         <<", pitch_state: "<<state_(4)
                         <<", yaw_state: "<<state_(5)<<std::endl;
      } // */

      // (4) Update the estimate error covariance using the Joseph form: (I - KH)P(I - KH)' + KRK'
      Eigen::MatrixXd gainResidual = identity_;
      gainResidual.noalias() -= kalmanGainSubset * stateTomeasurementSubset;
      estimateErrorCovariance_ = gainResidual * estimateErrorCovariance_ * gainResidual.transpose();
      estimateErrorCovariance_.noalias() += kalmanGainSubset *
                                            measurementCovarianceSubset *
                                            kalmanGainSubset.transpose();

      // std::cout<<"measurement topicName is "<<measurement.topicName_<<" Seq "<<measurementSeq_<<"correct roll "<<state_(3)<<", pitch: "<<state_(4)<<", yaw: "<<state_(5)<<std::endl;
      measurementSeq_++;
      // Handle wrapping of angles
      wrapStateAngles();

      FB_DEBUG("Kalman gain subset is:\n" << kalmanGainSubset <<
               "\nInnovation is:\n" << innovationSubset <<
               "\nCorrected full state is:\n" << state_ <<
               "\nCorrected full estimate error covariance is:\n" << estimateErrorCovariance_ <<
               "\n\n---------------------- /FireflyEkf::correct ----------------------\n");
    }
  }

  void FireflyEkf::predict(const double referenceTime, const double delta)
  {
    FB_DEBUG("---------------------- FireflyEkf::predict ----------------------\n" <<
             "delta is " << delta << "\n" <<
             "state is " << state_ << "\n");

    double roll = state_(StateMemberRoll);
    double pitch = state_(StateMemberPitch);
    double yaw = state_(StateMemberYaw);
    double xVel = state_(StateMemberVx);
    double yVel = state_(StateMemberVy);
    double zVel = state_(StateMemberVz);
    double rollVel = state_(StateMemberVroll);
    double pitchVel = state_(StateMemberVpitch);
    double yawVel = state_(StateMemberVyaw);
    double xAcc = state_(StateMemberAx);
    double yAcc = state_(StateMemberAy);
    double zAcc = state_(StateMemberAz);

    // We'll need these trig calculations a lot.
    double sp = ::sin(pitch);
    double cp = ::cos(pitch);
    double tp = ::tan(pitch);

    double sr = ::sin(roll);
    double cr = ::cos(roll);

    double sy = ::sin(yaw);
    double cy = ::cos(yaw);

    PrepareControl(referenceTime, delta);

    /* Transferfunction = [ G_position    0
                                                  0                      G_attitude]
    */
    // Prepare the transfer function of position state estimation
    transferFunction_(StateMemberX, StateMemberVx) = cy * cp * delta;
    transferFunction_(StateMemberX, StateMemberVy) = (cy * sp * sr - sy * cr) * delta;
    transferFunction_(StateMemberX, StateMemberVz) = (cy * sp * cr + sy * sr) * delta;
    transferFunction_(StateMemberY, StateMemberVx) = sy * cp * delta;
    transferFunction_(StateMemberY, StateMemberVy) = (sy * sp * sr + cy * cr) * delta;
    transferFunction_(StateMemberY, StateMemberVz) = (sy * sp * cr - cy * sr) * delta;
    transferFunction_(StateMemberZ, StateMemberVx) = -sp * delta;
    transferFunction_(StateMemberZ, StateMemberVy) = cp * sr * delta;
    transferFunction_(StateMemberZ, StateMemberVz) = cp * cr * delta;
    //transferFunction_(StateMemberVx, StateMemberVx) = 1.0;
    transferFunction_(StateMemberVx, StateMemberVy) = yawVel * delta;
    transferFunction_(StateMemberVx, StateMemberVz) = - pitchVel * delta;
    transferFunction_(StateMemberVy, StateMemberVx) = - yawVel * delta;
    //transferFunction_(StateMemberVy, StateMemberVy) = 1.0;
    transferFunction_(StateMemberVy, StateMemberVz) = rollVel * delta;
    transferFunction_(StateMemberVz, StateMemberVx) = pitchVel * delta;
    transferFunction_(StateMemberVz, StateMemberVy) = - rollVel * delta;
    //transferFunction_(StateMemberVz, StateMemberVz) = 1.0;
    transferFunction_(StateMemberVx, StateMemberAx) = delta;
    transferFunction_(StateMemberVy, StateMemberAy) = delta;
    transferFunction_(StateMemberVz, StateMemberAz) = delta;  // */

    // Prepare the transfer function of attitude state estimation  */
    //transferFunction_(StateMemberVroll, StateMemberVroll) = 1.0;
    //transferFunction_(StateMemberVroll, StateMemberVpitch)
    //     = (fireflyParameters_.I_v - fireflyParameters_.I_w) * yawVel/fireflyParameters_.I_u;
    //transferFunction_(StateMemberVroll, StateMemberVyaw)
    //    = (fireflyParameters_.I_v - fireflyParameters_.I_w) * pitchVel/fireflyParameters_.I_u;
    //transferFunction_(StateMemberVpitch, StateMemberVroll)
    //    = (fireflyParameters_.I_w - fireflyParameters_.I_u) * yawVel/fireflyParameters_.I_v;
    //transferFunctionJacobian_(StateMemberVpitch, StateMemberVpitch) = 1.0;
    //transferFunction_(StateMemberVpitch, StateMemberVyaw)
    //    = (fireflyParameters_.I_w - fireflyParameters_.I_u) * rollVel/fireflyParameters_.I_v;
    //transferFunction_(StateMemberVyaw, StateMemberVroll)
    //    = (fireflyParameters_.I_u - fireflyParameters_.I_v) * pitchVel/fireflyParameters_.I_w;
    // transferFunction_(StateMemberVyaw, StateMemberVpitch)
    //   = (fireflyParameters_.I_u - fireflyParameters_.I_v) * rollVel/fireflyParameters_.I_w;
    //transferFunctionJacobian_(StateMemberVyaw, StateMemberVyaw) = 1.0; */

    transferFunction_(StateMemberRoll, StateMemberVroll) = delta;
    transferFunction_(StateMemberRoll, StateMemberVpitch) = tp * sr * delta;
    transferFunction_(StateMemberRoll, StateMemberVyaw) = tp * cr * delta;
    //transferFunction_(StateMemberPitch, StateMemberVroll) = 0.0;
    transferFunction_(StateMemberPitch, StateMemberVpitch) = cr * delta;
    transferFunction_(StateMemberPitch, StateMemberVyaw) = - sr * delta;
    //transferFunction_(StateMemberYaw, StateMemberVroll) = 0.0;
    transferFunction_(StateMemberYaw, StateMemberVpitch) = sr/cp * delta;
    transferFunction_(StateMemberYaw, StateMemberVyaw) = cr/cp * delta;

     // Much of the transfer function Jacobian is identical to the transfer function
    transferFunctionJacobian_ = transferFunction_;
    /*transferFunctionJacobian_(StateMemberRoll, StateMemberVroll) = 0.0;
    transferFunctionJacobian_(StateMemberRoll, StateMemberVpitch) = 0.0;
    transferFunctionJacobian_(StateMemberRoll, StateMemberVyaw) = 0.0;
    //transferFunction_(StateMemberPitch, StateMemberVroll) = 0.0;
    transferFunctionJacobian_(StateMemberPitch, StateMemberVpitch) =  0.0;
    transferFunctionJacobian_(StateMemberPitch, StateMemberVyaw) = 0.0;
    //transferFunction_(StateMemberYaw, StateMemberVroll) = 0.0;
    transferFunctionJacobian_(StateMemberYaw, StateMemberVpitch) = 0.0;
    transferFunctionJacobian_(StateMemberYaw, StateMemberVyaw) =  0.0; // */

    transferFunctionJacobian_(StateMemberRoll, StateMemberRoll) = 1.0 + (pitchVel * cr - yawVel * sr) * tp * delta;
    transferFunctionJacobian_(StateMemberRoll, StateMemberPitch) = ((pitchVel * sr + yawVel * cr)/(cp * cp)) * delta;
    //transferFunction_(StateMemberRoll, StateMemberYaw) = 0.0;
    transferFunctionJacobian_(StateMemberPitch, StateMemberRoll) = (- pitchVel * sr - yawVel * cr) * delta;
    //transferFunction_(StateMemberPitch, StateMemberPitch) = 1.0;
    //transferFunction_(StateMemberPitch, StateMemberYaw) = 0.0;
    transferFunctionJacobian_(StateMemberYaw, StateMemberRoll) = ((pitchVel * cr - yawVel * sr)/cp) * delta;
    transferFunctionJacobian_(StateMemberYaw, StateMemberPitch) = ((pitchVel * sr + yawVel * cr)* tp/cp) * delta; // */

    /*
    transferFunctionJacobian_(StateMemberX, StateMemberRoll) = 0.0;
    transferFunctionJacobian_(StateMemberX, StateMemberPitch) = 0.0;
    transferFunctionJacobian_(StateMemberX, StateMemberYaw) = 0.0;
    transferFunctionJacobian_(StateMemberY, StateMemberRoll) = 0.0;
    transferFunctionJacobian_(StateMemberY, StateMemberPitch) = 0.0;
    transferFunctionJacobian_(StateMemberY, StateMemberYaw) = 0.0;
    transferFunctionJacobian_(StateMemberZ, StateMemberRoll) = 0.0;
    transferFunctionJacobian_(StateMemberZ, StateMemberPitch) = 0.0;
     transferFunctionJacobian_(StateMemberZ, StateMemberYaw) = 0.0;
     */


   /*  Original Transferfunction in ekf.cpp
    // Prepare the transfer function
    transferFunction_(StateMemberX, StateMemberVx) = cy * cp * delta;
    transferFunction_(StateMemberX, StateMemberVy) = (cy * sp * sr - sy * cr) * delta;
    transferFunction_(StateMemberX, StateMemberVz) = (cy * sp * cr + sy * sr) * delta;
    transferFunction_(StateMemberX, StateMemberAx) = 0.5 * transferFunction_(StateMemberX, StateMemberVx) * delta;
    transferFunction_(StateMemberX, StateMemberAy) = 0.5 * transferFunction_(StateMemberX, StateMemberVy) * delta;
    transferFunction_(StateMemberX, StateMemberAz) = 0.5 * transferFunction_(StateMemberX, StateMemberVz) * delta;
    transferFunction_(StateMemberY, StateMemberVx) = sy * cp * delta;
    transferFunction_(StateMemberY, StateMemberVy) = (sy * sp * sr + cy * cr) * delta;
    transferFunction_(StateMemberY, StateMemberVz) = (sy * sp * cr - cy * sr) * delta;
    transferFunction_(StateMemberY, StateMemberAx) = 0.5 * transferFunction_(StateMemberY, StateMemberVx) * delta;
    transferFunction_(StateMemberY, StateMemberAy) = 0.5 * transferFunction_(StateMemberY, StateMemberVy) * delta;
    transferFunction_(StateMemberY, StateMemberAz) = 0.5 * transferFunction_(StateMemberY, StateMemberVz) * delta;
    transferFunction_(StateMemberZ, StateMemberVx) = -sp * delta;
    transferFunction_(StateMemberZ, StateMemberVy) = cp * sr * delta;
    transferFunction_(StateMemberZ, StateMemberVz) = cp * cr * delta;
    transferFunction_(StateMemberZ, StateMemberAx) = 0.5 * transferFunction_(StateMemberZ, StateMemberVx) * delta;
    transferFunction_(StateMemberZ, StateMemberAy) = 0.5 * transferFunction_(StateMemberZ, StateMemberVy) * delta;
    transferFunction_(StateMemberZ, StateMemberAz) = 0.5 * transferFunction_(StateMemberZ, StateMemberVz) * delta;
    transferFunction_(StateMemberRoll, StateMemberVroll) = transferFunction_(StateMemberX, StateMemberVx);
    transferFunction_(StateMemberRoll, StateMemberVpitch) = transferFunction_(StateMemberX, StateMemberVy);
    transferFunction_(StateMemberRoll, StateMemberVyaw) = transferFunction_(StateMemberX, StateMemberVz);
    transferFunction_(StateMemberPitch, StateMemberVroll) = transferFunction_(StateMemberY, StateMemberVx);
    transferFunction_(StateMemberPitch, StateMemberVpitch) = transferFunction_(StateMemberY, StateMemberVy);
    transferFunction_(StateMemberPitch, StateMemberVyaw) = transferFunction_(StateMemberY, StateMemberVz);
    transferFunction_(StateMemberYaw, StateMemberVroll) = transferFunction_(StateMemberZ, StateMemberVx);
    transferFunction_(StateMemberYaw, StateMemberVpitch) = transferFunction_(StateMemberZ, StateMemberVy);
    transferFunction_(StateMemberYaw, StateMemberVyaw) = transferFunction_(StateMemberZ, StateMemberVz);
    transferFunction_(StateMemberVx, StateMemberAx) = delta;
    transferFunction_(StateMemberVy, StateMemberAy) = delta;
    transferFunction_(StateMemberVz, StateMemberAz) = delta;


    // Prepare the transfer function Jacobian. This function is analytically derived from the
    // transfer function.
    double xCoeff = 0.0;
    double yCoeff = 0.0;
    double zCoeff = 0.0;
    double oneHalfATSquared = 0.5 * delta * delta;

    yCoeff = cy * sp * cr + sy * sr;
    zCoeff = -cy * sp * sr + sy * cr;
    double dFx_dR = (yCoeff * yVel + zCoeff * zVel) * delta +
                    (yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;
    double dFR_dR = 1 + (yCoeff * pitchVel + zCoeff * yawVel) * delta;

    xCoeff = -cy * sp;
    yCoeff = cy * cp * sr;
    zCoeff = cy * cp * cr;
    double dFx_dP = (xCoeff * xVel + yCoeff * yVel + zCoeff * zVel) * delta +
                    (xCoeff * xAcc + yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;
    double dFR_dP = (xCoeff * rollVel + yCoeff * pitchVel + zCoeff * yawVel) * delta;

    xCoeff = -sy * cp;
    yCoeff = -sy * sp * sr - cy * cr;
    zCoeff = -sy * sp * cr + cy * sr;
    double dFx_dY = (xCoeff * xVel + yCoeff * yVel + zCoeff * zVel) * delta +
                    (xCoeff * xAcc + yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;
    double dFR_dY = (xCoeff * rollVel + yCoeff * pitchVel + zCoeff * yawVel) * delta;

    yCoeff = sy * sp * cr - cy * sr;
    zCoeff = -sy * sp * sr - cy * cr;
    double dFy_dR = (yCoeff * yVel + zCoeff * zVel) * delta +
                    (yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;
    double dFP_dR = (yCoeff * pitchVel + zCoeff * yawVel) * delta;

    xCoeff = -sy * sp;
    yCoeff = sy * cp * sr;
    zCoeff = sy * cp * cr;
    double dFy_dP = (xCoeff * xVel + yCoeff * yVel + zCoeff * zVel) * delta +
                    (xCoeff * xAcc + yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;
    double dFP_dP = 1 + (xCoeff * rollVel + yCoeff * pitchVel + zCoeff * yawVel) * delta;

    xCoeff = cy * cp;
    yCoeff = cy * sp * sr - sy * cr;
    zCoeff = cy * sp * cr + sy * sr;
    double dFy_dY = (xCoeff * xVel + yCoeff * yVel + zCoeff * zVel) * delta +
                    (xCoeff * xAcc + yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;
    double dFP_dY = (xCoeff * rollVel + yCoeff * pitchVel + zCoeff * yawVel) * delta;

    yCoeff = cp * cr;
    zCoeff = -cp * sr;
    double dFz_dR = (yCoeff * yVel + zCoeff * zVel) * delta +
                    (yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;
    double dFY_dR = (yCoeff * pitchVel + zCoeff * yawVel) * delta;

    xCoeff = -cp;
    yCoeff = -sp * sr;
    zCoeff = -sp * cr;
    double dFz_dP = (xCoeff * xVel + yCoeff * yVel + zCoeff * zVel) * delta +
                    (xCoeff * xAcc + yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;
    double dFY_dP = (xCoeff * rollVel + yCoeff * pitchVel + zCoeff * yawVel) * delta;


    // Much of the transfer function Jacobian is identical to the transfer function
    transferFunctionJacobian_ = transferFunction_;
    transferFunctionJacobian_(StateMemberX, StateMemberRoll) = dFx_dR;
    transferFunctionJacobian_(StateMemberX, StateMemberPitch) = dFx_dP;
    transferFunctionJacobian_(StateMemberX, StateMemberYaw) = dFx_dY;
    transferFunctionJacobian_(StateMemberY, StateMemberRoll) = dFy_dR;
    transferFunctionJacobian_(StateMemberY, StateMemberPitch) = dFy_dP;
    transferFunctionJacobian_(StateMemberY, StateMemberYaw) = dFy_dY;
    transferFunctionJacobian_(StateMemberZ, StateMemberRoll) = dFz_dR;
    transferFunctionJacobian_(StateMemberZ, StateMemberPitch) = dFz_dP;
    transferFunctionJacobian_(StateMemberRoll, StateMemberRoll) = dFR_dR;
    transferFunctionJacobian_(StateMemberRoll, StateMemberPitch) = dFR_dP;
    transferFunctionJacobian_(StateMemberRoll, StateMemberYaw) = dFR_dY;
    transferFunctionJacobian_(StateMemberPitch, StateMemberRoll) = dFP_dR;
    transferFunctionJacobian_(StateMemberPitch, StateMemberPitch) = dFP_dP;
    transferFunctionJacobian_(StateMemberPitch, StateMemberYaw) = dFP_dY;
    transferFunctionJacobian_(StateMemberYaw, StateMemberRoll) = dFY_dR;
    transferFunctionJacobian_(StateMemberYaw, StateMemberPitch) = dFY_dP;
     // */

    FB_DEBUG("Transfer function is:\n" << transferFunction_ <<
             "\nTransfer function Jacobian is:\n" << transferFunctionJacobian_ <<
             "\nProcess noise covariance is:\n" << processNoiseCovariance_ <<
             "\nCurrent state is:\n" << state_ << "\n");

    Eigen::MatrixXd *processNoiseCovariance = &processNoiseCovariance_;

    if (useDynamicProcessNoiseCovariance_)
    {
      computeDynamicProcessNoiseCovariance(state_, delta);
      processNoiseCovariance = &dynamicProcessNoiseCovariance_;
    }

    // (1) Apply control terms, which are actually accelerations
    // /*
    //if (estimatingXY_)
    //{
     // controlAcceleration without gravity acceleration is always zero, so this additional part is necessary
    if (motorStatus_)
    {
      state_(StateMemberAx) = controlAcceleration_(ControlMemberAu) + gravity_*sp;
      state_(StateMemberAy) = controlAcceleration_(ControlMemberAv) - gravity_*cp*sr;
      state_(StateMemberAz) = controlAcceleration_(ControlMemberAw) - gravity_*cp*cr; // */
    }
    state_(StateMemberVroll) += controlAcceleration_(ControlMemberAp)*delta;
    state_(StateMemberVpitch) += controlAcceleration_(ControlMemberAq)*delta;
    state_(StateMemberVyaw) += controlAcceleration_(ControlMemberAr)*delta;
   /* std::cout<<"p is "<<state_(StateMemberVroll) <<
                         ", q is "<<state_(StateMemberVpitch) <<
                         ", r is "<<state_(StateMemberVyaw) <<std::endl; */

    // (2) Project the state forward: x = Ax + Bu (really, x = f(x, u))
    // because transferFunction_ is initiated with Identity so there will be no problem with integration.  by ZD
    state_ = transferFunction_ * state_;

    // Handle wrapping
    wrapStateAngles();

    FB_DEBUG("Predicted state is:\n" << state_ <<
             "\nCurrent estimate error covariance is:\n" <<  estimateErrorCovariance_ << "\n");

    // (3) Project the error forward: P = J * P * J' + Q
    estimateErrorCovariance_ = (transferFunctionJacobian_ *
                                estimateErrorCovariance_ *
                                transferFunctionJacobian_.transpose());
    estimateErrorCovariance_.noalias() += delta * (*processNoiseCovariance);

    FB_DEBUG("Predicted estimate error covariance is:\n" << estimateErrorCovariance_ <<
             "\n\n--------------------- /FireflyEkf::predict ----------------------\n");

    delta_= delta;
  }

  void FireflyEkf::PrepareControl(const double referenceTime, const double predictionDelta)
  {
    controlAcceleration_.setZero();
    controlAcceleration_ = latestControl_;
  }

}  // namespace RobotLocalization
