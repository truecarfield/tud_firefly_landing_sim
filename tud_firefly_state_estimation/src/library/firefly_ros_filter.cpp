#include "tud_firefly_state_estimation/firefly_ros_filter.h"
#include "robot_localization/filter_utilities.h"

#include "robot_localization/ekf.h"
#include "robot_localization/ukf.h"
#include "tud_firefly_state_estimation/firefly_ekf.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <map>
#include <string>
#include <utility>
#include <vector>
#include <limits>

namespace RobotLocalization {

/////////////////////////////////////////////////////////////////////////////
/// Main executed functions
/////////////////////////////////////////////////////////////////////////////
template<typename T>
FireflyRosFilter<T>::FireflyRosFilter(std::vector<double> args) :
  // Original variables from RosFilter
  staticDiagErrorLevel_(diagnostic_msgs::DiagnosticStatus::OK),
  tfListener_(tfBuffer_),
  dynamicDiagErrorLevel_(diagnostic_msgs::DiagnosticStatus::OK),
  filter_(args),  // filter_ is a instance of the template T   by ZD
  frequency_(30.0),
  historyLength_(0),
  lastSetPoseTime_(0),
  latestControl_(),
  latestControlTime_(0),
  nhLocal_("~"),
  printDiagnostics_(true),
  publishTransform_(true),
  publishAcceleration_(false),
  twoDMode_(false),
  smoothLaggedData_(false),
  // Variables from FireflyRosFilter
  motorStatus_(false),
  xyOdomAvailable_(true),
  estimatingXY_(false),
  permittDelay_(true),
  //modeTopic("firefly/estimating_xy"),
  modeTopic_("firefly/mode"),
  motorTopic_("firefly/motor_status"),
  clockTopic_("clock"),
  actuatorTopic_("firefly/gazebo/command/motor_speed")
{
  stateVariableNames_.push_back("X");
  stateVariableNames_.push_back("Y");
  stateVariableNames_.push_back("Z");
  stateVariableNames_.push_back("ROLL");
  stateVariableNames_.push_back("PITCH");
  stateVariableNames_.push_back("YAW");
  stateVariableNames_.push_back("X_VELOCITY");
  stateVariableNames_.push_back("Y_VELOCITY");
  stateVariableNames_.push_back("Z_VELOCITY");
  stateVariableNames_.push_back("ROLL_VELOCITY");
  stateVariableNames_.push_back("PITCH_VELOCITY");
  stateVariableNames_.push_back("YAW_VELOCITY");
  stateVariableNames_.push_back("X_ACCELERATION");
  stateVariableNames_.push_back("Y_ACCELERATION");
  stateVariableNames_.push_back("Z_ACCELERATION");

  diagnosticUpdater_.setHardwareID("none");

  ref_rotor_velocities.resize(4);

  curMeasurement_.topicName_= "dataFusion";
  curMeasurement_.measurement_.resize(STATE_SIZE);
  curMeasurement_.covariance_.resize(STATE_SIZE, STATE_SIZE);
  curMeasurement_.measurement_.setZero();
  curMeasurement_.covariance_.setZero();
  curMeasurement_.updateVector_.resize(6);
  for (int i = 0;i<curMeasurement_.updateVector_.size();i++)
  {
    curMeasurement_.updateVector_[i] = 1;
  }
}

template<typename T>
FireflyRosFilter<T>::~FireflyRosFilter()
{
  topicSubs_.clear();
}

template<typename T>
void FireflyRosFilter<T>::loadParams()
{
  /* For diagnostic purposes, collect information about how many different
   * sources are measuring each absolute pose variable and do not have
   * differential integration enabled.
   */
  std::map<StateMembers, int> absPoseVarCounts;
  absPoseVarCounts[StateMemberX] = 0;
  absPoseVarCounts[StateMemberY] = 0;
  absPoseVarCounts[StateMemberZ] = 0;
  absPoseVarCounts[StateMemberRoll] = 0;
  absPoseVarCounts[StateMemberPitch] = 0;
  absPoseVarCounts[StateMemberYaw] = 0;

  // Same for twist variables
  std::map<StateMembers, int> twistVarCounts;
  twistVarCounts[StateMemberVx] = 0;
  twistVarCounts[StateMemberVy] = 0;
  twistVarCounts[StateMemberVz] = 0;
  twistVarCounts[StateMemberVroll] = 0;
  twistVarCounts[StateMemberVpitch] = 0;
  twistVarCounts[StateMemberVyaw] = 0;

  // Determine if we'll be printing diagnostic information
  nhLocal_.param("print_diagnostics", printDiagnostics_, true);

  // Grab the debug param. If true, the node will produce a LOT of output.
  bool debug;
  nhLocal_.param("debug", debug, false);

  if (debug)
  {
    std::string debugOutFile;

    try
    {
      nhLocal_.param("debug_out_file", debugOutFile, std::string("robot_localization_debug.txt"));
      debugStream_.open(debugOutFile.c_str());

      // Make sure we succeeded
      if (debugStream_.is_open())
      {
        filter_.setDebug(debug, &debugStream_);
      }
      else
      {
        ROS_WARN_STREAM("RosFilter::loadParams() - unable to create debug output file " << debugOutFile);
      }
    }
    catch(const std::exception &e)
    {
      ROS_WARN_STREAM("RosFilter::loadParams() - unable to create debug output file" << debugOutFile
                      << ". Error was " << e.what() << "\n");
    }
  }

  // These params specify the name of the robot's body frame (typically
  // base_link) and odometry frame (typically odom)
  nhLocal_.param("map_frame", mapFrameId_, std::string("map"));
  nhLocal_.param("odom_frame", odomFrameId_, std::string("odom"));
  nhLocal_.param("base_link_frame", baseLinkFrameId_, std::string("base_link"));

  tf2::Quaternion q(1.0, 0.0, 0.0, 0.0);
  tf2::Vector3 v(0.0, 0.0, 0.0);
  imu2BaselinkTrans_.setOrigin(v);
  imu2BaselinkTrans_.setRotation(q);

  /*
   * These parameters are designed to enforce compliance with REP-105:
   * http://www.ros.org/reps/rep-0105.html
   * When fusing absolute position data from sensors such as GPS, the state
   * estimate can undergo discrete jumps. According to REP-105, we want three
   * coordinate frames: map, odom, and base_link. The map frame can have
   * discontinuities, but is the frame with the most accurate position estimate
   * for the robot and should not suffer from drift. The odom frame drifts over
   * time, but is guaranteed to be continuous and is accurate enough for local
   * planning and navigation. The base_link frame is affixed to the robot. The
   * intention is that some odometry source broadcasts the odom->base_link
   * transform. The localization software should broadcast map->base_link.
   * However, tf does not allow multiple parents for a coordinate frame, so
   * we must *compute* map->base_link, but then use the existing odom->base_link
   * transform to compute *and broadcast* map->odom.
   *
   * The state estimation nodes in robot_localization therefore have two "modes."
   * If your world_frame parameter value matches the odom_frame parameter value,
   * then robot_localization will assume someone else is broadcasting a transform
   * from odom_frame->base_link_frame, and it will compute the
   * map_frame->odom_frame transform. Otherwise, it will simply compute the
   * odom_frame->base_link_frame transform.
   *
   * The default is the latter behavior (broadcast of odom->base_link).
   */
  nhLocal_.param("world_frame", worldFrameId_, odomFrameId_);

  ROS_FATAL_COND(mapFrameId_ == odomFrameId_ ||
                 odomFrameId_ == baseLinkFrameId_ ||
                 mapFrameId_ == baseLinkFrameId_,
                 "Invalid frame configuration! The values for map_frame, odom_frame, "
                 "and base_link_frame must be unique");

  // Try to resolve tf_prefix
  std::string tfPrefix = "";
  std::string tfPrefixPath = "";
  if (nhLocal_.searchParam("tf_prefix", tfPrefixPath))
  {
    nhLocal_.getParam(tfPrefixPath, tfPrefix);
  }

  // Append the tf prefix in a tf2-friendly manner
  FilterUtilities::appendPrefix(tfPrefix, mapFrameId_);
  FilterUtilities::appendPrefix(tfPrefix, odomFrameId_);
  FilterUtilities::appendPrefix(tfPrefix, baseLinkFrameId_);
  FilterUtilities::appendPrefix(tfPrefix, worldFrameId_);

  // Whether we're publshing the world_frame->base_link_frame transform
  nhLocal_.param("publish_tf", publishTransform_, true);

  // Whether we're publishing the acceleration state transform
  nhLocal_.param("publish_acceleration", publishAcceleration_, false);

  // Transform future dating
  double offsetTmp;
  nhLocal_.param("transform_time_offset", offsetTmp, 0.0);
  tfTimeOffset_.fromSec(offsetTmp);

  // Update frequency and sensor timeout
  double sensorTimeout;
  nhLocal_.param("frequency", frequency_, 30.0);
  nhLocal_.param("sensor_timeout", sensorTimeout, 1.0 / frequency_);
  filter_.setSensorTimeout(sensorTimeout);

  {
    ROS_WARN_STREAM("Filter history interval of " << historyLength_ <<
                    " specified, but smooth_lagged_data is set to false. Lagged data will not be smoothed.");
  }

  if(smoothLaggedData_ && historyLength_ < -1e9)
  {
    ROS_WARN_STREAM("Negative history interval of " << historyLength_ << " specified. Absolute value will be assumed.");
  }

  historyLength_ = ::fabs(historyLength_);

  // Initialization of control signals. To add the control signals to the estimation algorithm, the rotor angular velocities must be
  // translated into linear and angular accelerations.
  rotors_control::GetVehicleParameters(nhLocal_, &UAV_parameters_);
  rotors_control::calculateAllocationMatrix(UAV_parameters_.rotor_configuration_, &allocation_matrix_);
  inertia_mass_matrix_.setZero();
  inertia_mass_matrix_.block<3, 3>(0, 0) = UAV_parameters_.inertia_;
  inertia_mass_matrix_(3, 3) = UAV_parameters_.mass_;
  rotor_velocities_to_UAV_accelerations_.resize(4, 6);
  rotor_velocities_to_UAV_accelerations_.block<3, 6>(0, 0) = UAV_parameters_.inertia_.inverse() * allocation_matrix_.block<3, 6>(0, 0);
  rotor_velocities_to_UAV_accelerations_.block<1, 6>(3, 0) = allocation_matrix_.block<1, 6>(3, 0)/UAV_parameters_.mass_;

  //rotor_velocities_to_UAV_accelerations_ = allocation_matrix_ * inertia_mass_matrix_.inverse();
  filter_.setFireflyParams(inertia_mass_matrix_);

  RF_DEBUG("rotor_velocities_to_UAV_accelerations_ is \n\t" <<
           rotor_velocities_to_UAV_accelerations_ << "\n\t" <<
           "inertia_mass_matrix is \n\t" <<
           inertia_mass_matrix_<< "\n\t" <<
           "allocation_matrix is \n\t" <<
           allocation_matrix_<< "\n\t");

  double controlTimeout = sensorTimeout;
  std::vector<int> controlUpdateVector(CONTROL_SIZE, 0);
  for (int i = 0;i < CONTROL_SIZE;i++)
  {
    controlUpdateVector[i] = 1;
  }
  nhLocal_.param("control_timeout", controlTimeout, sensorTimeout);

  bool dynamicProcessNoiseCovariance = false;
  nhLocal_.param("dynamic_process_noise_covariance", dynamicProcessNoiseCovariance, false);
  filter_.setUseDynamicProcessNoiseCovariance(dynamicProcessNoiseCovariance);

  // Debugging writes to file
  RF_DEBUG("tf_prefix is " << tfPrefix <<
           "\nmap_frame is " << mapFrameId_ <<
           "\nodom_frame is " << odomFrameId_ <<
           "\nbase_link_frame is " << baseLinkFrameId_ <<
           "\nworld_frame is " << worldFrameId_ <<
           "\ntransform_time_offset is " << tfTimeOffset_.toSec() <<
           // "\nfrequency is " << frequency_ <<
           "\nsensor_timeout is " << filter_.getSensorTimeout() <<
           "\ntwo_d_mode is " << (twoDMode_ ? "true" : "false") <<
           "\nsmooth_lagged_data is " << (smoothLaggedData_ ? "true" : "false") <<
           "\nhistory_length is " << historyLength_ <<
           "\ncontrol_config is " << controlUpdateVector <<
           "\ncontrol_timeout is " << controlTimeout <<
           "\ndynamic_process_noise_covariance is " << (dynamicProcessNoiseCovariance ? "true" : "false") <<
           "\nprint_diagnostics is " << (printDiagnostics_ ? "true" : "false") << "\n");

  // Create a subscriber for manually setting/resetting pose
  setPoseSub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("set_pose",
                                                                        1,
                                                                        &FireflyRosFilter<T>::setPoseCallback,
                                                                        this, ros::TransportHints().tcpNoDelay(false));

  // Create a service for manually setting/resetting pose
  setPoseSrv_ = nh_.advertiseService("set_pose", &FireflyRosFilter<T>::setPoseSrvCallback, this); // this is the server program, it seems that the client program is for the users to write customly and is not yet written   by ZD

  // Init the last last measurement time so we don't get a huge initial delta
  filter_.setLastMeasurementTime(ros::Time::now().toSec());
  filter_.setLastUpdateTime(ros::Time::now().toSec());

  nhLocal_.param("xy_odom_available", xyOdomAvailable_, true);
  nhLocal_.param("estimating_xy", estimatingXY_, false);
  filter_.initOdomCondition(xyOdomAvailable_);
  filter_.setOdomCondition(estimatingXY_);

  // Now pull in each topic to which we want to subscribe. This is the very start   by ZD
  // Start with odom.
  size_t topicInd = 0;
  bool moreParams = false;
  do
  {
    // Build the string in the form of "odomX", where X is the odom topic number,
    // then check if we have any parameters with that value. Users need to make
    // sure they don't have gaps in their configs (e.g., odom0 and then odom2)
    std::stringstream ss;
    ss << "odom" << topicInd++;
    std::string odomTopicName = ss.str();
    moreParams = nhLocal_.hasParam(odomTopicName);

    if (moreParams)
    {
      // Determine if we want to integrate this sensor differentially
      bool differential;
      nhLocal_.param(odomTopicName + std::string("_differential"), differential, false);

      // Determine if we want to integrate this sensor relatively
      bool relative;
      nhLocal_.param(odomTopicName + std::string("_relative"), relative, false);

      if (relative && differential)
      {
        ROS_WARN_STREAM("Both " << odomTopicName << "_differential" << " and " << odomTopicName <<
                        "_relative were set to true. Using differential mode.");

        relative = false;
      }

      std::string odomTopic;
      nhLocal_.getParam(odomTopicName, odomTopic); // odomTopic is for example example/pose in ekf_template.yaml     by ZD

      // Check for pose rejection threshold
      double poseMahalanobisThresh;
      nhLocal_.param(odomTopicName + std::string("_pose_rejection_threshold"),
                     poseMahalanobisThresh,
                     std::numeric_limits<double>::max());

      // Check for twist rejection threshold
      double twistMahalanobisThresh;
      nhLocal_.param(odomTopicName + std::string("_twist_rejection_threshold"),
                     twistMahalanobisThresh,
                     std::numeric_limits<double>::max());

      // Now pull in its boolean update vector configuration. Create separate vectors for pose
      // and twist data, and then zero out the opposite values in each vector (no pose data in
      // the twist update vector and vice-versa).
      std::vector<int> updateVec = loadUpdateConfig(odomTopicName);  // row 2113 by ZD
      std::vector<int> poseUpdateVec = updateVec;
      std::fill(poseUpdateVec.begin() + POSITION_V_OFFSET, poseUpdateVec.begin() + POSITION_V_OFFSET + TWIST_SIZE, 0);
      std::vector<int> twistUpdateVec = updateVec;
      std::fill(twistUpdateVec.begin() + POSITION_OFFSET, twistUpdateVec.begin() + POSITION_OFFSET + POSE_SIZE, 0);

      int poseUpdateSum = std::accumulate(poseUpdateVec.begin(), poseUpdateVec.end(), 0); // get sum of the vector values     by ZD
      int twistUpdateSum = std::accumulate(twistUpdateVec.begin(), twistUpdateVec.end(), 0);
      int odomQueueSize = 1;
      nhLocal_.param(odomTopicName + "_queue_size", odomQueueSize, 1);

      const CallbackData poseCallbackData(odomTopicName + "_pose",poseUpdateVec, poseUpdateSum, differential,
        relative, poseMahalanobisThresh);
      const CallbackData twistCallbackData(odomTopicName + "_twist", twistUpdateVec, twistUpdateSum, false, false,
        twistMahalanobisThresh);

      bool nodelayOdom = false;
      nhLocal_.param(odomTopicName + "_nodelay", nodelayOdom, false);

      // Store the odometry topic subscribers so they don't go out of scope.
      if (poseUpdateSum + twistUpdateSum > 0)
      {
        topicSubs_.push_back(
          nh_.subscribe<nav_msgs::Odometry>(odomTopic, odomQueueSize,  // odom0_queue_size in example is 2   by ZD
            boost::bind(&FireflyRosFilter<T>::odometryCallback, this, _1, odomTopicName, poseCallbackData, twistCallbackData),
                                    ros::VoidPtr(), ros::TransportHints().tcpNoDelay(nodelayOdom)));
      }
      else
      {
        std::stringstream stream;
        stream << odomTopic << " is listed as an input topic, but all update variables are false";

        addDiagnostic(diagnostic_msgs::DiagnosticStatus::WARN,
                      odomTopic + "_configuration",
                      stream.str(),
                      true);
      }

      if (poseUpdateSum > 0)
      {
        if (differential)
        {
          twistVarCounts[StateMemberVx] += poseUpdateVec[StateMemberX];
          twistVarCounts[StateMemberVy] += poseUpdateVec[StateMemberY];
          twistVarCounts[StateMemberVz] += poseUpdateVec[StateMemberZ];
          twistVarCounts[StateMemberVroll] += poseUpdateVec[StateMemberRoll];
          twistVarCounts[StateMemberVpitch] += poseUpdateVec[StateMemberPitch];
          twistVarCounts[StateMemberVyaw] += poseUpdateVec[StateMemberYaw];

          // Load up the tau for differential filter of the odom (from the launch file/parameter server)
          std::vector<double> tauOdomStdVector(POSE_SIZE, 0);
          Eigen::VectorXd tauOdom(POSE_SIZE);
          tauOdom.setZero();
          if (nhLocal_.getParam("tau_"+ odomTopicName , tauOdomStdVector))
          {
            if(tauOdomStdVector.size() != POSE_SIZE)
            {
              ROS_ERROR_STREAM("Tau of the gyroscope differential filter must be of size " << POSE_SIZE << ". Provided config was of "
              "size " << tauOdomStdVector.size() << ". No time constants will be initialized for gyroscope differential filter");
            }
            else
            {
              /* std::cout<<"tauOdomStdVector0: "
                               <<tauOdomStdVector[0]<<std::endl
                               <<"tauOdomStdVector1: "
                               <<tauOdomStdVector[1]<<std::endl
                               <<"tauOdomStdVector2: "
                               <<tauOdomStdVector[2]<<std::endl; */
              for (int i=0;i<POSE_SIZE;i++)
              {
                tauOdom(i) = tauOdomStdVector[i];
              }
            }
          }
          else
          {
            ROS_ERROR_STREAM("Tau of the gyroscope differential filter initialization failed. No time constants will be initialized for gyroscope differential filter");
          }

          // Load up the tau config for differential filter of the gyroscope
          std::vector<int> tauOdomConfig(POSE_SIZE, 0);
          if(nhLocal_.getParam("tau_config_"+ odomTopicName, tauOdomConfig))
          {
            if(tauOdomConfig.size() != POSE_SIZE)
            {
              ROS_ERROR_STREAM("Tau configuration of the gyroscope differential filter must be of size " << POSE_SIZE << ". Provided config was of "
                "size " << tauOdomConfig.size() << ". ");
            }
          }

          rotors_control::DifferentialTracker* odomDifferentialTrackerPtr = new rotors_control::DifferentialTracker;
          odomDifferentialTrackerPtr->init(POSE_SIZE, tauOdom, tauOdomConfig);
          differentialTrackers_.insert(std::pair<std::string, rotors_control::DifferentialTracker*>(
                                          odomTopicName + "_pose", odomDifferentialTrackerPtr));
        }
        else
        {
          absPoseVarCounts[StateMemberX] += poseUpdateVec[StateMemberX];
          absPoseVarCounts[StateMemberY] += poseUpdateVec[StateMemberY];
          absPoseVarCounts[StateMemberZ] += poseUpdateVec[StateMemberZ];
          absPoseVarCounts[StateMemberRoll] += poseUpdateVec[StateMemberRoll];
          absPoseVarCounts[StateMemberPitch] += poseUpdateVec[StateMemberPitch];
          absPoseVarCounts[StateMemberYaw] += poseUpdateVec[StateMemberYaw];
        }
      }

      if (twistUpdateSum > 0)
      {
        twistVarCounts[StateMemberVx] += twistUpdateVec[StateMemberVx];
        twistVarCounts[StateMemberVy] += twistUpdateVec[StateMemberVx];
        twistVarCounts[StateMemberVz] += twistUpdateVec[StateMemberVz];
        twistVarCounts[StateMemberVroll] += twistUpdateVec[StateMemberVroll];
        twistVarCounts[StateMemberVpitch] += twistUpdateVec[StateMemberVpitch];
        twistVarCounts[StateMemberVyaw] += twistUpdateVec[StateMemberVyaw];
      }

      RF_DEBUG("Subscribed to " << odomTopic << " (" << odomTopicName << ")\n\t" <<
               odomTopicName << "_differential is " << (differential ? "true" : "false") << "\n\t" <<
               odomTopicName << "_pose_rejection_threshold is " << poseMahalanobisThresh << "\n\t" <<
               odomTopicName << "_twist_rejection_threshold is " << twistMahalanobisThresh << "\n\t" <<
               odomTopicName << "_queue_size is " << odomQueueSize << "\n\t" <<
               odomTopicName << " pose update vector is " << poseUpdateVec << "\t"<<
               odomTopicName << " twist update vector is " << twistUpdateVec);
    }
  }
  while (moreParams);

  // Repeat for pose
  topicInd = 0;
  moreParams = false;
  do
  {
    std::stringstream ss;
    ss << "pose" << topicInd++;
    std::string poseTopicName = ss.str();
    moreParams = nhLocal_.hasParam(poseTopicName);

    if (moreParams)
    {
      bool differential;
      nhLocal_.param(poseTopicName + std::string("_differential"), differential, false);

      // Determine if we want to integrate this sensor relatively
      bool relative;
      nhLocal_.param(poseTopicName + std::string("_relative"), relative, false);

      if (relative && differential)
      {
        ROS_WARN_STREAM("Both " << poseTopicName << "_differential" << " and " << poseTopicName <<
                        "_relative were set to true. Using differential mode.");

        relative = false;
      }

      std::string poseTopic;
      nhLocal_.getParam(poseTopicName, poseTopic);

      // Check for pose rejection threshold
      double poseMahalanobisThresh;
      nhLocal_.param(poseTopicName + std::string("_rejection_threshold"),
                     poseMahalanobisThresh,
                     std::numeric_limits<double>::max());

      int poseQueueSize = 1;
      nhLocal_.param(poseTopicName + "_queue_size", poseQueueSize, 1);

      bool nodelayPose = false;
      nhLocal_.param(poseTopicName + "_nodelay", nodelayPose, false);

      // Pull in the sensor's config, zero out values that are invalid for the pose type
      std::vector<int> poseUpdateVec = loadUpdateConfig(poseTopicName);
      std::fill(poseUpdateVec.begin() + POSITION_V_OFFSET,
                poseUpdateVec.begin() + POSITION_V_OFFSET + TWIST_SIZE,
                0);
      std::fill(poseUpdateVec.begin() + POSITION_A_OFFSET,
                poseUpdateVec.begin() + POSITION_A_OFFSET + ACCELERATION_SIZE,
                0);

      int poseUpdateSum = std::accumulate(poseUpdateVec.begin(), poseUpdateVec.end(), 0);

      if (poseUpdateSum > 0)
      {
        const CallbackData callbackData(poseTopicName, poseUpdateVec, poseUpdateSum, differential, relative,
          poseMahalanobisThresh);

        topicSubs_.push_back(
          nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(poseTopic, poseQueueSize,
            boost::bind(&FireflyRosFilter<T>::poseCallback, this, _1, callbackData, worldFrameId_, false),
                                                                   ros::VoidPtr(), ros::TransportHints().tcpNoDelay(nodelayPose)));

        if (differential)
        {
          twistVarCounts[StateMemberVx] += poseUpdateVec[StateMemberX];
          twistVarCounts[StateMemberVy] += poseUpdateVec[StateMemberY];
          twistVarCounts[StateMemberVz] += poseUpdateVec[StateMemberZ];
          twistVarCounts[StateMemberVroll] += poseUpdateVec[StateMemberRoll];
          twistVarCounts[StateMemberVpitch] += poseUpdateVec[StateMemberPitch];
          twistVarCounts[StateMemberVyaw] += poseUpdateVec[StateMemberYaw];
        }
        else
        {
          absPoseVarCounts[StateMemberX] += poseUpdateVec[StateMemberX];
          absPoseVarCounts[StateMemberY] += poseUpdateVec[StateMemberY];
          absPoseVarCounts[StateMemberZ] += poseUpdateVec[StateMemberZ];
          absPoseVarCounts[StateMemberRoll] += poseUpdateVec[StateMemberRoll];
          absPoseVarCounts[StateMemberPitch] += poseUpdateVec[StateMemberPitch];
          absPoseVarCounts[StateMemberYaw] += poseUpdateVec[StateMemberYaw];
        }
      }
      else
      {
        ROS_WARN_STREAM("Warning: " << poseTopic << " is listed as an input topic, "
                        "but all pose update variables are false");
      }

      RF_DEBUG("Subscribed to " << poseTopic << " (" << poseTopicName << ")\n\t" <<
               poseTopicName << "_differential is " << (differential ? "true" : "false") << "\n\t" <<
               poseTopicName << "_rejection_threshold is " << poseMahalanobisThresh << "\n\t" <<
               poseTopicName << "_queue_size is " << poseQueueSize << "\n\t" <<
               poseTopicName << " update vector is " << poseUpdateVec);
    }
  }
  while (moreParams);

  // Repeat for twist
  topicInd = 0;
  moreParams = false;
  do
  {
    std::stringstream ss;
    ss << "twist" << topicInd++;
    std::string twistTopicName = ss.str();
    moreParams = nhLocal_.hasParam(twistTopicName);

    if (moreParams)
    {
      std::string twistTopic;
      nhLocal_.getParam(twistTopicName, twistTopic);

      // Check for twist rejection threshold
      double twistMahalanobisThresh;
      nhLocal_.param(twistTopicName + std::string("_rejection_threshold"),
                     twistMahalanobisThresh,
                     std::numeric_limits<double>::max());

      int twistQueueSize = 1;
      nhLocal_.param(twistTopicName + "_queue_size", twistQueueSize, 1);

      bool nodelayTwist = false;
      nhLocal_.param(twistTopicName + "_nodelay", nodelayTwist, false);

      // Pull in the sensor's config, zero out values that are invalid for the twist type
      std::vector<int> twistUpdateVec = loadUpdateConfig(twistTopicName);
      std::fill(twistUpdateVec.begin() + POSITION_OFFSET, twistUpdateVec.begin() + POSITION_OFFSET + POSE_SIZE, 0);

      int twistUpdateSum = std::accumulate(twistUpdateVec.begin(), twistUpdateVec.end(), 0);

      if (twistUpdateSum > 0)
      {
        const CallbackData callbackData(twistTopicName, twistUpdateVec, twistUpdateSum,false, false,
          twistMahalanobisThresh);

        topicSubs_.push_back(
          nh_.subscribe<geometry_msgs::TwistWithCovarianceStamped>(twistTopic, twistQueueSize,
            boost::bind(&FireflyRosFilter<T>::twistCallback, this, _1, callbackData, baseLinkFrameId_), ros::VoidPtr(), ros::TransportHints().tcpNoDelay(nodelayTwist)));

        twistVarCounts[StateMemberVx] += twistUpdateVec[StateMemberVx];
        twistVarCounts[StateMemberVy] += twistUpdateVec[StateMemberVy];
        twistVarCounts[StateMemberVz] += twistUpdateVec[StateMemberVz];
        twistVarCounts[StateMemberVroll] += twistUpdateVec[StateMemberVroll];
        twistVarCounts[StateMemberVpitch] += twistUpdateVec[StateMemberVpitch];
        twistVarCounts[StateMemberVyaw] += twistUpdateVec[StateMemberVyaw];
      }
      else
      {
        ROS_WARN_STREAM("Warning: " << twistTopic << " is listed as an input topic, "
                        "but all twist update variables are false");
      }

      RF_DEBUG("Subscribed to " << twistTopic << " (" << twistTopicName << ")\n\t" <<
               twistTopicName << "_rejection_threshold is " << twistMahalanobisThresh << "\n\t" <<
               twistTopicName << "_queue_size is " << twistQueueSize << "\n\t" <<
               twistTopicName << " update vector is " << twistUpdateVec);
    }
  }
  while (moreParams);

  // Repeat for IMU
  topicInd = 0;
  moreParams = false;
  do
  {
    std::stringstream ss;
    ss << "imu" << topicInd++;
    std::string imuTopicName = ss.str();
    moreParams = nhLocal_.hasParam(imuTopicName);

    if (moreParams)
    {
      bool differential;
      nhLocal_.param(imuTopicName + std::string("_differential"), differential, false);

      // Determine if we want to integrate this sensor relatively
      bool relative;
      nhLocal_.param(imuTopicName + std::string("_relative"), relative, false);

      if (relative && differential)
      {
        ROS_WARN_STREAM("Both " << imuTopicName << "_differential" << " and " << imuTopicName <<
                        "_relative were set to true. Using differential mode.");

        relative = false;
      }

      std::string imuTopic;
      nhLocal_.getParam(imuTopicName, imuTopic);

      // Check for pose rejection threshold
      double poseMahalanobisThresh;
      nhLocal_.param(imuTopicName + std::string("_pose_rejection_threshold"),
                     poseMahalanobisThresh,
                     std::numeric_limits<double>::max());

      // Check for angular velocity rejection threshold
      double twistMahalanobisThresh;
      std::string imuTwistRejectionName =
        imuTopicName + std::string("_twist_rejection_threshold");
      nhLocal_.param(imuTwistRejectionName, twistMahalanobisThresh, std::numeric_limits<double>::max());

      // Check for acceleration rejection threshold
      double accelMahalanobisThresh;
      nhLocal_.param(imuTopicName + std::string("_linear_acceleration_rejection_threshold"),
                     accelMahalanobisThresh,
                     std::numeric_limits<double>::max());

      bool removeGravAcc = false;
      nhLocal_.param(imuTopicName + "_remove_gravitational_acceleration", removeGravAcc, false);
      removeGravitationalAcc_[imuTopicName + "_acceleration"] = removeGravAcc;

      // Now pull in its boolean update vector configuration and differential
      // update configuration (as this contains pose information)
      std::vector<int> updateVec = loadUpdateConfig(imuTopicName);

      std::vector<int> poseUpdateVec = updateVec; // put update vector in poseUpdateVec  by ZD
      std::fill(poseUpdateVec.begin() + POSITION_V_OFFSET,
                poseUpdateVec.begin() + POSITION_V_OFFSET + TWIST_SIZE,
                0);
      std::fill(poseUpdateVec.begin() + POSITION_A_OFFSET,
                poseUpdateVec.begin() + POSITION_A_OFFSET + ACCELERATION_SIZE,
                0);

      std::vector<int> twistUpdateVec = updateVec;
      std::fill(twistUpdateVec.begin() + POSITION_OFFSET,
                twistUpdateVec.begin() + POSITION_OFFSET + POSE_SIZE,
                0);
      std::fill(twistUpdateVec.begin() + POSITION_A_OFFSET,
                twistUpdateVec.begin() + POSITION_A_OFFSET + ACCELERATION_SIZE,
                0);

      std::vector<int> accelUpdateVec = updateVec;
      std::fill(accelUpdateVec.begin() + POSITION_OFFSET,
                accelUpdateVec.begin() + POSITION_OFFSET + POSE_SIZE,
                0);
      std::fill(accelUpdateVec.begin() + POSITION_V_OFFSET,
                accelUpdateVec.begin() + POSITION_V_OFFSET + TWIST_SIZE,
                0);

      int poseUpdateSum = std::accumulate(poseUpdateVec.begin(), poseUpdateVec.end(), 0);
      int twistUpdateSum = std::accumulate(twistUpdateVec.begin(), twistUpdateVec.end(), 0);
      int accelUpdateSum = std::accumulate(accelUpdateVec.begin(), accelUpdateVec.end(), 0);

      int imuQueueSize = 1;
      nhLocal_.param(imuTopicName + "_queue_size", imuQueueSize, 1);

      bool nodelayImu = false;
      nhLocal_.param(imuTopicName + "_nodelay", nodelayImu, false);

      if (poseUpdateSum + twistUpdateSum + accelUpdateSum > 0)
      {
        const CallbackData poseCallbackData(imuTopicName + "_pose", poseUpdateVec, poseUpdateSum, differential,
          relative, poseMahalanobisThresh);
        const CallbackData twistCallbackData(imuTopicName + "_twist", twistUpdateVec, twistUpdateSum, differential,
          relative, poseMahalanobisThresh);
        const CallbackData accelCallbackData(imuTopicName + "_acceleration", accelUpdateVec, accelUpdateSum,
          differential, relative, accelMahalanobisThresh);

        topicSubs_.push_back(
          nh_.subscribe<sensor_msgs::Imu>(imuTopic, imuQueueSize,
            boost::bind(&FireflyRosFilter<T>::imuCallback, this, _1, imuTopicName, poseCallbackData, twistCallbackData,
              accelCallbackData), ros::VoidPtr(), ros::TransportHints().tcpNoDelay(nodelayImu)));

        //imuPubs_.insert(std::pair<std::string, ros::Publisher>(
                            //imuTopicName + "_acceleration", nh_.advertise<geometry_msgs::PointStamped>(imuTopicName+"_accel_corrected", 2)));
      }
      else
      {
        ROS_WARN_STREAM("Warning: " << imuTopic << " is listed as an input topic, "
                        "but all its update variables are false");
      }

      if (poseUpdateSum > 0)
      {
        if (differential)
        {
          twistVarCounts[StateMemberVroll] += poseUpdateVec[StateMemberRoll];
          twistVarCounts[StateMemberVpitch] += poseUpdateVec[StateMemberPitch];
          twistVarCounts[StateMemberVyaw] += poseUpdateVec[StateMemberYaw];
        }
        else
        {
          absPoseVarCounts[StateMemberRoll] += poseUpdateVec[StateMemberRoll];
          absPoseVarCounts[StateMemberPitch] += poseUpdateVec[StateMemberPitch];
          absPoseVarCounts[StateMemberYaw] += poseUpdateVec[StateMemberYaw];
        }
      }

      if (twistUpdateSum > 0)
      {
        twistVarCounts[StateMemberVroll] += twistUpdateVec[StateMemberVroll];
        twistVarCounts[StateMemberVpitch] += twistUpdateVec[StateMemberVpitch];
        twistVarCounts[StateMemberVyaw] += twistUpdateVec[StateMemberVyaw];
      }

      RF_DEBUG("Subscribed to " << imuTopic << " (" << imuTopicName << ")\n\t" <<
               imuTopicName << "_differential is " << (differential ? "true" : "false") << "\n\t" <<
               imuTopicName << "_pose_rejection_threshold is " << poseMahalanobisThresh << "\n\t" <<
               imuTopicName << "_twist_rejection_threshold is " << twistMahalanobisThresh << "\n\t" <<
               imuTopicName << "_linear_acceleration_rejection_threshold is " << accelMahalanobisThresh << "\n\t" <<
               imuTopicName << "_remove_gravitational_acceleration is " <<
                               (removeGravAcc ? "true" : "false") << "\n\t" <<
               imuTopicName << "_queue_size is " << imuQueueSize << "\n\t" <<
               imuTopicName << " pose update vector is " << poseUpdateVec << "\t"<<
               imuTopicName << " twist update vector is " << twistUpdateVec << "\t" <<
               imuTopicName << " acceleration update vector is " << accelUpdateVec);
    }
  }
  while (moreParams);

  // Now that we've checked if IMU linear acceleration is being used, we can determine our final control parameters
  if(std::accumulate(controlUpdateVector.begin(), controlUpdateVector.end(), 0) == 0)
  {
    ROS_ERROR_STREAM("control_config has only false values. No control term "
      "will be used.");
  }

  // Control term is now the one that can not be deactivated in comparison with RosFilter
  latestControl_.resize(CONTROL_SIZE);
  latestControl_.setZero();
  filter_.setControlParams(controlUpdateVector, controlTimeout);  

  controlSub_ = nh_.subscribe<mav_msgs::Actuators>(actuatorTopic_, 1, &FireflyRosFilter<T>::actuatorCallback, this);
  motorSub_ = nh_.subscribe(motorTopic_, 1, &FireflyRosFilter::motorCallback, this);
  clockSub_ = nh_.subscribe(clockTopic_, 1, &FireflyRosFilter::clockCallback, this);
  modeSub_ = nh_.subscribe(modeTopic_, 1, &FireflyRosFilter::modeCallback, this);

  /* Warn users about:
  *    1. Multiple non-differential input sources
  *    2. No absolute *or* velocity measurements for pose variables
  */
  if (printDiagnostics_)
  {
    for (int stateVar = StateMemberX; stateVar <= StateMemberYaw; ++stateVar)
    {
      if (absPoseVarCounts[static_cast<StateMembers>(stateVar)] > 1)
      {
        std::stringstream stream;
        stream <<  absPoseVarCounts[static_cast<StateMembers>(stateVar - POSITION_OFFSET)] <<
            " absolute pose inputs detected for " << stateVariableNames_[stateVar] <<
            ". This may result in oscillations. Please ensure that your variances for each "
            "measured variable are set appropriately.";

        addDiagnostic(diagnostic_msgs::DiagnosticStatus::WARN,
                      stateVariableNames_[stateVar] + "_configuration",
                      stream.str(),
                      true);
      }
      else if (absPoseVarCounts[static_cast<StateMembers>(stateVar)] == 0)
      {
        if ((static_cast<StateMembers>(stateVar) == StateMemberX &&
             twistVarCounts[static_cast<StateMembers>(StateMemberVx)] == 0) ||
            (static_cast<StateMembers>(stateVar) == StateMemberY &&
             twistVarCounts[static_cast<StateMembers>(StateMemberVy)] == 0) ||
            (static_cast<StateMembers>(stateVar) == StateMemberZ &&
             twistVarCounts[static_cast<StateMembers>(StateMemberVz)] == 0 &&
             twoDMode_ == false) ||
            (static_cast<StateMembers>(stateVar) == StateMemberRoll &&
             twistVarCounts[static_cast<StateMembers>(StateMemberVroll)] == 0 &&
             twoDMode_ == false) ||
            (static_cast<StateMembers>(stateVar) == StateMemberPitch &&
             twistVarCounts[static_cast<StateMembers>(StateMemberVpitch)] == 0 &&
             twoDMode_ == false) ||
            (static_cast<StateMembers>(stateVar) == StateMemberYaw &&
             twistVarCounts[static_cast<StateMembers>(StateMemberVyaw)] == 0))
        {
          std::stringstream stream;
          stream << "Neither " << stateVariableNames_[stateVar] << " nor its "
                    "velocity is being measured. This will result in unbounded "
                    "error growth and erratic filter behavior.";

          addDiagnostic(diagnostic_msgs::DiagnosticStatus::ERROR,
                        stateVariableNames_[stateVar] + "_configuration",
                        stream.str(),
                        true);
        }
      }
    }
  }

  // Load up the process noise covariance (from the launch file/parameter server)
  Eigen::MatrixXd processNoiseCovariance(STATE_SIZE, STATE_SIZE);
  processNoiseCovariance.setZero();
  XmlRpc::XmlRpcValue processNoiseCovarConfig;

  if (nhLocal_.hasParam("process_noise_covariance"))
  {
    try
    {
      nhLocal_.getParam("process_noise_covariance", processNoiseCovarConfig);

      ROS_ASSERT(processNoiseCovarConfig.getType() == XmlRpc::XmlRpcValue::TypeArray);

      int matSize = processNoiseCovariance.rows();

      for (int i = 0; i < matSize; i++)
      {
        for (int j = 0; j < matSize; j++)
        {
          try
          {
            // These matrices can cause problems if all the types
            // aren't specified with decimal points. Handle that
            // using string streams.
            std::ostringstream ostr;
            ostr << processNoiseCovarConfig[matSize * i + j];
            std::istringstream istr(ostr.str());
            istr >> processNoiseCovariance(i, j);
          }
          catch(XmlRpc::XmlRpcException &e)
          {
            throw e;
          }
          catch(...)
          {
            throw;
          }
        }
      }

      RF_DEBUG("Process noise covariance is:\n" << processNoiseCovariance << "\n");
    }
    catch (XmlRpc::XmlRpcException &e)
    {
      ROS_ERROR_STREAM("ERROR reading sensor config: " <<
                       e.getMessage() <<
                       " for process_noise_covariance (type: " <<
                       processNoiseCovarConfig.getType() << ")");
    }

    filter_.setProcessNoiseCovariance(processNoiseCovariance);
  }

  // Load up the initial estimate error covariance (from the launch file/parameter server)
  Eigen::MatrixXd initialEstimateErrorCovariance(STATE_SIZE, STATE_SIZE);
  initialEstimateErrorCovariance.setZero();
  XmlRpc::XmlRpcValue estimateErrorCovarConfig;

  if (nhLocal_.hasParam("initial_estimate_covariance"))
  {
    try
    {
      nhLocal_.getParam("initial_estimate_covariance", estimateErrorCovarConfig);

      ROS_ASSERT(estimateErrorCovarConfig.getType() == XmlRpc::XmlRpcValue::TypeArray);

      int matSize = initialEstimateErrorCovariance.rows();

      for (int i = 0; i < matSize; i++)
      {
        for (int j = 0; j < matSize; j++)
        {
          try
          {
            // These matrices can cause problems if all the types
            // aren't specified with decimal points. Handle that
            // using string streams.
            std::ostringstream ostr;
            ostr << estimateErrorCovarConfig[matSize * i + j];
            std::istringstream istr(ostr.str());
            istr >> initialEstimateErrorCovariance(i, j);
          }
          catch(XmlRpc::XmlRpcException &e)
          {
            throw e;
          }
          catch(...)
          {
            throw;
          }
        }
      }

      RF_DEBUG("Initial estimate error covariance is:\n" << initialEstimateErrorCovariance << "\n");
    }
    catch (XmlRpc::XmlRpcException &e)  // if error type is XmlRpcException
    {
      ROS_ERROR_STREAM("ERROR reading initial_estimate_covariance (type: " <<
                       estimateErrorCovarConfig.getType() <<
                       "): " <<
                       e.getMessage());
    }
    catch(...)
    {
      ROS_ERROR_STREAM(
        "ERROR reading initial_estimate_covariance (type: " << estimateErrorCovarConfig.getType() << ")"); // for any other type of error
    }
    filter_.setEstimateErrorCovariance(initialEstimateErrorCovariance);
  }

  nhLocal_.param("use_measurement_noise_covariance",  useMeasNoiseCovariance_, true);
  // Load up the initial estimate error covariance (from the launch file/parameter server)
  Eigen::MatrixXd initialMeasNoiseCovariance(STATE_SIZE, STATE_SIZE);
  initialMeasNoiseCovariance.setZero();
  XmlRpc::XmlRpcValue measNoiseCovarConfig;

  if (nhLocal_.hasParam("initial_measurement_noise_covariance"))
  {
    try
    {
      nhLocal_.getParam("initial_measurement_noise_covariance", measNoiseCovarConfig);

      ROS_ASSERT(measNoiseCovarConfig.getType() == XmlRpc::XmlRpcValue::TypeArray);

      int matSize = initialMeasNoiseCovariance.rows();

      for (int i = 0; i < matSize; i++)
      {
        for (int j = 0; j < matSize; j++)
        {
          try
          {
            // These matrices can cause problems if all the types
            // aren't specified with decimal points. Handle that
            // using string streams.
            std::ostringstream ostr;
            ostr << measNoiseCovarConfig[matSize * i + j];
            std::istringstream istr(ostr.str());
            istr >> initialMeasNoiseCovariance(i, j);
          }
          catch(XmlRpc::XmlRpcException &e)
          {
            throw e;
          }
          catch(...)
          {
            throw;
          }
        }
      }

      RF_DEBUG("Initial measurement error covariance is:\n" << initialMeasNoiseCovariance << "\n");
    }
    catch (XmlRpc::XmlRpcException &e)  // if error type is XmlRpcException
    {
      ROS_ERROR_STREAM("ERROR reading initial_measurement_noise_covariance (type: " <<
                       measNoiseCovarConfig.getType() <<
                       "): " <<
                       e.getMessage());
    }
    catch(...)
    {
      ROS_ERROR_STREAM(
        "ERROR reading initial_measurement_noise_covariance (type: " << measNoiseCovarConfig.getType() << ")"); // for any other type of error
    }

    filter_.setMeasurementCovariance(useMeasNoiseCovariance_, initialMeasNoiseCovariance);
    curMeasurement_.covariance_ = initialMeasNoiseCovariance;
  }
}

template<typename T>
void FireflyRosFilter<T>::run()  // to row 1812  by ZD
{
  ros::Time::init();

  loadParams();

  if (printDiagnostics_)
  { // diagnostic_updater::DiagnosticTaskVektor::add(name, c, f) @name wrapper of diag msg, @c class instance the method is being called on; @f method to call to fill out the wrapper   by ZD
    diagnosticUpdater_.add("Filter diagnostic updater", this, &FireflyRosFilter<T>::aggregateDiagnostics); // @name is the task name @f is a function pointer whose variable is diagnostic_updater::diagnosticStatusWrapper  by ZD
  }

  // Set up the frequency diagnostic
  double minFrequency = frequency_ - 2;
  double maxFrequency = frequency_ + 2;
  diagnostic_updater::HeaderlessTopicDiagnostic freqDiag("odometry/filtered",
                                                         diagnosticUpdater_,
                                                         diagnostic_updater::FrequencyStatusParam(&minFrequency,
                                                                                                  &maxFrequency,
                                                                                                  0.1, 10));

  // We may need to broadcast a different transform than
  // the one we've already calculated.
  tf2::Transform mapOdomTrans;
  tf2::Transform odomBaseLinkTrans;
  geometry_msgs::TransformStamped mapOdomTransMsg;
  ros::Time curTime;
  ros::Time lastDiagTime = ros::Time::now();

  // Clear out the transforms
  worldBaseLinkTransMsg_.transform = tf2::toMsg(tf2::Transform::getIdentity());
  mapOdomTransMsg.transform = tf2::toMsg(tf2::Transform::getIdentity()); // now we pause here

  // Publisher
  ros::Publisher positionPub = nh_.advertise<nav_msgs::Odometry>("odometry/filtered", 20);
  tf2_ros::TransformBroadcaster worldTransformBroadcaster;

  // Optional acceleration publisher
  ros::Publisher accelPub;
  if (publishAcceleration_)
  {
    //accelPub = nh_.advertise<geometry_msgs::AccelWithCovarianceStamped>("accel/filtered", 20);
    accelPub = nh_.advertise<geometry_msgs::PointStamped>("accel/filtered", 20);
  }

  // ros::Rate loop_rate(frequency_);

  while (ros::ok())
  {
    // The spin will call all the available callbacks and enqueue
    // their received measurements
    ros::spinOnce();

    // std::cout<<"curMeas is: "<<curMeasurement_.measurement_<<std::endl;
    if (!updateQueue_.empty())
    {
      while (!updateQueue_.empty())
      {
        // curTime = ros::Time::now();
        curTime = currentTime_;

        // Now we'll integrate any measurements we've received
        integrateMeasurements(curTime);  // row 466 main function that include processMeasurement( the function that process the prediction and correction)  by ZD

        // Get latest state and publish it
        nav_msgs::Odometry filteredPosition;

        if (getFilteredOdometryMessage(filteredPosition))
        {
          worldBaseLinkTransMsg_.header.stamp = filteredPosition.header.stamp + tfTimeOffset_;
          worldBaseLinkTransMsg_.header.frame_id = filteredPosition.header.frame_id;
          worldBaseLinkTransMsg_.child_frame_id = filteredPosition.child_frame_id;

          worldBaseLinkTransMsg_.transform.translation.x = filteredPosition.pose.pose.position.x;
          worldBaseLinkTransMsg_.transform.translation.y = filteredPosition.pose.pose.position.y;
          worldBaseLinkTransMsg_.transform.translation.z = filteredPosition.pose.pose.position.z;
          worldBaseLinkTransMsg_.transform.rotation = filteredPosition.pose.pose.orientation;

          // If the worldFrameId_ is the odomFrameId_ frame, then we can just send the transform. If the
          // worldFrameId_ is the mapFrameId_ frame, we'll have some work to do.
          if (publishTransform_)
          {
            if (filteredPosition.header.frame_id == odomFrameId_)
            {
              worldTransformBroadcaster.sendTransform(worldBaseLinkTransMsg_);
            }
            else if (filteredPosition.header.frame_id == mapFrameId_)
            {
              try
              {
                tf2::Transform worldBaseLinkTrans;
                tf2::fromMsg(worldBaseLinkTransMsg_.transform, worldBaseLinkTrans);

                tf2::fromMsg(tfBuffer_.lookupTransform(baseLinkFrameId_, odomFrameId_, ros::Time(0)).transform,
                               odomBaseLinkTrans);

                /*
                  * First, see these two references:
                  * http://wiki.ros.org/tf/Overview/Using%20Published%20Transforms#lookupTransform
                  * http://wiki.ros.org/geometry/CoordinateFrameConventions#Transform_Direction
                  * We have a transform from mapFrameId_->baseLinkFrameId_, but it would actually transform
                  * a given pose from baseLinkFrameId_->mapFrameId_. We then used lookupTransform, whose
                  * first two arguments are target frame and source frame, to get a transform from
                  * baseLinkFrameId_->odomFrameId_. However, this transform would actually transform data
                  * from odomFrameId_->baseLinkFrameId_. Now imagine that we have a position in the
                  * mapFrameId_ frame. First, we multiply it by the inverse of the
                  * mapFrameId_->baseLinkFrameId, which will transform that data from mapFrameId_ to
                  * baseLinkFrameId_. Now we want to go from baseLinkFrameId_->odomFrameId_, but the
                  * transform we have takes data from odomFrameId_->baseLinkFrameId_, so we need its
                  * inverse as well. We have now transformed our data from mapFrameId_ to odomFrameId_.
                  * However, if we want other users to be able to do the same, we need to broadcast
                  * the inverse of that entire transform.
                  */

                mapOdomTrans.mult(worldBaseLinkTrans, odomBaseLinkTrans);

                mapOdomTransMsg.transform = tf2::toMsg(mapOdomTrans);
                mapOdomTransMsg.header.stamp = filteredPosition.header.stamp + tfTimeOffset_;
                mapOdomTransMsg.header.frame_id = mapFrameId_;
                mapOdomTransMsg.child_frame_id = "odometry/filtered";

                worldTransformBroadcaster.sendTransform(mapOdomTransMsg);
              }
              catch(...)
              {
                ROS_ERROR_STREAM("Could not obtain transform from " << odomFrameId_ << "->" << baseLinkFrameId_);
              }
            }
            else
            {
              ROS_ERROR_STREAM("Odometry message frame_id was " << filteredPosition.header.frame_id <<
                                 ", expected " << mapFrameId_ << " or " << odomFrameId_);
            }
          }

          // Fire off the position and the transform
          positionPub.publish(filteredPosition);

          if (printDiagnostics_)
          {
            freqDiag.tick();
          }
        }

        // Publish the acceleration if desired and filter is initialized
        // geometry_msgs::AccelWithCovarianceStamped filteredAcceleration;
        geometry_msgs::PointStamped filteredAcceleration;
        if (publishAcceleration_ && getFilteredAccelMessage(filteredAcceleration))
        {
          accelPub.publish(filteredAcceleration);
        }

       /* Diagnostics can behave strangely when playing back from bag
         * files and using simulated time, so we have to check for
         * time suddenly moving backwards as well as the standard
         * timeout criterion before publishing. */
        double diagDuration = (curTime - lastDiagTime).toSec();
        if (printDiagnostics_ && (diagDuration >= diagnosticUpdater_.getPeriod() || diagDuration < 0.0))
        {
          diagnosticUpdater_.force_update();
          lastDiagTime = curTime;
        }

        // Clear out expired history data
        if (smoothLaggedData_)
        {
          clearExpiredHistory(filter_.getLastMeasurementTime() - historyLength_);
        }
        updateQueue_.pop_front();
      }
    }
  }
}

// original integrateMeasurements
// /*
template<typename T>
void FireflyRosFilter<T>::integrateMeasurements(const ros::Time &currentTime)
{
  const double currentTimeSec = currentTime.toSec();

  RF_DEBUG("------ FireflyRosFilter::integrateMeasurements ------\n\n"
           "Integration time is " << std::setprecision(20) << currentTimeSec << "\n"
           << measurementQueue_.size() << " measurements in queue.\n");

  if (!measurementQueue_.empty())
  {
    // Check if the first measurement we're going to process is older than the filter's last measurement.
    // This means we have received an out-of-sequence message (one with an old timestamp), and we need to
    // revert both the filter state and measurement queue to the first state that preceded the time stamp
    // of our first measurement.
    const MeasurementPtr& firstMeasurement =  measurementQueue_.top();
    int restoredMeasurementCount = 0;
    if (smoothLaggedData_ && firstMeasurement->time_ < filter_.getLastMeasurementTime())
    {
      RF_DEBUG("Received a measurement that was " << filter_.getLastMeasurementTime() - firstMeasurement->time_ <<
               " seconds in the past. Reverting filter state and measurement queue...");

      int originalCount = static_cast<int>(measurementQueue_.size());
      if (!revertTo(firstMeasurement->time_ - 1e-9))
      {
        RF_DEBUG("ERROR: history interval is too small to revert to time " << firstMeasurement->time_ << "\n");
        ROS_WARN_STREAM_THROTTLE(10.0, "Received old measurement for topic " << firstMeasurement->topicName_ <<
                                 ", but history interval is insufficiently sized to revert state and measurement queue.");
        restoredMeasurementCount = 0;
      }

      restoredMeasurementCount = static_cast<int>(measurementQueue_.size()) - originalCount;
    }

    while (!measurementQueue_.empty())  // keep correcting until measurementQueue_ is empty  by ZD
    {
      MeasurementPtr measurement =  measurementQueue_.top();

      // If we've reached a measurement that has a time later than now, it should wait until a future iteration.
      // Since measurements are stored in a priority queue, all remaining measurements will be in the future.
      if (measurement->time_ > currentTime.toSec())
      {
        break;
      }

       measurementQueue_.pop(); // pop out the top element of the queue   by ZD

      // This will call predict and, if necessary, correct
       filter_.processMeasurement(*(measurement.get()));  // prediction and correction and filter.state_ update are here

      // Store old states and measurements if we're smoothing
      if (smoothLaggedData_)
      {
        // Invariant still holds: measurementHistoryDeque_.back().time_ < measurementQueue_.top().time_
         measurementHistory_.push_back(measurement);

        // We should only save the filter state once per unique timestamp
        if (measurementQueue_.empty() ||
            ::fabs(measurementQueue_.top()->time_ -  filter_.getLastMeasurementTime()) > 1e-9)
        {
           saveFilterState(filter_); // row 2851 this is the function that keeping fills out the filterStateHistory_   by ZD
        }
      }
    }
     filter_.setLastUpdateTime(currentTimeSec);
  }
  else if (filter_.getInitializedStatus())
  {
    // In the event that we don't get any measurements for a long time,
    // we still need to continue to estimate our state. Therefore, we
    // should project the state forward here.
    double lastUpdateDelta = currentTimeSec - filter_.getLastUpdateTime();
    // If we get a large delta, then continuously predict until
    if (lastUpdateDelta >=  filter_.getSensorTimeout())
    {
      RF_DEBUG("Sensor timeout! Last update time was " << filter_.getLastUpdateTime() <<
                   ", current time is " << currentTimeSec <<
                   ", delta is " << lastUpdateDelta << "\n");

      filter_.validateDelta(lastUpdateDelta);
      filter_.predict(currentTimeSec, lastUpdateDelta);

      // Update the last measurement time and last update time
      filter_.setLastMeasurementTime(filter_.getLastMeasurementTime() + lastUpdateDelta);
      filter_.setLastUpdateTime(filter_.getLastUpdateTime() + lastUpdateDelta);
    }
  }
  else
  {
    RF_DEBUG("Filter not yet initialized.\n");
  }
  RF_DEBUG("\n----- /RosFilter::integrateMeasurements ------\n");
}
// */

// modified integrateMeasurements
/*
template<typename T>
void FireflyRosFilter<T>::integrateMeasurements(const ros::Time &currentTime)
{
  const double currentTimeSec = currentTime.toSec();

  // This will call predict and, if necessary, correct
  filter_.processMeasurement(curMeasurement_);  // prediction and correction and filter.state_ update are here

  // Store old states and measurements if we're smoothing
  if (smoothLaggedData_)
 {
    // Invariant still holds: measurementHistoryDeque_.back().time_ < measurementQueue_.top().time_
    // measurementHistory_.push_back(curMeasurement_);

    // We should only save the filter state once per unique timestamp
    if (measurementQueue_.empty() ||
        ::fabs(measurementQueue_.top()->time_ -  filter_.getLastMeasurementTime()) > 1e-9)
    {
       saveFilterState(filter_); // row 2851 this is the function that keeping fills out the filterStateHistory_   by ZD
    }
  }
  filter_.setLastUpdateTime(currentTimeSec);
}
// */

/////////////////////////////////////////////////////////////////////////////
/// Callback functions
/////////////////////////////////////////////////////////////////////////////
template<typename T>
void FireflyRosFilter<T>::motorCallback(const std_msgs::Bool& msg)
{
  motorStatus_ = msg.data;
  filter_.setMotorStatus(motorStatus_);
}

template<typename T>
void FireflyRosFilter<T>::modeCallback(const std_msgs::Bool& msg) {
  estimatingXY_ = msg.data;
  if (estimatingXY_ )
  { ROS_INFO_NAMED("ekf_landing", "start estimation");}
  else
  { ROS_INFO_NAMED("ekf_landing", "stop estimation");}
  filter_.setOdomCondition(estimatingXY_);
}

// This one triggers the correction step of the Kalman filter.
template<typename T>
void FireflyRosFilter<T>::clockCallback(const rosgraph_msgs::Clock& msg) {
  if (!permittDelay_)
  {
    updateQueue_.clear();
  }
  currentTime_ = msg.clock;
  updateQueue_.push_back(currentTime_);
}

// This one triggers the prediction step of the Kalman filter
template<typename T>
void FireflyRosFilter<T>::actuatorCallback(const mav_msgs::ActuatorsConstPtr& msg) {
  ref_rotor_velocities << msg->angular_velocities[0]*msg->angular_velocities[0],
                                                msg->angular_velocities[1]*msg->angular_velocities[1],
                                                msg->angular_velocities[2]*msg->angular_velocities[2],
                                                msg->angular_velocities[3]*msg->angular_velocities[3],
                                                msg->angular_velocities[4]*msg->angular_velocities[4],
                                                msg->angular_velocities[5]*msg->angular_velocities[5];
  // latestControl_ = [a_p, a_q, a_r, a_w, time];
  latestControl_ = rotor_velocities_to_UAV_accelerations_ * ref_rotor_velocities;
  latestControlTime_ = msg->header.stamp;
  filter_.setControl(latestControl_, msg->header.stamp.toSec());
 /* std::cout<<"Ap:"<<latestControl_(0)<<
                        ", Aq:"<<latestControl_(1)<<
                        ", Ar:"<<latestControl_(2)<<
                        ", Az:"<<latestControl_(3)<<std::endl; */
}

// original odometryCallback
/*
template<typename T>
void FireflyRosFilter<T>::odometryCallback(const nav_msgs::Odometry::ConstPtr &msg, const std::string &topicName,
  const CallbackData &poseCallbackData, const CallbackData &twistCallbackData)
{
  // If we've just reset the filter, then we want to ignore any messages
  // that arrive with an older timestamp
  if (msg->header.stamp <= lastSetPoseTime_)
  {
    std::stringstream stream;
    stream << "The " << topicName << " message has a timestamp equal to or before the last filter reset, " <<
              "this message will be ignored. This may indicate an empty or bad timestamp. (message time: " <<
              msg->header.stamp.toSec() << ")";
    addDiagnostic(diagnostic_msgs::DiagnosticStatus::WARN,
                  topicName + "_timestamp",
                  stream.str(),
                  false);
    RF_DEBUG("Received message that preceded the most recent pose reset. Ignoring...");

    return;
  }

  RF_DEBUG("------ FireflyRosFilter::odometryCallback (" << topicName << ") ------\n" << "Odometry message:\n" << *msg);

  if (poseCallbackData.updateSum_ > 0)
  {
    // Grab the pose portion of the message and pass it to the poseCallback
    geometry_msgs::PoseWithCovarianceStamped *posPtr = new geometry_msgs::PoseWithCovarianceStamped();
    posPtr->header = msg->header;
    posPtr->pose = msg->pose;  // Entire pose object, also copies covariance

    geometry_msgs::PoseWithCovarianceStampedConstPtr pptr(posPtr);
    poseCallback(pptr, poseCallbackData, worldFrameId_, false);
  }

  if (twistCallbackData.updateSum_ > 0)
  {
    // Grab the twist portion of the message and pass it to the twistCallback
    geometry_msgs::TwistWithCovarianceStamped *twistPtr = new geometry_msgs::TwistWithCovarianceStamped();
    twistPtr->header = msg->header;
    twistPtr->header.frame_id = msg->child_frame_id;
    twistPtr->twist = msg->twist;  // Entire twist object, also copies covariance

    geometry_msgs::TwistWithCovarianceStampedConstPtr tptr(twistPtr);
    twistCallback(tptr, twistCallbackData, baseLinkFrameId_);
  }

  RF_DEBUG("\n----- /RosFilter::odometryCallback (" << topicName << ") ------\n");
}
// */

// modified odometryCallback
// /*
template<typename T>
void FireflyRosFilter<T>::odometryCallback(const nav_msgs::Odometry::ConstPtr &msg, const std::string &topicName,
  const CallbackData &poseCallbackData, const CallbackData &twistCallbackData)
{
  // If we've just reset the filter, then we want to ignore any messages
  // that arrive with an older timestamp
  if (msg->header.stamp <= lastSetPoseTime_)
  {
    std::stringstream stream;
    stream << "The " << topicName << " message has a timestamp equal to or before the last filter reset, " <<
              "this message will be ignored. This may indicate an empty or bad timestamp. (message time: " <<
              msg->header.stamp.toSec() << ")";
    addDiagnostic(diagnostic_msgs::DiagnosticStatus::WARN,
                  topicName + "_timestamp",
                  stream.str(),
                  false);
    RF_DEBUG("Received message that preceded the most recent pose reset. Ignoring...");

    return;
  }

  RF_DEBUG("------ FireflyRosFilter::odometryCallback (" << topicName << ") ------\n" << "Odometry message:\n" << *msg);

  if (poseCallbackData.updateSum_ > 0)
  {
    // Grab the pose portion of the message and pass it to the poseCallback
    geometry_msgs::PoseWithCovarianceStamped *posPtr = new geometry_msgs::PoseWithCovarianceStamped();
    posPtr->header = msg->header;
    posPtr->pose = msg->pose;  // Entire pose object, also copies covariance

    geometry_msgs::PoseWithCovarianceStampedConstPtr pptr(posPtr);
    poseCallback(pptr, poseCallbackData, worldFrameId_, false);
  }

  if (twistCallbackData.updateSum_ > 0)
  {
    // Grab the twist portion of the message and pass it to the twistCallback
    geometry_msgs::TwistWithCovarianceStamped *twistPtr = new geometry_msgs::TwistWithCovarianceStamped();
    twistPtr->header = msg->header;
    twistPtr->header.frame_id = msg->child_frame_id;
    twistPtr->twist = msg->twist;  // Entire twist object, also copies covariance

    geometry_msgs::TwistWithCovarianceStampedConstPtr tptr(twistPtr);
    twistCallback(tptr, twistCallbackData, baseLinkFrameId_);
  }

  RF_DEBUG("\n----- /RosFilter::odometryCallback (" << topicName << ") ------\n");
}
// */

// original imuCallback
// /*
template<typename T>
void FireflyRosFilter<T>::imuCallback(const sensor_msgs::Imu::ConstPtr &msg,   // in row 1261 a subscriber has used this callback function
                               const std::string &topicName,
                               const CallbackData &poseCallbackData,
                               const CallbackData &twistCallbackData,
                               const CallbackData &accelCallbackData)
{
  RF_DEBUG("------ RosFilter::imuCallback (" << topicName << ") ------\n" << "IMU message:\n" << *msg);

  // If we've just reset the filter, then we want to ignore any messages
  // that arrive with an older timestamp
  if (msg->header.stamp <= lastSetPoseTime_)
  {
    std::stringstream stream;
    stream << "The " << topicName << " message has a timestamp equal to or before the last filter reset, " <<
              "this message will be ignored. This may indicate an empty or bad timestamp. (message time: " <<
              msg->header.stamp.toSec() << ")";
    addDiagnostic(diagnostic_msgs::DiagnosticStatus::WARN,
                  topicName + "_timestamp",
                  stream.str(),
                  false);
    RF_DEBUG("Received message that preceded the most recent pose reset. Ignoring...");

    return;
  }

  // As with the odometry message, we can separate out the pose- and twist-related variables
  // in the IMU message and pass them to the pose and twist callbacks (filters)
  if (poseCallbackData.updateSum_ > 0)
  {
    // Per the IMU message specification, if the IMU does not provide orientation,
    // then its first covariance value should be set to -1, and we should ignore
    // that portion of the message. robot_localization allows users to explicitly
    // ignore data using its parameters, but we should also be compliant with
    // message specs.
    if(::fabs(msg->orientation_covariance[0] + 1) < 1e-9)
    {
      ROS_INFO("wtf");
      RF_DEBUG("Received IMU message with -1 as its first covariance value for orientation. "
               "Ignoring orientation...");
    }
    else
    {
      // Extract the pose (orientation) data, pass it to its filter
      geometry_msgs::PoseWithCovarianceStamped *posPtr = new geometry_msgs::PoseWithCovarianceStamped();
      posPtr->header = msg->header;
      posPtr->pose.pose.orientation = msg->orientation;

      // Copy the covariance for roll, pitch, and yaw
      for (size_t i = 0; i < ORIENTATION_SIZE; i++)
      {
        for (size_t j = 0; j < ORIENTATION_SIZE; j++)
        {
          posPtr->pose.covariance[POSE_SIZE * (i + ORIENTATION_SIZE) + (j + ORIENTATION_SIZE)] =
              msg->orientation_covariance[ORIENTATION_SIZE * i + j];
        }
      }

      // IMU data gets handled a bit differently, since the message is ambiguous and has only a single frame_id,
      // even though the data in it is reported in two different frames. As we assume users will specify a base_link
      // to imu transform, we make the target frame baseLinkFrameId_ and tell the poseCallback that it is working
      // with IMU data. This will cause it to apply different logic to the data.
      geometry_msgs::PoseWithCovarianceStampedConstPtr pptr(posPtr);

      // for test   by ZD
      /*
      tf2::Quaternion orientation;
      double roll, pitch, yaw;
      tf2::fromMsg(pptr->pose.pose.orientation, orientation);
      RosFilterUtilities::quatToRPY(orientation, roll, pitch, yaw);
      std::cout<<"msg imuCallback"<<std::endl
                       <<"roll:"<<roll<<", pitch:"<<pitch<<", yaw:"<<yaw<<std::endl;
      // */

      poseCallback(pptr, poseCallbackData, baseLinkFrameId_, true); // row 1550    by ZD
    }
  }

  if (twistCallbackData.updateSum_ > 0)
  {
    // Ignore rotational velocity if the first covariance value is -1
    if(::fabs(msg->angular_velocity_covariance[0] + 1) < 1e-9)
    {
      RF_DEBUG("Received IMU message with -1 as its first covariance value for angular "
               "velocity. Ignoring angular velocity...");
    }
    else
    {
      // Repeat for velocity
      geometry_msgs::TwistWithCovarianceStamped *twistPtr = new geometry_msgs::TwistWithCovarianceStamped();
      twistPtr->header = msg->header;
      twistPtr->twist.twist.angular = msg->angular_velocity;

      // Copy the covariance
      for (size_t i = 0; i < ORIENTATION_SIZE; i++)
      {
        for (size_t j = 0; j < ORIENTATION_SIZE; j++)
        {
          twistPtr->twist.covariance[TWIST_SIZE * (i + ORIENTATION_SIZE) + (j + ORIENTATION_SIZE)] =
            msg->angular_velocity_covariance[ORIENTATION_SIZE * i + j];
        }
      }

      geometry_msgs::TwistWithCovarianceStampedConstPtr tptr(twistPtr);
      twistCallback(tptr, twistCallbackData, baseLinkFrameId_);
    }
  }

  if (accelCallbackData.updateSum_ > 0)
  {
    // Ignore linear acceleration if the first covariance value is -1
    if(::fabs(msg->linear_acceleration_covariance[0] + 1) < 1e-9)
    {
      RF_DEBUG("Received IMU message with -1 as its first covariance value for linear "
               "acceleration. Ignoring linear acceleration...");
    }
    else
    {
      // Pass the message on
      accelerationCallback(msg, accelCallbackData, baseLinkFrameId_);
    }
  }

  RF_DEBUG("\n----- /RosFilter::imuCallback (" << topicName << ") ------\n");
}
// */

// original poseCallback
 /*
template<typename T>
void FireflyRosFilter<T>::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg,
                                const CallbackData &callbackData,
                                const std::string &targetFrame,
                                const bool imuData)
{
  const std::string &topicName = callbackData.topicName_;

  // If we've just reset the filter, then we want to ignore any messages
  // that arrive with an older timestamp
  if (msg->header.stamp <= lastSetPoseTime_)
  {
    std::stringstream stream;
    stream << "The " << topicName << " message has a timestamp equal to or before the last filter reset, " <<
              "this message will be ignored. This may indicate an empty or bad timestamp. (message time: " <<
              msg->header.stamp.toSec() << ")";
    addDiagnostic(diagnostic_msgs::DiagnosticStatus::WARN,
                  topicName + "_timestamp",
                  stream.str(),
                  false);
    return;
  }

  RF_DEBUG("------ FireflyRosFilter::poseCallback (" << topicName << ") ------\n" <<
           "Pose message:\n" << *msg);

  // Put the initial value in the lastMessagTimes_ for this variable if it's empty
  if (lastMessageTimes_.count(topicName) == 0)
  {
    lastMessageTimes_.insert(std::pair<std::string, ros::Time>(topicName, msg->header.stamp));
  }

  // Make sure this message is newer than the last one
  if (msg->header.stamp >= lastMessageTimes_[topicName])
  {
    RF_DEBUG("Update vector for " << topicName << " is:\n" << callbackData.updateVector_);

    Eigen::VectorXd measurement(STATE_SIZE);
    Eigen::MatrixXd measurementCovariance(STATE_SIZE, STATE_SIZE);

    measurement.setZero();
    measurementCovariance.setZero();

    // Make sure we're actually updating at least one of these variables
    std::vector<int> updateVectorCorrected = callbackData.updateVector_;

    // Prepare the pose data for inclusion in the filter
    if (preparePose(msg,  // row 2315  by ZD
                    topicName,
                    targetFrame,
                    callbackData.differential_,
                    callbackData.relative_,
                    imuData,
                    updateVectorCorrected,
                    measurement,
                    measurementCovariance))
    {
      // Store the measurement. Add a "pose" suffix so we know what kind of measurement
      // we're dealing with when we debug the core filter logic.
      enqueueMeasurement(topicName,
                         measurement,
                         measurementCovariance,
                         updateVectorCorrected,
                         callbackData.rejectionThreshold_,
                         msg->header.stamp);

      RF_DEBUG("Enqueued new measurement for " << topicName << "\n");
    }
    else
    {
      RF_DEBUG("Did *not* enqueue measurement for " << topicName << "\n");
    }

    lastMessageTimes_[topicName] = msg->header.stamp;

    RF_DEBUG("Last message time for " << topicName << " is now " <<
      lastMessageTimes_[topicName] << "\n");
  }
  else
  {
    std::stringstream stream;
    stream << "The " << topicName << " message has a timestamp before that of the previous message received," <<
              " this message will be ignored. This may indicate a bad timestamp. (message time: " <<
              msg->header.stamp.toSec() << ")";
    addDiagnostic(diagnostic_msgs::DiagnosticStatus::WARN,
                  topicName + "_timestamp",
                  stream.str(),
                  false);

    RF_DEBUG("Message is too old. Last message time for " << topicName << " is "
             << lastMessageTimes_[topicName] << ", current message time is "
             << msg->header.stamp << ".\n");
  }

  RF_DEBUG("\n----- /RosFilter::poseCallback (" << topicName << ") ------\n");
}
// */

// modified poseCallback
 // /*
template<typename T>
void FireflyRosFilter<T>::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg,
                                const CallbackData &callbackData,
                                const std::string &targetFrame,
                                const bool imuData)
{
  if (callbackData.topicName_ == "imu0_pose")
  {
    //std::cout<<callbackData.topicName_<<", poseCallback for imu0_pose, imuData:" <<imuData<<std::endl;
  }

  const std::string &topicName = callbackData.topicName_;

  // If we've just reset the filter, then we want to ignore any messages
  // that arrive with an older timestamp
  if (msg->header.stamp <= lastSetPoseTime_)
  {
    std::stringstream stream;
    stream << "The " << topicName << " message has a timestamp equal to or before the last filter reset, " <<
              "this message will be ignored. This may indicate an empty or bad timestamp. (message time: " <<
              msg->header.stamp.toSec() << ")";
    addDiagnostic(diagnostic_msgs::DiagnosticStatus::WARN,
                  topicName + "_timestamp",
                  stream.str(),
                  false);
    return;
  }

  RF_DEBUG("------ FireflyRosFilter::poseCallback (" << topicName << ") ------\n" <<
           "Pose message:\n" << *msg);

  // Put the initial value in the lastMessagTimes_ for this variable if it's empty
  if (lastMessageTimes_.count(topicName) == 0)
  {
    lastMessageTimes_.insert(std::pair<std::string, ros::Time>(topicName, msg->header.stamp));
  }

  // Make sure this message is newer than the last one
  if (msg->header.stamp >= lastMessageTimes_[topicName])
  {
    RF_DEBUG("Update vector for " << topicName << " is:\n" << callbackData.updateVector_);

    // Make sure we're actually updating at least one of these variables
    std::vector<int> updateVectorCorrected = callbackData.updateVector_;

    // Prepare the pose data for inclusion in the filter
    if (preparePose(msg,  // row 2315  by ZD
                    topicName,
                    targetFrame,
                    callbackData.differential_,
                    callbackData.relative_,
                    imuData,
                    updateVectorCorrected,
                    curMeasurement_.measurement_,
                    curMeasurement_.covariance_))
    {
      curMeasurement_.time_ = msg->header.stamp.toSec();
      // std::cout<<topicName<<" measurement x: "<<msg->pose.pose.position.x<<", y: "<<msg->pose.pose.position.y<<", z: "<<msg->pose.pose.position.z<<std::endl;
      // Store the measurement. Add a "pose" suffix so we know what kind of measurement
      // we're dealing with when we debug the core filter logic.
      enqueueMeasurement(topicName,
                         curMeasurement_.measurement_,
                         curMeasurement_.covariance_,
                         updateVectorCorrected,//curMeasurement_.updateVector_,
                         callbackData.rejectionThreshold_,
                         msg->header.stamp);

      RF_DEBUG("Enqueued new measurement for " << topicName << "\n");
    }
    else
    {
      RF_DEBUG("Did *not* enqueue measurement for " << topicName << "\n");
    }

    lastMessageTimes_[topicName] = msg->header.stamp;

    RF_DEBUG("Last message time for " << topicName << " is now " <<
      lastMessageTimes_[topicName] << "\n");
  }
  else
  {
    std::stringstream stream;
    stream << "The " << topicName << " message has a timestamp before that of the previous message received," <<
              " this message will be ignored. This may indicate a bad timestamp. (message time: " <<
              msg->header.stamp.toSec() << ")";
    addDiagnostic(diagnostic_msgs::DiagnosticStatus::WARN,
                  topicName + "_timestamp",
                  stream.str(),
                  false);

    RF_DEBUG("Message is too old. Last message time for " << topicName << " is "
             << lastMessageTimes_[topicName] << ", current message time is "
             << msg->header.stamp << ".\n");
  }

  RF_DEBUG("\n----- /RosFilter::poseCallback (" << topicName << ") ------\n");
}
// */

// original twistCallback
 /*
template<typename T>
void FireflyRosFilter<T>::twistCallback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr &msg,
                                 const CallbackData &callbackData,
                                 const std::string &targetFrame)
{
  const std::string &topicName = callbackData.topicName_;

  // If we've just reset the filter, then we want to ignore any messages
  // that arrive with an older timestamp
  if (msg->header.stamp <= lastSetPoseTime_)
  {
    std::stringstream stream;
    stream << "The " << topicName << " message has a timestamp equal to or before the last filter reset, " <<
              "this message will be ignored. This may indicate an empty or bad timestamp. (message time: " <<
              msg->header.stamp.toSec() << ")";
    addDiagnostic(diagnostic_msgs::DiagnosticStatus::WARN,
                  topicName + "_timestamp",
                  stream.str(),
                  false);
    return;
  }

  RF_DEBUG("------ FireflyRosFilter::twistCallback (" << topicName << ") ------\n"
           "Twist message:\n" << *msg);

  if (lastMessageTimes_.count(topicName) == 0)
  {
    lastMessageTimes_.insert(std::pair<std::string, ros::Time>(topicName, msg->header.stamp));
  }

  // Make sure this message is newer than the last one
  if (msg->header.stamp >= lastMessageTimes_[topicName])
  {
    RF_DEBUG("Update vector for " << topicName << " is:\n" << callbackData.updateVector_);

    Eigen::VectorXd measurement(STATE_SIZE);
    Eigen::MatrixXd measurementCovariance(STATE_SIZE, STATE_SIZE);

    measurement.setZero();
    measurementCovariance.setZero();

    // Make sure we're actually updating at least one of these variables
    std::vector<int> updateVectorCorrected = callbackData.updateVector_;

    // Prepare the twist data for inclusion in the filter
    if (prepareTwist(msg, topicName, targetFrame, updateVectorCorrected, measurement, measurementCovariance))
    {
      // Store the measurement. Add a "twist" suffix so we know what kind of measurement
      // we're dealing with when we debug the core filter logic.
      enqueueMeasurement(topicName,
                         measurement,
                         measurementCovariance,
                         updateVectorCorrected,
                         callbackData.rejectionThreshold_,
                         msg->header.stamp);

      RF_DEBUG("Enqueued new measurement for " << topicName << "_twist\n");
    }
    else
    {
      RF_DEBUG("Did *not* enqueue measurement for " << topicName << "_twist\n");
    }

    lastMessageTimes_[topicName] = msg->header.stamp;

    RF_DEBUG("Last message time for " << topicName << " is now " <<
      lastMessageTimes_[topicName] << "\n");
  }
  else
  {
    std::stringstream stream;
    stream << "The " << topicName << " message has a timestamp before that of the previous message received," <<
              " this message will be ignored. This may indicate a bad timestamp. (message time: " <<
              msg->header.stamp.toSec() << ")";
    addDiagnostic(diagnostic_msgs::DiagnosticStatus::WARN,
                  topicName + "_timestamp",
                  stream.str(),
                  false);

    RF_DEBUG("Message is too old. Last message time for " << topicName << " is " << lastMessageTimes_[topicName] <<
      ", current message time is " << msg->header.stamp << ".\n");
  }

  RF_DEBUG("\n----- /RosFilter::twistCallback (" << topicName << ") ------\n");
}
// */

// modified twistCallback
// /*
template<typename T>
void FireflyRosFilter<T>::twistCallback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr &msg,
                                 const CallbackData &callbackData,
                                 const std::string &targetFrame)
{
  const std::string &topicName = callbackData.topicName_;

  // If we've just reset the filter, then we want to ignore any messages
  // that arrive with an older timestamp
  if (msg->header.stamp <= lastSetPoseTime_)
  {
    std::stringstream stream;
    stream << "The " << topicName << " message has a timestamp equal to or before the last filter reset, " <<
              "this message will be ignored. This may indicate an empty or bad timestamp. (message time: " <<
              msg->header.stamp.toSec() << ")";
    addDiagnostic(diagnostic_msgs::DiagnosticStatus::WARN,
                  topicName + "_timestamp",
                  stream.str(),
                  false);
    return;
  }

  RF_DEBUG("------ FireflyRosFilter::twistCallback (" << topicName << ") ------\n"
           "Twist message:\n" << *msg);

  if (lastMessageTimes_.count(topicName) == 0)
  {
    lastMessageTimes_.insert(std::pair<std::string, ros::Time>(topicName, msg->header.stamp));
  }

  // Make sure this message is newer than the last one
  if (msg->header.stamp >= lastMessageTimes_[topicName])
  {
    RF_DEBUG("Update vector for " << topicName << " is:\n" << callbackData.updateVector_);

    // Make sure we're actually updating at least one of these variables
    std::vector<int> updateVectorCorrected = callbackData.updateVector_;

    // Prepare the twist data for inclusion in the filter
    if (prepareTwist(msg, topicName, targetFrame, updateVectorCorrected, curMeasurement_.measurement_, curMeasurement_.covariance_))
    {
      curMeasurement_.time_ = msg->header.stamp.toSec();
      // Store the measurement. Add a "twist" suffix so we know what kind of measurement
      // we're dealing with when we debug the core filter logic.
      enqueueMeasurement(topicName,
                         curMeasurement_.measurement_,
                         curMeasurement_.covariance_,
                         updateVectorCorrected,// curMeasurement_.updateVector_,
                         callbackData.rejectionThreshold_,
                         msg->header.stamp);

      RF_DEBUG("Enqueued new measurement for " << topicName << "_twist\n");
    }
    else
    {
      RF_DEBUG("Did *not* enqueue measurement for " << topicName << "_twist\n");
    }

    lastMessageTimes_[topicName] = msg->header.stamp;

    RF_DEBUG("Last message time for " << topicName << " is now " <<
      lastMessageTimes_[topicName] << "\n");
  }
  else
  {
    std::stringstream stream;
    stream << "The " << topicName << " message has a timestamp before that of the previous message received," <<
              " this message will be ignored. This may indicate a bad timestamp. (message time: " <<
              msg->header.stamp.toSec() << ")";
    addDiagnostic(diagnostic_msgs::DiagnosticStatus::WARN,
                  topicName + "_timestamp",
                  stream.str(),
                  false);

    RF_DEBUG("Message is too old. Last message time for " << topicName << " is " << lastMessageTimes_[topicName] <<
      ", current message time is " << msg->header.stamp << ".\n");
  }

  RF_DEBUG("\n----- /RosFilter::twistCallback (" << topicName << ") ------\n");
}
// */

// original accelerationCallback
 /*
template<typename T>
void FireflyRosFilter<T>::accelerationCallback(const sensor_msgs::Imu::ConstPtr &msg, const CallbackData &callbackData,
  const std::string &targetFrame)
{
  // If we've just reset the filter, then we want to ignore any messages
  // that arrive with an older timestamp
  if (msg->header.stamp <= lastSetPoseTime_)
  {
    return;
  }

  const std::string &topicName = callbackData.topicName_;

  RF_DEBUG("------ FireflyRosFilter::accelerationCallback (" << topicName << ") ------\n"
           "Twist message:\n" << *msg);

  if (lastMessageTimes_.count(topicName) == 0)
  {
    lastMessageTimes_.insert(std::pair<std::string, ros::Time>(topicName, msg->header.stamp));
  }

  // Make sure this message is newer than the last one
  if (msg->header.stamp >= lastMessageTimes_[topicName])
  {
    RF_DEBUG("Update vector for " << topicName << " is:\n" << topicName);

    Eigen::VectorXd measurement(STATE_SIZE);
    Eigen::MatrixXd measurementCovariance(STATE_SIZE, STATE_SIZE);

    measurement.setZero();
    measurementCovariance.setZero();

    // Make sure we're actually updating at least one of these variables
    std::vector<int> updateVectorCorrected = callbackData.updateVector_;

    // Prepare the twist data for inclusion in the filter
    if (prepareAcceleration(msg, topicName, targetFrame, updateVectorCorrected, measurement,
          measurementCovariance))        // From here the variable @p measurement and @p measurementCovariance will be calculated       by ZD
    {
      // Store the measurement. Add an "acceleration" suffix so we know what kind of measurement
      // we're dealing with when we debug the core filter logic.
      enqueueMeasurement(topicName,
                         measurement,
                         measurementCovariance,
                         updateVectorCorrected,
                         callbackData.rejectionThreshold_,
                         msg->header.stamp);

      RF_DEBUG("Enqueued new measurement for " << topicName << "_acceleration\n");
    }
    else
    {
      RF_DEBUG("Did *not* enqueue measurement for " << topicName << "_acceleration\n");
    }

    lastMessageTimes_[topicName] = msg->header.stamp;

    RF_DEBUG("Last message time for " << topicName << " is now " <<
      lastMessageTimes_[topicName] << "\n");
  }
  else
  {
    std::stringstream stream;
    stream << "The " << topicName << " message has a timestamp before that of the previous message received," <<
              " this message will be ignored. This may indicate a bad timestamp. (message time: " <<
              msg->header.stamp.toSec() << ")";
    addDiagnostic(diagnostic_msgs::DiagnosticStatus::WARN,
                  topicName + "_timestamp",
                  stream.str(),
                  false);

    RF_DEBUG("Message is too old. Last message time for " << topicName <<
             " is " << lastMessageTimes_[topicName] << ", current message time is " <<
             msg->header.stamp << ".\n");
  }

  RF_DEBUG("\n----- /RosFilter::accelerationCallback (" << topicName << ") ------\n");
}
// */

// modified accelerationCallback
// /*
template<typename T>
void FireflyRosFilter<T>::accelerationCallback(const sensor_msgs::Imu::ConstPtr &msg, const CallbackData &callbackData,
  const std::string &targetFrame)
{
  // If we've just reset the filter, then we want to ignore any messages
  // that arrive with an older timestamp
  if (msg->header.stamp <= lastSetPoseTime_)
  {
    return;
  }

  const std::string &topicName = callbackData.topicName_;

  RF_DEBUG("------ FireflyRosFilter::accelerationCallback (" << topicName << ") ------\n"
           "Twist message:\n" << *msg);

  if (lastMessageTimes_.count(topicName) == 0)
  {
    lastMessageTimes_.insert(std::pair<std::string, ros::Time>(topicName, msg->header.stamp));
  }

  // Make sure this message is newer than the last one
  if (msg->header.stamp >= lastMessageTimes_[topicName])
  {
    RF_DEBUG("Update vector for " << topicName << " is:\n" << topicName);

    // Make sure we're actually updating at least one of these variables
    std::vector<int> updateVectorCorrected = callbackData.updateVector_;

    // Prepare the twist data for inclusion in the filter
    if (prepareAcceleration(msg, topicName, targetFrame, updateVectorCorrected, curMeasurement_.measurement_,
           curMeasurement_.covariance_))        // From here the variable @p measurement and @p measurementCovariance will be calculated       by ZD
    {
      curMeasurement_.time_ = msg->header.stamp.toSec();
      // Store the measurement. Add an "acceleration" suffix so we know what kind of measurement
      // we're dealing with when we debug the core filter logic.
      enqueueMeasurement(topicName,
                         curMeasurement_.measurement_,
                         curMeasurement_.covariance_,
                         updateVectorCorrected,//curMeasurement_.updateVector_,
                         callbackData.rejectionThreshold_,
                         msg->header.stamp);


      RF_DEBUG("Enqueued new measurement for " << topicName << "_acceleration\n");
    }
    else
    {
      RF_DEBUG("Did *not* enqueue measurement for " << topicName << "_acceleration\n");
    }

    lastMessageTimes_[topicName] = msg->header.stamp;

    RF_DEBUG("Last message time for " << topicName << " is now " <<
      lastMessageTimes_[topicName] << "\n");
  }
  else
  {
    std::stringstream stream;
    stream << "The " << topicName << " message has a timestamp before that of the previous message received," <<
              " this message will be ignored. This may indicate a bad timestamp. (message time: " <<
              msg->header.stamp.toSec() << ")";
    addDiagnostic(diagnostic_msgs::DiagnosticStatus::WARN,
                  topicName + "_timestamp",
                  stream.str(),
                  false);

    RF_DEBUG("Message is too old. Last message time for " << topicName <<
             " is " << lastMessageTimes_[topicName] << ", current message time is " <<
             msg->header.stamp << ".\n");
  }

  RF_DEBUG("\n----- /RosFilter::accelerationCallback (" << topicName << ") ------\n");
}
// */

template<typename T>
void FireflyRosFilter<T>::setPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
  RF_DEBUG("------ FireflyRosFilter::setPoseCallback ------\nPose message:\n" << *msg);

  std::string topicName("setPose");

  // Get rid of any initial poses (pretend we've never had a measurement)
  initialMeasurements_.clear();
  previousMeasurements_.clear();
  previousMeasurementCovariances_.clear();

  // Clear out the measurement queue so that we don't immediately undo our
  // reset.
  while (!measurementQueue_.empty() && ros::ok())
  {
    measurementQueue_.pop(); // delete the first element
  }

  ros::getGlobalCallbackQueue()->clear();
  filterStateHistory_.clear();
  measurementHistory_.clear();

  // Also set the last set pose time, so we ignore all messages
  // that occur before it
  lastSetPoseTime_ = msg->header.stamp;

  // Set the state vector to the reported pose
  Eigen::VectorXd measurement(STATE_SIZE);
  Eigen::MatrixXd measurementCovariance(STATE_SIZE, STATE_SIZE);
  std::vector<int> updateVector(STATE_SIZE, true);

  // We only measure pose variables, so initialize the vector to 0
  measurement.setZero();

  // Set this to the identity and let the message reset it
  measurementCovariance.setIdentity();
  measurementCovariance *= 1e-6;

  // Prepare the pose data (really just using this to transform it into the target frame).
  // Twist data is going to get zeroed out.
  preparePose(msg, topicName, worldFrameId_, false, false, false, updateVector, measurement, measurementCovariance); // row 2315   by ZD

  // For the state
  filter_.setState(measurement);
  filter_.setEstimateErrorCovariance(measurementCovariance);

  filter_.setLastMeasurementTime(ros::Time::now().toSec());
  filter_.setLastUpdateTime(ros::Time::now().toSec());

  RF_DEBUG("\n------ /RosFilter::setPoseCallback ------\n");
}

template<typename T>
bool FireflyRosFilter<T>::setPoseSrvCallback(robot_localization::SetPose::Request& request,
                        robot_localization::SetPose::Response&)
{
  geometry_msgs::PoseWithCovarianceStamped::Ptr msg;
  msg = boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>(request.pose);
  setPoseCallback(msg);

  return true;
}

/////////////////////////////////////////////////////////////////////////////
/// Assistant functions
/////////////////////////////////////////////////////////////////////////////
template<typename T>
void FireflyRosFilter<T>::addDiagnostic(const int errLevel,
                                 const std::string &topicAndClass,
                                 const std::string &message,
                                 const bool staticDiag)
{
  if (staticDiag)
  {
    staticDiagnostics_[topicAndClass] = message;
    staticDiagErrorLevel_ = std::max(staticDiagErrorLevel_, errLevel);
  }
  else
  {
    dynamicDiagnostics_[topicAndClass] = message;
    dynamicDiagErrorLevel_ = std::max(dynamicDiagErrorLevel_, errLevel);
  }
}

template<typename T>
void FireflyRosFilter<T>::aggregateDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &wrapper)
{
  wrapper.clear();
  wrapper.clearSummary();

  int maxErrLevel = std::max(staticDiagErrorLevel_, dynamicDiagErrorLevel_);

  // Report the overall status
  switch (maxErrLevel)
  {
    case diagnostic_msgs::DiagnosticStatus::ERROR:
      wrapper.summary(maxErrLevel,
                      "Erroneous data or settings detected for a robot_localization state estimation node.");
      break;
    case diagnostic_msgs::DiagnosticStatus::WARN:
      wrapper.summary(maxErrLevel,
                      "Potentially erroneous data or settings detected for "
                      "a robot_localization state estimation node.");
      break;
    case diagnostic_msgs::DiagnosticStatus::STALE:
      wrapper.summary(maxErrLevel,
                      "The state of the robot_localization state estimation node is stale.");
      break;
    case diagnostic_msgs::DiagnosticStatus::OK:
      wrapper.summary(maxErrLevel,
                      "The robot_localization state estimation node appears to be functioning properly.");
      break;
    default:
      break;
  }

  // Aggregate all the static messages         from staticDiagnostics_ into wrapper  by ZD
  for (std::map<std::string, std::string>::iterator diagIt = staticDiagnostics_.begin();
      diagIt != staticDiagnostics_.end();
      ++diagIt)
  {
    wrapper.add(diagIt->first, diagIt->second);
  }

  // Aggregate all the dynamic messages, then clear them          aggregation is from dynamicDiagnostics_ into wrapper
  for (std::map<std::string, std::string>::iterator diagIt = dynamicDiagnostics_.begin();
      diagIt != dynamicDiagnostics_.end();
      ++diagIt)
  {
    wrapper.add(diagIt->first, diagIt->second);
  }
  dynamicDiagnostics_.clear();

  // Reset the warning level for the dynamic diagnostic messages
  dynamicDiagErrorLevel_ = diagnostic_msgs::DiagnosticStatus::OK;  // dynamicDiagnostics_ has been cleared, so its level is then of course set to OK
}

template<typename T>
void FireflyRosFilter<T>::copyCovariance(const double *arr,
                                  Eigen::MatrixXd &covariance,
                                  const std::string &topicName,
                                  const std::vector<int> &updateVector,
                                  const size_t offset,
                                  const size_t dimension)
{
  for (size_t i = 0; i < dimension; i++)
  {
    for (size_t j = 0; j < dimension; j++)
    {
      covariance(i, j) = arr[dimension * i + j];

      if (printDiagnostics_)
      {
        std::string iVar = stateVariableNames_[offset + i];

        if (covariance(i, j) > 1e3 && (updateVector[offset  + i] || updateVector[offset  + j]))
        {
          std::string jVar = stateVariableNames_[offset + j];

          std::stringstream stream;
          stream << "The covariance at position (" << dimension * i + j << "), which corresponds to " <<
              (i == j ? iVar + " variance" : iVar + " and " + jVar + " covariance") <<
              ", the value is extremely large (" << covariance(i, j) << "), but the update vector for " <<
              (i == j ? iVar : iVar + " and/or " + jVar) << " is set to true. This may produce undesirable results.";

          addDiagnostic(diagnostic_msgs::DiagnosticStatus::WARN,
                        topicName + "_covariance",
                        stream.str(),
                        false);
        }
        else if (updateVector[i] && i == j && covariance(i, j) == 0)
        {
          std::stringstream stream;
          stream << "The covariance at position (" << dimension * i + j << "), which corresponds to " <<
                     iVar << " variance, was zero. This will be replaced with a small value to maintain filter "
                     "stability, but should be corrected at the message origin node.";

          addDiagnostic(diagnostic_msgs::DiagnosticStatus::WARN,
                        topicName + "_covariance",
                        stream.str(),
                        false);
        }
        else if (updateVector[i] && i == j && covariance(i, j) < 0)
        {
          std::stringstream stream;
          stream << "The covariance at position (" << dimension * i + j <<
                    "), which corresponds to " << iVar << " variance, was negative. This will be replaced with a "
                    "small positive value to maintain filter stability, but should be corrected at the message "
                    "origin node.";

          addDiagnostic(diagnostic_msgs::DiagnosticStatus::WARN,
                        topicName + "_covariance",
                        stream.str(),
                        false);
        }
      }
    }
  }
}

template<typename T>
void FireflyRosFilter<T>::copyCovariance(const Eigen::MatrixXd &covariance,
                               double *arr,
                               const size_t dimension)
{
  for (size_t i = 0; i < dimension; i++)
  {
    for (size_t j = 0; j < dimension; j++)
    {
      arr[dimension * i + j] = covariance(i, j);
    }
  }
}

template<typename T>
void FireflyRosFilter<T>::enqueueMeasurement(const std::string &topicName,   // store the corresponding meesurements (input) into measurementQueue_    by ZD
                                      const Eigen::VectorXd &measurement,
                                      const Eigen::MatrixXd &measurementCovariance,
                                      const std::vector<int> &updateVector,
                                      const double mahalanobisThresh,
                                      const ros::Time &time)
{
  MeasurementPtr meas = MeasurementPtr(new Measurement());

  meas->topicName_ = topicName;
  meas->measurement_ = measurement;
  meas->covariance_ = measurementCovariance;
  meas->updateVector_ = updateVector;
  meas->time_ = time.toSec();
  meas->mahalanobisThresh_ = mahalanobisThresh;
  meas->latestControl_ = latestControl_;
  meas->latestControlTime_ = latestControlTime_.toSec();
  measurementQueue_.push(meas);
}

template<typename T>
void FireflyRosFilter<T>::forceTwoD(Eigen::VectorXd &measurement,    // In two D mode only what happens in x, y plane will be concerned   by ZD
                             Eigen::MatrixXd &measurementCovariance,
                             std::vector<int> &updateVector)
{
  measurement(StateMemberZ) = 0.0;
  measurement(StateMemberRoll) = 0.0;
  measurement(StateMemberPitch) = 0.0;
  measurement(StateMemberVz) = 0.0;
  measurement(StateMemberVroll) = 0.0;
  measurement(StateMemberVpitch) = 0.0;
  measurement(StateMemberAz) = 0.0;

  measurementCovariance(StateMemberZ, StateMemberZ) = 1e-6;
  measurementCovariance(StateMemberRoll, StateMemberRoll) = 1e-6;
  measurementCovariance(StateMemberPitch, StateMemberPitch) = 1e-6;
  measurementCovariance(StateMemberVz, StateMemberVz) = 1e-6;
  measurementCovariance(StateMemberVroll, StateMemberVroll) = 1e-6;
  measurementCovariance(StateMemberVpitch, StateMemberVpitch) = 1e-6;
  measurementCovariance(StateMemberAz, StateMemberAz) = 1e-6;

  updateVector[StateMemberZ] = 1;
  updateVector[StateMemberRoll] = 1;
  updateVector[StateMemberPitch] = 1;
  updateVector[StateMemberVz] = 1;
  updateVector[StateMemberVroll] = 1;
  updateVector[StateMemberVpitch] = 1;
  updateVector[StateMemberAz] = 1;
}

template<typename T>
bool FireflyRosFilter<T>::getFilteredOdometryMessage(nav_msgs::Odometry &message)  // get the inner odometry states of the filter and send them out in message form  by ZD
{
  // If the filter has received a measurement at some point...
  if (filter_.getInitializedStatus())
  {
    // Grab our current state and covariance estimates
    const Eigen::VectorXd &state = filter_.getState();
    const Eigen::MatrixXd &estimateErrorCovariance = filter_.getEstimateErrorCovariance();

    // Convert from roll, pitch, and yaw back to quaternion for
    // orientation values
    tf2::Quaternion quat;
    quat.setRPY(state(StateMemberRoll), state(StateMemberPitch), state(StateMemberYaw));

    // Fill out the message
    message.pose.pose.position.x = state(StateMemberX);
    message.pose.pose.position.y = state(StateMemberY);
    message.pose.pose.position.z = state(StateMemberZ);
    message.pose.pose.orientation.x = quat.x();
    message.pose.pose.orientation.y = quat.y();
    message.pose.pose.orientation.z = quat.z();
    message.pose.pose.orientation.w = quat.w();
    message.twist.twist.linear.x = state(StateMemberVx);
    message.twist.twist.linear.y = state(StateMemberVy);
    message.twist.twist.linear.z = state(StateMemberVz);
    message.twist.twist.angular.x = state(StateMemberVroll);
    message.twist.twist.angular.y = state(StateMemberVpitch);
    message.twist.twist.angular.z = state(StateMemberVyaw);

    // Our covariance matrix layout doesn't quite match
    for (size_t i = 0; i < POSE_SIZE; i++)
    {
      for (size_t j = 0; j < POSE_SIZE; j++)
      {
        message.pose.covariance[POSE_SIZE * i + j] = estimateErrorCovariance(i, j);
      }
    }

    // POSE_SIZE and TWIST_SIZE are currently the same size, but we can spare a few
    // cycles to be meticulous and not index a twist covariance array on the size of
    // a pose covariance array
    for (size_t i = 0; i < TWIST_SIZE; i++)
    {
      for (size_t j = 0; j < TWIST_SIZE; j++)
      {
        message.twist.covariance[TWIST_SIZE * i + j] =
            estimateErrorCovariance(i + POSITION_V_OFFSET, j + POSITION_V_OFFSET);
      }
    }

    message.header.stamp = ros::Time(filter_.getLastMeasurementTime());
    message.header.frame_id = worldFrameId_;
    message.child_frame_id = baseLinkFrameId_;
  }

  return filter_.getInitializedStatus();
}

// original getFilteredAccelMessage
/*
template<typename T>
bool FireflyRosFilter<T>::getFilteredAccelMessage(geometry_msgs::AccelWithCovarianceStamped &message) // get filtered acceleration from inner states and send it out in message form   by ZD
{
  // If the filter has received a measurement at some point...
  if (filter_.getInitializedStatus())
  {
    // Grab our current state and covariance estimates
    const Eigen::VectorXd &state = filter_.getState();
    const Eigen::MatrixXd &estimateErrorCovariance = filter_.getEstimateErrorCovariance();

    //! Fill out the accel_msg
    message.accel.accel.linear.x = state(StateMemberAx);
    message.accel.accel.linear.y = state(StateMemberAy);
    message.accel.accel.linear.z = state(StateMemberAz);

    // Fill the covariance (only the left-upper matrix since we are not estimating
    // the rotational accelerations arround the axes
    for (size_t i = 0; i < ACCELERATION_SIZE; i++)
    {
      for (size_t j = 0; j < ACCELERATION_SIZE; j++)
      {
        // We use the POSE_SIZE since the accel cov matrix of ROS is 6x6
        message.accel.covariance[POSE_SIZE * i + j] =
            estimateErrorCovariance(i + POSITION_A_OFFSET, j + POSITION_A_OFFSET);
      }
    }

    // Fill header information
    message.header.stamp = ros::Time(filter_.getLastMeasurementTime());
    message.header.frame_id = baseLinkFrameId_;
  }

  return filter_.getInitializedStatus();
}
// */

// modified getFilteredAccelMessage
// /*
template<typename T>
bool FireflyRosFilter<T>::getFilteredAccelMessage(geometry_msgs::PointStamped &message) // get filtered acceleration from inner states and send it out in message form   by ZD
{
  // If the filter has received a measurement at some point...
  if (filter_.getInitializedStatus())
  {
    // Grab our current state and covariance estimates
    const Eigen::VectorXd &state = filter_.getState();
    const Eigen::MatrixXd &estimateErrorCovariance = filter_.getEstimateErrorCovariance();

    /*
    tf2::Vector3 normAcc(0, 0, 9.80665);
    tf2::Quaternion curAttitude;
    tf2::Transform trans;

    tf2::Vector3 stateTmp(state(StateMemberRoll),
                                                 state(StateMemberPitch),
                                                 state(StateMemberYaw));
    curAttitude.setRPY(stateTmp.getX(), stateTmp.getY(), stateTmp.getZ());
    trans.setRotation(curAttitude);
    tf2::Vector3 rotNorm = trans.getBasis().inverse() * normAcc;
    */

    //double p = state(9);
    //double q = state(10);
    //double r = state(11);

    //! Fill out the accel_msg
    message.point.x = state(StateMemberAx); //+ rotNorm.getX(); //- 9.81*sin(state(9));
    message.point.y = state(StateMemberAy); //+ rotNorm.getY(); //+ 9.81*cos(state(9))*sin(state(11));
    message.point.z = state(StateMemberAz); // + rotNorm.getZ(); //+ 9.81*cos(state(9))*cos(state(11));

    // Fill the covariance (only the left-upper matrix since we are not estimating
    // the rotational accelerations arround the axes
    /*
    for (size_t i = 0; i < ACCELERATION_SIZE; i++)
    {
      for (size_t j = 0; j < ACCELERATION_SIZE; j++)
      {
        // We use the POSE_SIZE since the accel cov matrix of ROS is 6x6
        message.accel.covariance[POSE_SIZE * i + j] =
            estimateErrorCovariance(i + POSITION_A_OFFSET, j + POSITION_A_OFFSET);
      }
    } */

    // Fill header information
    message.header.stamp = ros::Time(filter_.getLastMeasurementTime());
    message.header.frame_id = baseLinkFrameId_;
  }

  return filter_.getInitializedStatus();
}
// */

template<typename T>
std::vector<int> FireflyRosFilter<T>::loadUpdateConfig(const std::string &topicName)
{
  XmlRpc::XmlRpcValue topicConfig;  // XML remote procedure call, which means using xml to interpret the normal data type
  std::vector<int> updateVector(STATE_SIZE, 0);
  std::string topicConfigName = topicName + "_config";

  try
  {
    nhLocal_.getParam(topicConfigName, topicConfig);

    ROS_ASSERT(topicConfig.getType() == XmlRpc::XmlRpcValue::TypeArray);

    if (topicConfig.size() != STATE_SIZE)
    {
      ROS_WARN_STREAM("Configuration vector for " << topicConfigName << " should have 15 entries.");
    }

    for (int i = 0; i < topicConfig.size(); i++)
    {
      // The double cast looks strange, but we'll get exceptions if we
      // remove the cast to bool. vector<bool> is discouraged, so updateVector
      // uses integers.
      updateVector[i] = static_cast<int>(static_cast<bool>(topicConfig[i]));
    }
  }
  catch (XmlRpc::XmlRpcException &e) // included in XmlRpcException.h in ros_filter.h
  {
    ROS_FATAL_STREAM("Could not read sensor update configuration for topic " << topicName <<
                     " (type: " << topicConfig.getType() << ", expected: " << XmlRpc::XmlRpcValue::TypeArray
                     << "). Error was " << e.getMessage() << "\n");
  }

  return updateVector;
}

template<typename T>
bool FireflyRosFilter<T>::prepareImu(const sensor_msgs::Imu::ConstPtr &msg,
                                                                           const std::string &topicName,
                                                                           const std::string &targetFrame,
                                                                           std::vector<int> &updateVector,
                                                                           Eigen::VectorXd &measurement,
                                                                           Eigen::MatrixXd &measurementCovariance)
{
  //bool retVal = false;

  //RF_DEBUG("------ FireflyRosFilter::prepareImu (" << topicName << ") ------\n");
}

// original preparePose
 /*
template<typename T>
bool FireflyRosFilter<T>::preparePose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg,  // save transform the pose msg into target frame and output as measurement and measurementCovariance   by ZD
                               const std::string &topicName,
                               const std::string &targetFrame,
                               const bool differential,
                               const bool relative,
                               const bool imuData,
                               std::vector<int> &updateVector,
                               Eigen::VectorXd &measurement,
                               Eigen::MatrixXd &measurementCovariance)
{
  bool retVal = false;

  RF_DEBUG("------ FireflyRosFilter::preparePose (" << topicName << ") ------\n");

  // 1. Get the measurement into a tf-friendly transform (pose) object
  tf2::Stamped<tf2::Transform> poseTmp;

  // We'll need this later for storing this measurement for differential integration
  tf2::Transform curMeasurement;

  // Handle issues where frame_id data is not filled out properly
  // @todo: verify that this is necessary still. New IMU handling may
  // have rendered this obsolete.
  std::string finalTargetFrame;
  if (targetFrame == "" && msg->header.frame_id == "")
  {
    // Blank target and message frames mean we can just
    // use our world_frame
    finalTargetFrame = worldFrameId_;
    poseTmp.frame_id_ = finalTargetFrame;
  }
  else if (targetFrame == "")
  {
    // A blank target frame means we shouldn't bother
    // transforming the data
    finalTargetFrame = msg->header.frame_id;
    poseTmp.frame_id_ = finalTargetFrame;
  }
  else
  {
    // Otherwise, we should use our target frame
    finalTargetFrame = targetFrame;
    poseTmp.frame_id_ = (differential ? finalTargetFrame : msg->header.frame_id);
  }

  RF_DEBUG("Final target frame for " << topicName << " is " << finalTargetFrame << "\n");

  poseTmp.stamp_ = msg->header.stamp;

  // Fill out the position data
  poseTmp.setOrigin(tf2::Vector3(msg->pose.pose.position.x,
                                 msg->pose.pose.position.y,
                                 msg->pose.pose.position.z));

  tf2::Quaternion orientation;

  // Handle bad (empty) quaternions
  if (msg->pose.pose.orientation.x == 0 && msg->pose.pose.orientation.y == 0 &&
     msg->pose.pose.orientation.z == 0 && msg->pose.pose.orientation.w == 0)
  {
    orientation.setValue(0.0, 0.0, 0.0, 1.0);

    if (updateVector[StateMemberRoll] || updateVector[StateMemberPitch] || updateVector[StateMemberYaw])
    {
      std::stringstream stream;
      stream << "The " << topicName << " message contains an invalid orientation quaternion, " <<
                "but its configuration is such that orientation data is being used. Correcting...";

      addDiagnostic(diagnostic_msgs::DiagnosticStatus::WARN,
                    topicName + "_orientation",
                    stream.str(),
                    false);
    }
  }
  else
  {
    tf2::fromMsg(msg->pose.pose.orientation, orientation);
  }

  // Fill out the orientation data
  poseTmp.setRotation(orientation);

  // 2. Get the target frame transformation
  tf2::Transform targetFrameTrans;
  bool canTransform = RosFilterUtilities::lookupTransformSafe(tfBuffer_,  // ros_filter_utilities.h row 78, obtaining a transform from sourceFrame to targetFrame
                                                              finalTargetFrame,
                                                              poseTmp.frame_id_,
                                                              poseTmp.stamp_,
                                                              targetFrameTrans);

  // 3. Make sure we can work with this data before carrying on
  if (canTransform)
  {
    // 4. robot_localization lets users configure which variables from the sensor should be
    //     fused with the filter. This is specified at the sensor level. However, the data
    //     may go through transforms before being fused with the state estimate. In that case,
    //     we need to know which of the transformed variables came from the pre-transformed
    //     "approved" variables (i.e., the ones that had "true" in their xxx_config parameter).
    //     To do this, we construct matrices using the update vector values on the diagonals,
    //     pass this matrix through the rotation, and use the length of each row to determine
    //     the transformed update vector. The process is slightly different for IMUs, as the
    //     coordinate frame transform is really the base_link->imu_frame transform, and not
    //     a transform from some other world-fixed frame (even though the IMU data itself *is*
    //     reported in a world fixed frame).
    tf2::Matrix3x3 maskPosition(updateVector[StateMemberX], 0, 0,
                                0, updateVector[StateMemberY], 0,
                                0, 0, updateVector[StateMemberZ]);

    tf2::Matrix3x3 maskOrientation(updateVector[StateMemberRoll], 0, 0,
                                   0, updateVector[StateMemberPitch], 0,
                                   0, 0, updateVector[StateMemberYaw]);

    if (imuData)
    {
      // We have to treat IMU orientation data differently. Even though we are dealing with pose
      //  data when we work with orientations, for IMUs, the frame_id is the frame in which the
      //  sensor is mounted, and not the coordinate frame of the IMU. Imagine an IMU that is mounted
      //  facing sideways. The pitch in the IMU frame becomes roll for the vehicle. This means that
      //  we need to rotate roll and pitch angles by the IMU's mounting yaw offset, and we must apply
      //  similar treatment to its update mask and covariance.

      double dummy, yaw;
      targetFrameTrans.getBasis().getRPY(dummy, dummy, yaw);
      tf2::Matrix3x3 transTmp;
      transTmp.setRPY(0.0, 0.0, yaw);

      maskPosition = transTmp * maskPosition;
      maskOrientation = transTmp * maskOrientation;
    }
    else
    {
      maskPosition = targetFrameTrans.getBasis() * maskPosition; // getBasis returns the matrix, while getRotation returns the quaternion  by ZD
      maskOrientation = targetFrameTrans.getBasis() * maskOrientation;
    }

    // Now copy the mask values back into the update vector: any row with a significant vector length
    // indicates that we want to set that variable to true in the update vector.
    updateVector[StateMemberX] = static_cast<int>(
      maskPosition.getRow(StateMemberX - POSITION_OFFSET).length() >= 1e-6);
    updateVector[StateMemberY] = static_cast<int>(
      maskPosition.getRow(StateMemberY - POSITION_OFFSET).length() >= 1e-6);
    updateVector[StateMemberZ] = static_cast<int>(
      maskPosition.getRow(StateMemberZ - POSITION_OFFSET).length() >= 1e-6);
    updateVector[StateMemberRoll] = static_cast<int>(
      maskOrientation.getRow(StateMemberRoll - ORIENTATION_OFFSET).length() >= 1e-6);
    updateVector[StateMemberPitch] = static_cast<int>(
      maskOrientation.getRow(StateMemberPitch - ORIENTATION_OFFSET).length() >= 1e-6);
    updateVector[StateMemberYaw] = static_cast<int>(
      maskOrientation.getRow(StateMemberYaw - ORIENTATION_OFFSET).length() >= 1e-6);

    // 5a. We'll need to rotate the covariance as well. Create a container and copy over the
    // covariance data
    Eigen::MatrixXd covariance(POSE_SIZE, POSE_SIZE);
    covariance.setZero();
    copyCovariance(&(msg->pose.covariance[0]), covariance, topicName, updateVector, POSITION_OFFSET, POSE_SIZE);

    // 5b. Now rotate the covariance: create an augmented matrix that
    // contains a 3D rotation matrix in the upper-left and lower-right
    // quadrants, with zeros elsewhere.
    tf2::Matrix3x3 rot;
    Eigen::MatrixXd rot6d(POSE_SIZE, POSE_SIZE);
    rot6d.setIdentity();
    Eigen::MatrixXd covarianceRotated;

    if (imuData)
    {
      // Apply the same special logic to the IMU covariance rotation
      double dummy, yaw;
      targetFrameTrans.getBasis().getRPY(dummy, dummy, yaw);
      rot.setRPY(0.0, 0.0, yaw);  // which means imus worlds orientation is the orientation of the imu at time 0     by ZD
    }
    else
    {
      rot.setRotation(targetFrameTrans.getRotation()); // quaternion to matrix, this rot is used to calculate the rot6d for covarianceRotated   by ZD
    }

    for (size_t rInd = 0; rInd < POSITION_SIZE; ++rInd)
    {
      rot6d(rInd, 0) = rot.getRow(rInd).getX();
      rot6d(rInd, 1) = rot.getRow(rInd).getY();
      rot6d(rInd, 2) = rot.getRow(rInd).getZ();
      rot6d(rInd+POSITION_SIZE, 3) = rot.getRow(rInd).getX();
      rot6d(rInd+POSITION_SIZE, 4) = rot.getRow(rInd).getY();
      rot6d(rInd+POSITION_SIZE, 5) = rot.getRow(rInd).getZ();
    }

    // Now carry out the rotation
    covarianceRotated = rot6d * covariance * rot6d.transpose();

    RF_DEBUG("After rotating into the " << finalTargetFrame <<
             " frame, covariance is \n" << covarianceRotated <<  "\n");

    // 6a. For IMU data, the transform that we get is the transform from the body
    //  frame of the robot (e.g., base_link) to the mounting frame of the robot. It
    //  is *not* the coordinate frame in which the IMU orientation data is reported.
    //  If the IMU is mounted in a non-neutral orientation, we need to remove those
    //  offsets, and then we need to potentially "swap" roll and pitch.
    //  Note that this transform does NOT handle NED->ENU conversions. Data is assumed
    //  to be in the ENU frame when it is received.
    //
    if (imuData)
    {
      // First, convert the transform and measurement rotation to RPY
      // @todo: There must be a way to handle this with quaternions. Need to look into it.
      double rollOffset = 0;
      double pitchOffset = 0;
      double yawOffset = 0;
      double roll = 0;
      double pitch = 0;
      double yaw = 0;
      RosFilterUtilities::quatToRPY(targetFrameTrans.getRotation(), rollOffset, pitchOffset, yawOffset); // this roll pitch yaw offsets are roll pitch yaw of the imu frame   by ZD
      RosFilterUtilities::quatToRPY(poseTmp.getRotation(), roll, pitch, yaw);  // poseTmp saves the pose from the msg sent from callback function by ZD

      // 6b. Apply the offset (making sure to bound them), and throw them in a vector
      tf2::Vector3 rpyAngles(FilterUtilities::clampRotation(roll - rollOffset),
                             FilterUtilities::clampRotation(pitch - pitchOffset),
                             FilterUtilities::clampRotation(yaw - yawOffset));

      // 6c. Now we need to rotate the roll and pitch by the yaw offset value.
      // Imagine a case where an IMU is mounted facing sideways. In that case
      // pitch for the IMU's world frame is roll for the robot.
      tf2::Matrix3x3 mat;
      mat.setRPY(0.0, 0.0, yawOffset);
      rpyAngles = mat * rpyAngles;
      poseTmp.getBasis().setRPY(rpyAngles.getX(), rpyAngles.getY(), rpyAngles.getZ());

      // We will use this target transformation later on, but
      // we've already transformed this data as if the IMU
      // were mounted neutrall on the robot, so we can just
      // make the transform the identity.
      targetFrameTrans.setIdentity();
    }

    // 7. Two cases: if we're in differential mode, we need to generate a twist
    // message. Otherwise, we just transform it to the target frame.
    if (differential)
    {
      bool success = false;

      // We're going to be playing with poseTmp, so store it,
      // as we'll need to save its current value for the next
      // measurement.
      curMeasurement = poseTmp;

      // Make sure we have previous measurements to work with
      if (previousMeasurements_.count(topicName) > 0 && previousMeasurementCovariances_.count(topicName) > 0)
      {
        // 7a. If we are carrying out differential integration and
        // we have a previous measurement for this sensor,then we
        // need to apply the inverse of that measurement to this new
        // measurement to produce a "delta" measurement between the two.
        // Even if we're not using all of the variables from this sensor,
        // we need to use the whole measurement to determine the delta
        // to the new measurement
        tf2::Transform prevMeasurement = previousMeasurements_[topicName];
        poseTmp.setData(prevMeasurement.inverseTimes(poseTmp)); // inverseTimes = inverse this matrix times another, now this poseTmp is the delta we need     by ZD

        RF_DEBUG("Previous measurement:\n" << previousMeasurements_[topicName] <<
                 "\nAfter removing previous measurement, measurement delta is:\n" << poseTmp << "\n");

        // 7b. Now we we have a measurement delta in the frame_id of the
        // message, but we want that delta to be in the target frame, so
        // we need to apply the rotation of the target frame transform.
        targetFrameTrans.setOrigin(tf2::Vector3(0.0, 0.0, 0.0)); // setOrigin = set the translational vector, which means only the orientation will be changed   by ZD
        poseTmp.mult(targetFrameTrans, poseTmp); // poseTmp = targetFrameTrans*poseTmp   , now targetFrameTrans only has orientational part  by ZD

        RF_DEBUG("After rotating to the target frame, measurement delta is:\n" << poseTmp << "\n");

        // 7c. Now use the time difference from the last message to compute
        // translational and rotational velocities
        double dt = msg->header.stamp.toSec() - lastMessageTimes_[topicName].toSec();
        double xVel = poseTmp.getOrigin().getX() / dt;
        double yVel = poseTmp.getOrigin().getY() / dt;
        double zVel = poseTmp.getOrigin().getZ() / dt;  // calculate the odometry velocities using differential method    by ZD

        double rollVel = 0;
        double pitchVel = 0;
        double yawVel = 0;

        RosFilterUtilities::quatToRPY(poseTmp.getRotation(), rollVel, pitchVel, yawVel);
        rollVel /= dt;
        pitchVel /= dt;
        yawVel /= dt;  // calculate the odometry attitude velocities (NOT body angular velocities) using differential method     by ZD

        RF_DEBUG("Previous message time was " << lastMessageTimes_[topicName].toSec() <<
                 ", current message time is " << msg->header.stamp.toSec() << ", delta is " <<
                 dt << ", velocity is (vX, vY, vZ): (" << xVel << ", " << yVel << ", " << zVel <<
                 ")\n" << "(vRoll, vPitch, vYaw): (" << rollVel << ", " << pitchVel << ", " <<
                 yawVel << ")\n");

        // 7d. Fill out the velocity data in the message
        geometry_msgs::TwistWithCovarianceStamped *twistPtr = new geometry_msgs::TwistWithCovarianceStamped();
        twistPtr->header = msg->header;
        twistPtr->header.frame_id = baseLinkFrameId_;
        twistPtr->twist.twist.linear.x = xVel;
        twistPtr->twist.twist.linear.y = yVel;
        twistPtr->twist.twist.linear.z = zVel;
        twistPtr->twist.twist.angular.x = rollVel;
        twistPtr->twist.twist.angular.y = pitchVel;
        twistPtr->twist.twist.angular.z = yawVel;
        std::vector<int> twistUpdateVec(STATE_SIZE, false);
        std::copy(updateVector.begin() + POSITION_OFFSET,  // while (first!=last), *result=*first, ++result,++first  by ZD
                  updateVector.begin() + POSE_SIZE,
                  twistUpdateVec.begin() + POSITION_V_OFFSET); // twistUpdateVec = f f f f f f t t t t t t f f f    by ZD
        std::copy(twistUpdateVec.begin(), twistUpdateVec.end(), updateVector.begin());  // updateVector = twistUpdateVec  by ZD
        geometry_msgs::TwistWithCovarianceStampedConstPtr ptr(twistPtr);

        // 7e. Now rotate the previous covariance for this measurement to get it
        // into the target frame, and add the current measurement's rotated covariance
        // to the previous measurement's rotated covariance, and multiply by the time delta.
        Eigen::MatrixXd prevCovarRotated = rot6d * previousMeasurementCovariances_[topicName] * rot6d.transpose();
        covarianceRotated = (covarianceRotated.eval() + prevCovarRotated) * dt;  // why ? I think here should be /dt    by ZD
        copyCovariance(covarianceRotated, &(twistPtr->twist.covariance[0]), POSE_SIZE);

        RF_DEBUG("Previous measurement covariance:\n" << previousMeasurementCovariances_[topicName] <<
                 "\nPrevious measurement covariance rotated:\n" << prevCovarRotated <<
                 "\nFinal twist covariance:\n" << covarianceRotated << "\n");

        // Now pass this on to prepareTwist, which will convert it to the required frame
        success = prepareTwist(ptr,
                               topicName + "_twist",
                               twistPtr->header.frame_id,
                               updateVector,
                               measurement,   // it seems prepareTwist only modify twist part of the measurement  by ZD
                               measurementCovariance);
      }

      // 7f. Update the previous measurement and measurement covariance
      previousMeasurements_[topicName] = curMeasurement;
      previousMeasurementCovariances_[topicName] = covariance;

      retVal = success;
    }
    else
    {
      // 7g. If we're in relative mode, remove the initial measurement
      if (relative)
      {
        if (initialMeasurements_.count(topicName) == 0)
        {
          initialMeasurements_.insert(std::pair<std::string, tf2::Transform>(topicName, poseTmp));
        }

        tf2::Transform initialMeasurement = initialMeasurements_[topicName];
        poseTmp.setData(initialMeasurement.inverseTimes(poseTmp));
      }

      // 7h. Apply the target frame transformation to the pose object.
      poseTmp.mult(targetFrameTrans, poseTmp);
      poseTmp.frame_id_ = finalTargetFrame;

      // 7i. Finally, copy everything into our measurement and covariance objects
      measurement(StateMemberX) = poseTmp.getOrigin().x();
      measurement(StateMemberY) = poseTmp.getOrigin().y();
      measurement(StateMemberZ) = poseTmp.getOrigin().z();

      // The filter needs roll, pitch, and yaw values instead of quaternions
      double roll, pitch, yaw;
      RosFilterUtilities::quatToRPY(poseTmp.getRotation(), roll, pitch, yaw);
      measurement(StateMemberRoll) = roll;
      measurement(StateMemberPitch) = pitch;
      measurement(StateMemberYaw) = yaw;

      measurementCovariance.block(0, 0, POSE_SIZE, POSE_SIZE) = covarianceRotated.block(0, 0, POSE_SIZE, POSE_SIZE);

      // 8. Handle 2D mode
      if (twoDMode_)
      {
        forceTwoD(measurement, measurementCovariance, updateVector); // row 228
      }

      retVal = true;
    }
  }
  else // if (canTransform = false)     by ZD
  {
    retVal = false;

    RF_DEBUG("Could not transform measurement into " << finalTargetFrame << ". Ignoring...");
  }

  RF_DEBUG("\n----- /FireflyRosFilter::preparePose (" << topicName << ") ------\n");

  return retVal;
}
// */

// modified preparePose
// /*
template<typename T>
bool FireflyRosFilter<T>::preparePose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg,  // save transform the pose msg into target frame and output as measurement and measurementCovariance   by ZD
                              const std::string &topicName,
                              const std::string &targetFrame,
                              const bool differential,
                              const bool relative,
                              const bool imuData,
                              std::vector<int> &updateVector,
                              Eigen::VectorXd &measurement,
                              Eigen::MatrixXd &measurementCovariance)
{
  /* for test   by ZD
  if (topicName == "imu0_pose")
  {
    tf2::Quaternion orientation1;
    double roll1, pitch1, yaw1;
    tf2::fromMsg(msg->pose.pose.orientation, orientation1);
    RosFilterUtilities::quatToRPY(orientation1, roll1, pitch1, yaw1);
    std::cout<<topicName<<std::endl
                     <<"vor roll:"<<roll1<<", pitch:"<<pitch1<<", yaw:"<<yaw1<<std::endl;
  }
  // */

 bool retVal = false;

 RF_DEBUG("------ FireflyRosFilter::preparePose (" << topicName << ") ------\n");

 // 1. Get the measurement into a tf-friendly transform (pose) object
 tf2::Stamped<tf2::Transform> poseTmp;

 // We'll need this later for storing this measurement for differential integration
 tf2::Transform curMeasurement;

 // Handle issues where frame_id data is not filled out properly
 // @todo: verify that this is necessary still. New IMU handling may
 // have rendered this obsolete.
 std::string finalTargetFrame;
 if (targetFrame == "" && msg->header.frame_id == "")
 {
   // Blank target and message frames mean we can just
   // use our world_frame
   finalTargetFrame = worldFrameId_;
   poseTmp.frame_id_ = finalTargetFrame;
 }
 else if (targetFrame == "")
 {
   // A blank target frame means we shouldn't bother
   // transforming the data
   finalTargetFrame = msg->header.frame_id;
   poseTmp.frame_id_ = finalTargetFrame;
 }
 else
 {
   // Otherwise, we should use our target frame
   finalTargetFrame = targetFrame;
   poseTmp.frame_id_ = (differential ? finalTargetFrame : msg->header.frame_id);
 }

 RF_DEBUG("Final target frame for " << topicName << " is " << finalTargetFrame << "\n");

 poseTmp.stamp_ = msg->header.stamp;

 // Fill out the position data
 poseTmp.setOrigin(tf2::Vector3(msg->pose.pose.position.x,
                                msg->pose.pose.position.y,
                                msg->pose.pose.position.z));

 tf2::Quaternion orientation;

 // Handle bad (empty) quaternions
 if (msg->pose.pose.orientation.x == 0 && msg->pose.pose.orientation.y == 0 &&
    msg->pose.pose.orientation.z == 0 && msg->pose.pose.orientation.w == 0)
 {
   orientation.setValue(0.0, 0.0, 0.0, 1.0);

   if (updateVector[StateMemberRoll] || updateVector[StateMemberPitch] || updateVector[StateMemberYaw])
   {
     std::stringstream stream;
     stream << "The " << topicName << " message contains an invalid orientation quaternion, " <<
               "but its configuration is such that orientation data is being used. Correcting...";

     addDiagnostic(diagnostic_msgs::DiagnosticStatus::WARN,
                   topicName + "_orientation",
                   stream.str(),
                   false);
   }
 }
 else
 {
   tf2::fromMsg(msg->pose.pose.orientation, orientation);
 }

 // Fill out the orientation data
 poseTmp.setRotation(orientation);

 // 2. Get the target frame transformation
 tf2::Transform targetFrameTrans;
 bool canTransform = RosFilterUtilities::lookupTransformSafe(tfBuffer_,  // ros_filter_utilities.h row 78, obtaining a transform from sourceFrame to targetFrame
                                                             finalTargetFrame,
                                                             poseTmp.frame_id_,
                                                             poseTmp.stamp_,
                                                             targetFrameTrans);

 // 3. Make sure we can work with this data before carrying on
 if (canTransform)
 {
   // 4. robot_localization lets users configure which variables from the sensor should be
   //     fused with the filter. This is specified at the sensor level. However, the data
   //     may go through transforms before being fused with the state estimate. In that case,
   //     we need to know which of the transformed variables came from the pre-transformed
   //     "approved" variables (i.e., the ones that had "true" in their xxx_config parameter).
   //     To do this, we construct matrices using the update vector values on the diagonals,
   //     pass this matrix through the rotation, and use the length of each row to determine
   //     the transformed update vector. The process is slightly different for IMUs, as the
   //     coordinate frame transform is really the base_link->imu_frame transform, and not
   //     a transform from some other world-fixed frame (even though the IMU data itself *is*
   //     reported in a world fixed frame).
   tf2::Matrix3x3 maskPosition(updateVector[StateMemberX], 0, 0,
                               0, updateVector[StateMemberY], 0,
                               0, 0, updateVector[StateMemberZ]);

   tf2::Matrix3x3 maskOrientation(updateVector[StateMemberRoll], 0, 0,
                                  0, updateVector[StateMemberPitch], 0,
                                  0, 0, updateVector[StateMemberYaw]);

   if (imuData)
   {
     // We have to treat IMU orientation data differently. Even though we are dealing with pose
     //  data when we work with orientations, for IMUs, the frame_id is the frame in which the
     //  sensor is mounted, and not the coordinate frame of the IMU. Imagine an IMU that is mounted
     //  facing sideways. The pitch in the IMU frame becomes roll for the vehicle. This means that
     //  we need to rotate roll and pitch angles by the IMU's mounting yaw offset, and we must apply
     //  similar treatment to its update mask and covariance.

     double dummy, yaw;
     targetFrameTrans.getBasis().getRPY(dummy, dummy, yaw);
     tf2::Matrix3x3 transTmp;
     transTmp.setRPY(0.0, 0.0, yaw);

     maskPosition = transTmp * maskPosition;
     maskOrientation = transTmp * maskOrientation;
   }
   else
   {
     maskPosition = targetFrameTrans.getBasis() * maskPosition; // getBasis returns the matrix, while getRotation returns the quaternion  by ZD
     maskOrientation = targetFrameTrans.getBasis() * maskOrientation;
   }

   // Now copy the mask values back into the update vector: any row with a significant vector length
   // indicates that we want to set that variable to true in the update vector.
   updateVector[StateMemberX] = static_cast<int>(
     maskPosition.getRow(StateMemberX - POSITION_OFFSET).length() >= 1e-6);
   updateVector[StateMemberY] = static_cast<int>(
     maskPosition.getRow(StateMemberY - POSITION_OFFSET).length() >= 1e-6);
   updateVector[StateMemberZ] = static_cast<int>(
     maskPosition.getRow(StateMemberZ - POSITION_OFFSET).length() >= 1e-6);
   updateVector[StateMemberRoll] = static_cast<int>(
     maskOrientation.getRow(StateMemberRoll - ORIENTATION_OFFSET).length() >= 1e-6);
   updateVector[StateMemberPitch] = static_cast<int>(
     maskOrientation.getRow(StateMemberPitch - ORIENTATION_OFFSET).length() >= 1e-6);
   updateVector[StateMemberYaw] = static_cast<int>(
     maskOrientation.getRow(StateMemberYaw - ORIENTATION_OFFSET).length() >= 1e-6);

   // 5a. We'll need to rotate the covariance as well. Create a container and copy over the
   // covariance data
   Eigen::MatrixXd covariance(POSE_SIZE, POSE_SIZE);
   covariance.setZero();
   copyCovariance(&(msg->pose.covariance[0]), covariance, topicName, updateVector, POSITION_OFFSET, POSE_SIZE);

   // 5b. Now rotate the covariance: create an augmented matrix that
   // contains a 3D rotation matrix in the upper-left and lower-right
   // quadrants, with zeros elsewhere.
   tf2::Matrix3x3 rot;
   Eigen::MatrixXd rot6d(POSE_SIZE, POSE_SIZE);
   rot6d.setIdentity();
   Eigen::MatrixXd covarianceRotated;

   if (imuData)
   {
     // Apply the same special logic to the IMU covariance rotation
     double dummy, yaw;
     targetFrameTrans.getBasis().getRPY(dummy, dummy, yaw);
     rot.setRPY(0.0, 0.0, yaw);  // which means imus worlds orientation is the orientation of the imu at time 0     by ZD
   }
   else
   {
     rot.setRotation(targetFrameTrans.getRotation()); // quaternion to matrix, this rot is used to calculate the rot6d for covarianceRotated   by ZD
   }

   for (size_t rInd = 0; rInd < POSITION_SIZE; ++rInd)
   {
     rot6d(rInd, 0) = rot.getRow(rInd).getX();
     rot6d(rInd, 1) = rot.getRow(rInd).getY();
     rot6d(rInd, 2) = rot.getRow(rInd).getZ();
     rot6d(rInd+POSITION_SIZE, 3) = rot.getRow(rInd).getX();
     rot6d(rInd+POSITION_SIZE, 4) = rot.getRow(rInd).getY();
     rot6d(rInd+POSITION_SIZE, 5) = rot.getRow(rInd).getZ();
   }

   // Now carry out the rotation
   covarianceRotated = rot6d * covariance * rot6d.transpose();

   RF_DEBUG("After rotating into the " << finalTargetFrame <<
            " frame, covariance is \n" << covarianceRotated <<  "\n");

   // 6a. For IMU data, the transform that we get is the transform from the body
   //  frame of the robot (e.g., base_link) to the mounting frame of the robot. It
   //  is *not* the coordinate frame in which the IMU orientation data is reported.
   //  If the IMU is mounted in a non-neutral orientation, we need to remove those
   //  offsets, and then we need to potentially "swap" roll and pitch.
   //  Note that this transform does NOT handle NED->ENU conversions. Data is assumed
   //  to be in the ENU frame when it is received.
   //

   if (imuData)
   {
     // First, convert the transform and measurement rotation to RPY
     // @todo: There must be a way to handle this with quaternions. Need to look into it.
     double rollOffset = 0;
     double pitchOffset = 0;
     double yawOffset = 0;
     double roll = 0;
     double pitch = 0;
     double yaw = 0;
     RosFilterUtilities::quatToRPY(targetFrameTrans.getRotation(), rollOffset, pitchOffset, yawOffset); // this roll pitch yaw offsets are roll pitch yaw of the imu frame   by ZD
     RosFilterUtilities::quatToRPY(poseTmp.getRotation(), roll, pitch, yaw);  // poseTmp saves the pose from the msg sent from callback function by ZD

     // 6b. Apply the offset (making sure to bound them), and throw them in a vector
     tf2::Vector3 rpyAngles(FilterUtilities::clampRotation(roll - rollOffset),
                            FilterUtilities::clampRotation(pitch - pitchOffset),
                            FilterUtilities::clampRotation(yaw - yawOffset));

     // 6c. Now we need to rotate the roll and pitch by the yaw offset value.
     // Imagine a case where an IMU is mounted facing sideways. In that case
     // pitch for the IMU's world frame is roll for the robot.
     tf2::Matrix3x3 mat;
     mat.setRPY(0.0, 0.0, yawOffset);
     rpyAngles = mat * rpyAngles;
     poseTmp.getBasis().setRPY(rpyAngles.getX(), rpyAngles.getY(), rpyAngles.getZ());

     // We will use this target transformation later on, but
     // we've already transformed this data as if the IMU
     // were mounted neutrall on the robot, so we can just
     // make the transform the identity.
     targetFrameTrans.setIdentity();
   }

   // 7. Two cases: if we're in differential mode, we need to generate a twist
   // message. Otherwise, we just transform it to the target frame.
   if (differential)
   {
     bool success = false;

     // We're going to be playing with poseTmp, so store it,
     // as we'll need to save its current value for the next
     // measurement.
     curMeasurement = poseTmp;

     // Make sure we have previous measurements to work with
     if (previousMeasurements_.count(topicName) > 0 && previousMeasurementCovariances_.count(topicName) > 0)
     {
       // 7a. If we are carrying out differential integration and
       // we have a previous measurement for this sensor,then we
       // need to apply the inverse of that measurement to this new
       // measurement to produce a "delta" measurement between the two.
       // Even if we're not using all of the variables from this sensor,
       // we need to use the whole measurement to determine the delta
       // to the new measurement
       tf2::Transform prevMeasurement = previousMeasurements_[topicName];
       // std::cout<<"pre x:"<<prevMeasurement.getOrigin().x()<<", y:"<<prevMeasurement.getOrigin().y()<<", z:"<<prevMeasurement.getOrigin().z()<<std::endl;

       // Now we do what happend in the original 7h.    by ZD
       // 7h. Apply the target frame transformation to the pose object.
       tf2::Stamped<tf2::Transform> poseTmpDifferential(poseTmp);
       poseTmp.mult(targetFrameTrans, poseTmp);
       poseTmp.frame_id_ = finalTargetFrame;

       // 8. Handle 2D mode
       if (twoDMode_)
       {
         forceTwoD(measurement, measurementCovariance, updateVector); // row 228
       }

       // The filter needs roll, pitch, and yaw values instead of quaternions
       double roll, pitch, yaw;
       RosFilterUtilities::quatToRPY(poseTmp.getRotation(), roll, pitch, yaw);

       double dt = msg->header.stamp.toSec() - lastMessageTimes_[topicName].toSec();
       // std::cout<<"new round"<<std::endl;
       // transform the differential filtered velocity back to {B} frame
       const Eigen::VectorXd &state = filter_.getState();
           // std::cout<<"state is:"<<std::endl
           //                  <<state<<std::endl;
       Eigen::VectorXd poseTmplp(6), VelTmplp(6), poseEigen(6);
       std::cout<<"VectorXd generated"<<std::endl;
       poseEigen << state(StateMemberX),//poseTmp.getOrigin().x(),
                                   state(StateMemberY),//poseTmp.getOrigin().y(),
                                   state(StateMemberZ),
                                   roll,
                                   pitch,
                                   yaw;  // */
       differentialTrackers_[topicName]->track(poseEigen, poseTmplp, VelTmplp, dt);

       tf2::Quaternion curAttitude; tf2::Transform trans;
       curAttitude.setRPY(state(StateMemberRoll), state(StateMemberPitch), state(StateMemberYaw));
           // std::cout<<"quaternion is x:"<<curAttitude.getX()<<", y:"<<curAttitude.getY() <<", z:"<<curAttitude.getZ()<<", w:"<<curAttitude.getW()<<std::endl;
       trans.setRotation(curAttitude);
           //std::cout<<"trans is:"<<trans.getRotation()<<std::endl;
       tf2::Vector3 VelTmplptf(VelTmplp(0), VelTmplp(1), VelTmplp(2));
           // std::cout<<"VelTmplptf is:"<<VelTmplptf<<std::endl;
       tf2::Vector3 VelTmplptfB = trans.getBasis().inverse() * VelTmplptf;
           //std::cout<<"VelTmplptfB is:"<<VelTmplptfB<<std::endl;

       /*
       std_msgs::Float64MultiArray msg_test;
           // std::cout<<"Msg Float64MultiArray generated"<<std::endl;
       msg_test.data.resize(19);
           // std::cout<<"Msg Float64MultiArray resized"<<std::endl;
       for (int i=0;i<6;i++)
       {
         msg_test.data[i] = poseEigen(i);
         msg_test.data[6+i] = poseTmplp(i);
         msg_test.data[12+i] = VelTmplp(i);
       }
       msg_test.data[12] = VelTmplptfB.x();
       msg_test.data[13] = VelTmplptfB.y();
       msg_test.data[14] = VelTmplptfB.z();
       msg_test.data[18] = dt;
       //std::cout<<"Msg Float64MultiArray charged"<<std::endl;
       //std::cout<<"Msg Float64MultiArray sent"<<std::endl;
       // */

       // 7i. Finally, copy everything into our measurement and covariance objects
       measurement(StateMemberX) = (updateVector[StateMemberX]) ? poseTmp.getOrigin().x() : measurement(StateMemberX);
       measurement(StateMemberY) = (updateVector[StateMemberY]) ? poseTmp.getOrigin().y() : measurement(StateMemberY);
       measurement(StateMemberZ) = (updateVector[StateMemberZ]) ? poseTmp.getOrigin().z() : measurement(StateMemberZ);

       measurement(StateMemberRoll) = (updateVector[StateMemberRoll]) ? roll : measurement(StateMemberRoll);
       measurement(StateMemberPitch) = (updateVector[StateMemberPitch]) ? pitch : measurement(StateMemberPitch);
       measurement(StateMemberYaw) = (updateVector[StateMemberYaw]) ? yaw : measurement(StateMemberYaw);
       measurementCovariance.block(0, 0, POSE_SIZE, POSE_SIZE) = covarianceRotated.block(0, 0, POSE_SIZE, POSE_SIZE);

       // inverseTimes = (orientation⁻¹)*orientation, translation - translation, poseTmp is the delta pose we need     by ZD
       // poseTmpDifferential.setData(prevMeasurement.inverseTimes(poseTmpDifferential));

       RF_DEBUG("Previous measurement:\n" << previousMeasurements_[topicName] <<
                "\nAfter removing previous measurement, measurement delta is:\n" << poseTmpDifferential << "\n");

       RF_DEBUG("Previous message time was " << lastMessageTimes_[topicName].toSec() <<
                ", current message time is " << msg->header.stamp.toSec() << ", delta is " <<
                dt <<// ", velocity is (vX, vY, vZ): (" << xVel << ", " << yVel << ", " << zVel <<
                ")\n" << "(vRoll, vPitch, vYaw): (" << VelTmplp(3) << ", " << VelTmplp(4) << ", " <<
                VelTmplp(5) << ")\n");

       // 7d. Fill out the velocity data in the message
       geometry_msgs::TwistWithCovarianceStamped *twistPtr = new geometry_msgs::TwistWithCovarianceStamped();
       twistPtr->header = msg->header;
       twistPtr->header.frame_id = baseLinkFrameId_;
       twistPtr->twist.twist.linear.x = (updateVector[StateMemberX]) ? VelTmplptfB.x() : measurement(StateMemberVx);
       twistPtr->twist.twist.linear.y = (updateVector[StateMemberY]) ? VelTmplptfB.y() : measurement(StateMemberVy);
       twistPtr->twist.twist.linear.z = (updateVector[StateMemberZ]) ? VelTmplptfB.z() : measurement(StateMemberVz);
       twistPtr->twist.twist.angular.x = (updateVector[StateMemberRoll]) ? VelTmplp(3) : measurement(StateMemberVroll);
       twistPtr->twist.twist.angular.y = (updateVector[StateMemberPitch]) ? VelTmplp(4) : measurement(StateMemberVpitch);
       twistPtr->twist.twist.angular.z = (updateVector[StateMemberYaw]) ? VelTmplp(5) : measurement(StateMemberVyaw);
       std::vector<int> twistUpdateVec(STATE_SIZE, false);
       std::copy(updateVector.begin() + POSITION_OFFSET,  // while (first!=last), *result=*first, ++result,++first  by ZD
                 updateVector.begin() + POSE_SIZE,
                 twistUpdateVec.begin() + POSITION_V_OFFSET); // twistUpdateVec = f f f f f f t t t t t t f f f    by ZD
       //std::copy(twistUpdateVec.begin(), twistUpdateVec.end(), updateVector.begin());  // updateVector = twistUpdateVec  by ZD
       std::copy(twistUpdateVec.begin() + POSITION_V_OFFSET, twistUpdateVec.begin() + POSE_SIZE + TWIST_SIZE, updateVector.begin() + POSITION_V_OFFSET);  // updateVector = twistUpdateVec  by ZD
       geometry_msgs::TwistWithCovarianceStampedConstPtr ptr(twistPtr);
       // std::cout<<"UpdateVec "<<updateVector<<std::endl;
       // std::cout<<"Nach Vx:"<<twistPtr->twist.twist.linear.x<<"Vy:"<<twistPtr->twist.twist.linear.y<<"Vz:"<<twistPtr->twist.twist.linear.z<<std::endl;

       // 7e. Now rotate the previous covariance for this measurement to get it
       // into the target frame, and add the current measurement's rotated covariance
       // to the previous measurement's rotated covariance, and multiply by the time delta.
       Eigen::MatrixXd prevCovarRotated = rot6d * previousMeasurementCovariances_[topicName] * rot6d.transpose();
       covarianceRotated = (covarianceRotated.eval() + prevCovarRotated) * dt;  // why ? I think here should be /dt    by ZD
       copyCovariance(covarianceRotated, &(twistPtr->twist.covariance[0]), POSE_SIZE);

       RF_DEBUG("Previous measurement covariance:\n" << previousMeasurementCovariances_[topicName] <<
                "\nPrevious measurement covariance rotated:\n" << prevCovarRotated <<
                "\nFinal twist covariance:\n" << covarianceRotated << "\n");

       // std::cout<<"Vor x:"<<curMeasurement.getOrigin().getX()<<"y:"<<curMeasurement.getOrigin().getY()<<"z:"<<curMeasurement.getOrigin().getZ()<<std::endl;

       // Now pass this on to prepareTwist, which will convert it to the required frame
       success = prepareTwist(ptr,
                              topicName + "_twist",
                              twistPtr->header.frame_id,
                              updateVector,
                              measurement,   // it seems prepareTwist only modify twist part of the measurement  by ZD
                              measurementCovariance);
       // std::cout<<"measurement x:"<<measurement(StateMemberX)<<", y:"<<measurement(StateMemberY)<<", z:"<<measurement(StateMemberZ)<<std::endl;
     }

     // 7f. Update the previous measurement and measurement covariance
     previousMeasurements_[topicName] = curMeasurement;
     previousMeasurementCovariances_[topicName] = covariance;

     retVal = success;
   }
   else
   {
     // 7g. If we're in relative mode, remove the initial measurement
     if (relative)
     {
       if (initialMeasurements_.count(topicName) == 0)
       {
         initialMeasurements_.insert(std::pair<std::string, tf2::Transform>(topicName, poseTmp));
       }

       tf2::Transform initialMeasurement = initialMeasurements_[topicName];
       poseTmp.setData(initialMeasurement.inverseTimes(poseTmp));
     }

     // 7h. Apply the target frame transformation to the poseTmp
     poseTmp.mult(targetFrameTrans, poseTmp);
     poseTmp.frame_id_ = finalTargetFrame;  

     // 7i. Finally, copy everything into our measurement and covariance objects
     measurement(StateMemberX) = (updateVector[StateMemberX]) ? poseTmp.getOrigin().x() : measurement(StateMemberX);
     measurement(StateMemberY) = (updateVector[StateMemberY]) ? poseTmp.getOrigin().y() : measurement(StateMemberY);
     measurement(StateMemberZ) = (updateVector[StateMemberZ]) ? poseTmp.getOrigin().z() : measurement(StateMemberZ);

     // The filter needs roll, pitch, and yaw values instead of quaternions
     double roll, pitch, yaw;
     RosFilterUtilities::quatToRPY(poseTmp.getRotation(), roll, pitch, yaw);
     measurement(StateMemberRoll) = (updateVector[StateMemberRoll]) ? roll : measurement(StateMemberRoll);
     measurement(StateMemberPitch) = (updateVector[StateMemberPitch]) ? pitch : measurement(StateMemberPitch);
     measurement(StateMemberYaw) = (updateVector[StateMemberYaw]) ? yaw : measurement(StateMemberYaw);

     measurementCovariance.block(0, 0, POSE_SIZE, POSE_SIZE) = covarianceRotated.block(0, 0, POSE_SIZE, POSE_SIZE);

     // 8. Handle 2D mode
     if (twoDMode_)
     {
       forceTwoD(measurement, measurementCovariance, updateVector); // row 228
     }

     retVal = true;
   }
 }
 else // if (canTransform = false)     by ZD
 {
   retVal = false;

   RF_DEBUG("Could not transform measurement into " << finalTargetFrame << ". Ignoring...");
 }

 RF_DEBUG("\n----- /FireflyRosFilter::preparePose (" << topicName << ") ------\n");

 return retVal;
}
// */

// original prepareTwist
 /*
template<typename T>
bool FireflyRosFilter<T>::prepareTwist(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr &msg,
                             const std::string &topicName,
                             const std::string &targetFrame,
                             std::vector<int> &updateVector,
                             Eigen::VectorXd &measurement,
                             Eigen::MatrixXd &measurementCovariance)
{
  RF_DEBUG("------ FireflyRosFilter::prepareTwist (" << topicName << ") ------\n");

  // 1. Get the measurement into two separate vector objects.
  tf2::Vector3 twistLin(msg->twist.twist.linear.x,
                        msg->twist.twist.linear.y,
                        msg->twist.twist.linear.z);
  tf2::Vector3 measTwistRot(msg->twist.twist.angular.x,
                            msg->twist.twist.angular.y,
                            msg->twist.twist.angular.z);

  // 1a. This sensor may or may not measure rotational velocity. Regardless,
  // if it measures linear velocity, then later on, we'll need to remove "false"
  // linear velocity resulting from angular velocity and the translational offset
  // of the sensor from the vehicle origin.
  const Eigen::VectorXd &state = filter_.getState();
  tf2::Vector3 stateTwistRot(state(StateMemberVroll),
                             state(StateMemberVpitch),
                             state(StateMemberVyaw));

  // Determine the frame_id of the data
  std::string msgFrame = (msg->header.frame_id == "" ? targetFrame : msg->header.frame_id);

  // 2. robot_localization lets users configure which variables from the sensor should be
  //    fused with the filter. This is specified at the sensor level. However, the data
  //    may go through transforms before being fused with the state estimate. In that case,
  //    we need to know which of the transformed variables came from the pre-transformed
  //    "approved" variables (i.e., the ones that had "true" in their xxx_config parameter).
  //    To do this, we construct matrices using the update vector values on the diagonals,
  //    pass this matrix through the rotation, and use the length of each row to determine
  //    the transformed update vector.
  tf2::Matrix3x3 maskLin(updateVector[StateMemberVx], 0, 0,
                         0, updateVector[StateMemberVy], 0,
                         0, 0, updateVector[StateMemberVz]);

  tf2::Matrix3x3 maskRot(updateVector[StateMemberVroll], 0, 0,
                         0, updateVector[StateMemberVpitch], 0,
                         0, 0, updateVector[StateMemberVyaw]);

  // 3. We'll need to rotate the covariance as well
  Eigen::MatrixXd covarianceRotated(TWIST_SIZE, TWIST_SIZE);
  covarianceRotated.setZero();

  copyCovariance(&(msg->twist.covariance[0]),
                 covarianceRotated,
                 topicName,
                 updateVector,
                 POSITION_V_OFFSET,
                 TWIST_SIZE);

  RF_DEBUG("Original measurement as tf object:\nLinear: " << twistLin <<
           "Rotational: " << measTwistRot <<
           "\nOriginal update vector:\n" << updateVector <<
           "\nOriginal covariance matrix:\n" << covarianceRotated << "\n");

  // 4. We need to transform this into the target frame (probably base_link)
  tf2::Transform targetFrameTrans;
  bool canTransform = RosFilterUtilities::lookupTransformSafe(tfBuffer_,
                                                              targetFrame,
                                                              msgFrame,
                                                              msg->header.stamp,
                                                              targetFrameTrans);

  if (canTransform)
  {
    // Transform to correct frame. Note that we can get linear velocity
    // as a result of the sensor offset and rotational velocity
    measTwistRot = targetFrameTrans.getBasis() * measTwistRot;
    twistLin = targetFrameTrans.getBasis() * twistLin + targetFrameTrans.getOrigin().cross(stateTwistRot);
    maskLin = targetFrameTrans.getBasis() * maskLin;
    maskRot = targetFrameTrans.getBasis() * maskRot;

    // Now copy the mask values back into the update vector
    updateVector[StateMemberVx] = static_cast<int>(
      maskLin.getRow(StateMemberVx - POSITION_V_OFFSET).length() >= 1e-6);
    updateVector[StateMemberVy] = static_cast<int>(
      maskLin.getRow(StateMemberVy - POSITION_V_OFFSET).length() >= 1e-6);
    updateVector[StateMemberVz] = static_cast<int>(
      maskLin.getRow(StateMemberVz - POSITION_V_OFFSET).length() >= 1e-6);
    updateVector[StateMemberVroll] = static_cast<int>(
      maskRot.getRow(StateMemberVroll - ORIENTATION_V_OFFSET).length() >= 1e-6);
    updateVector[StateMemberVpitch] = static_cast<int>(
      maskRot.getRow(StateMemberVpitch - ORIENTATION_V_OFFSET).length() >= 1e-6);
    updateVector[StateMemberVyaw] = static_cast<int>(
      maskRot.getRow(StateMemberVyaw - ORIENTATION_V_OFFSET).length() >= 1e-6);

    RF_DEBUG(msg->header.frame_id << "->" << targetFrame << " transform:\n" << targetFrameTrans <<
             "\nAfter applying transform to " << targetFrame << ", update vector is:\n" << updateVector <<
             "\nAfter applying transform to " << targetFrame << ", measurement is:\n" <<
             "Linear: " << twistLin << "Rotational: " << measTwistRot << "\n");

    // 5. Now rotate the covariance: create an augmented
    // matrix that contains a 3D rotation matrix in the
    // upper-left and lower-right quadrants, and zeros
    // elsewhere
    tf2::Matrix3x3 rot(targetFrameTrans.getRotation());
    Eigen::MatrixXd rot6d(TWIST_SIZE, TWIST_SIZE);
    rot6d.setIdentity();

    for (size_t rInd = 0; rInd < POSITION_SIZE; ++rInd)
    {
      rot6d(rInd, 0) = rot.getRow(rInd).getX();
      rot6d(rInd, 1) = rot.getRow(rInd).getY();
      rot6d(rInd, 2) = rot.getRow(rInd).getZ();
      rot6d(rInd+POSITION_SIZE, 3) = rot.getRow(rInd).getX();
      rot6d(rInd+POSITION_SIZE, 4) = rot.getRow(rInd).getY();
      rot6d(rInd+POSITION_SIZE, 5) = rot.getRow(rInd).getZ();
    }

    // Carry out the rotation
    covarianceRotated = rot6d * covarianceRotated.eval() * rot6d.transpose();

    RF_DEBUG("Transformed covariance is \n" << covarianceRotated << "\n");

    // 6. Store our corrected measurement and covariance
    measurement(StateMemberVx) = twistLin.getX();
    measurement(StateMemberVy) = twistLin.getY();
    measurement(StateMemberVz) = twistLin.getZ();
    measurement(StateMemberVroll) = measTwistRot.getX();
    measurement(StateMemberVpitch) = measTwistRot.getY();
    measurement(StateMemberVyaw) = measTwistRot.getZ();

    // Copy the covariances
    measurementCovariance.block(POSITION_V_OFFSET, POSITION_V_OFFSET, TWIST_SIZE, TWIST_SIZE) =
      covarianceRotated.block(0, 0, TWIST_SIZE, TWIST_SIZE);

    // 7. Handle 2D mode
    if (twoDMode_)
    {
      forceTwoD(measurement, measurementCovariance, updateVector);
    }
  }
  else
  {
    RF_DEBUG("Could not transform measurement into " << targetFrame << ". Ignoring...");
  }

  RF_DEBUG("\n----- /RosFilter::prepareTwist (" << topicName << ") ------\n");

  return canTransform;
}
// */

// modified prepareTwist
// /*
template<typename T>
bool FireflyRosFilter<T>::prepareTwist(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr &msg,
                             const std::string &topicName,
                             const std::string &targetFrame,
                             std::vector<int> &updateVector,
                             Eigen::VectorXd &measurement,
                             Eigen::MatrixXd &measurementCovariance)
{
  RF_DEBUG("------ FireflyRosFilter::prepareTwist (" << topicName << ") ------\n");

  // 1. Get the measurement into two separate vector objects.
  tf2::Vector3 twistLin(msg->twist.twist.linear.x,
                        msg->twist.twist.linear.y,
                        msg->twist.twist.linear.z);
  tf2::Vector3 measTwistRot(msg->twist.twist.angular.x,
                            msg->twist.twist.angular.y,
                            msg->twist.twist.angular.z);

  // 1a. This sensor may or may not measure rotational velocity. Regardless,
  // if it measures linear velocity, then later on, we'll need to remove "false"
  // linear velocity resulting from angular velocity and the translational offset
  // of the sensor from the vehicle origin.
  const Eigen::VectorXd &state = filter_.getState();
  tf2::Vector3 stateTwistRot(state(StateMemberVroll),
                             state(StateMemberVpitch),
                             state(StateMemberVyaw));

  // Determine the frame_id of the data
  std::string msgFrame = (msg->header.frame_id == "" ? targetFrame : msg->header.frame_id);

  // 2. robot_localization lets users configure which variables from the sensor should be
  //    fused with the filter. This is specified at the sensor level. However, the data
  //    may go through transforms before being fused with the state estimate. In that case,
  //    we need to know which of the transformed variables came from the pre-transformed
  //    "approved" variables (i.e., the ones that had "true" in their xxx_config parameter).
  //    To do this, we construct matrices using the update vector values on the diagonals,
  //    pass this matrix through the rotation, and use the length of each row to determine
  //    the transformed update vector.
  tf2::Matrix3x3 maskLin(updateVector[StateMemberVx], 0, 0,
                         0, updateVector[StateMemberVy], 0,
                         0, 0, updateVector[StateMemberVz]);

  tf2::Matrix3x3 maskRot(updateVector[StateMemberVroll], 0, 0,
                         0, updateVector[StateMemberVpitch], 0,
                         0, 0, updateVector[StateMemberVyaw]);

  // 3. We'll need to rotate the covariance as well
  Eigen::MatrixXd covarianceRotated(TWIST_SIZE, TWIST_SIZE);
  covarianceRotated.setZero();

  copyCovariance(&(msg->twist.covariance[0]),
                 covarianceRotated,
                 topicName,
                 updateVector,
                 POSITION_V_OFFSET,
                 TWIST_SIZE);

  RF_DEBUG("Original measurement as tf object:\nLinear: " << twistLin <<
           "Rotational: " << measTwistRot <<
           "\nOriginal update vector:\n" << updateVector <<
           "\nOriginal covariance matrix:\n" << covarianceRotated << "\n");

  // 4. We need to transform this into the target frame (probably base_link)
  tf2::Transform targetFrameTrans;
  bool canTransform = RosFilterUtilities::lookupTransformSafe(tfBuffer_,
                                                              targetFrame,
                                                              msgFrame,
                                                              msg->header.stamp,
                                                              targetFrameTrans);

  if (canTransform)
  {
    // Transform to correct frame. Note that we can get linear velocity
    // as a result of the sensor offset and rotational velocity
    measTwistRot = targetFrameTrans.getBasis() * measTwistRot;
    twistLin = targetFrameTrans.getBasis() * twistLin + targetFrameTrans.getOrigin().cross(stateTwistRot);
    maskLin = targetFrameTrans.getBasis() * maskLin;
    maskRot = targetFrameTrans.getBasis() * maskRot;

    // Now copy the mask values back into the update vector
    updateVector[StateMemberVx] = static_cast<int>(
      maskLin.getRow(StateMemberVx - POSITION_V_OFFSET).length() >= 1e-6);
    updateVector[StateMemberVy] = static_cast<int>(
      maskLin.getRow(StateMemberVy - POSITION_V_OFFSET).length() >= 1e-6);
    updateVector[StateMemberVz] = static_cast<int>(
      maskLin.getRow(StateMemberVz - POSITION_V_OFFSET).length() >= 1e-6);
    updateVector[StateMemberVroll] = static_cast<int>(
      maskRot.getRow(StateMemberVroll - ORIENTATION_V_OFFSET).length() >= 1e-6);
    updateVector[StateMemberVpitch] = static_cast<int>(
      maskRot.getRow(StateMemberVpitch - ORIENTATION_V_OFFSET).length() >= 1e-6);
    updateVector[StateMemberVyaw] = static_cast<int>(
      maskRot.getRow(StateMemberVyaw - ORIENTATION_V_OFFSET).length() >= 1e-6);

    RF_DEBUG(msg->header.frame_id << "->" << targetFrame << " transform:\n" << targetFrameTrans <<
             "\nAfter applying transform to " << targetFrame << ", update vector is:\n" << updateVector <<
             "\nAfter applying transform to " << targetFrame << ", measurement is:\n" <<
             "Linear: " << twistLin << "Rotational: " << measTwistRot << "\n");

    // 5. Now rotate the covariance: create an augmented
    // matrix that contains a 3D rotation matrix in the
    // upper-left and lower-right quadrants, and zeros
    // elsewhere
    tf2::Matrix3x3 rot(targetFrameTrans.getRotation());
    Eigen::MatrixXd rot6d(TWIST_SIZE, TWIST_SIZE);
    rot6d.setIdentity();

    for (size_t rInd = 0; rInd < POSITION_SIZE; ++rInd)
    {
      rot6d(rInd, 0) = rot.getRow(rInd).getX();
      rot6d(rInd, 1) = rot.getRow(rInd).getY();
      rot6d(rInd, 2) = rot.getRow(rInd).getZ();
      rot6d(rInd+POSITION_SIZE, 3) = rot.getRow(rInd).getX();
      rot6d(rInd+POSITION_SIZE, 4) = rot.getRow(rInd).getY();
      rot6d(rInd+POSITION_SIZE, 5) = rot.getRow(rInd).getZ();
    }

    // Carry out the rotation
    covarianceRotated = rot6d * covarianceRotated.eval() * rot6d.transpose();

    RF_DEBUG("Transformed covariance is \n" << covarianceRotated << "\n");

    // 6. Store our corrected measurement and covariance
    measurement(StateMemberVx) = (updateVector[StateMemberVx]) ? twistLin.getX() : measurement(StateMemberVx);
    measurement(StateMemberVy) = (updateVector[StateMemberVy]) ? twistLin.getY() : measurement(StateMemberVy);
    measurement(StateMemberVz) = (updateVector[StateMemberVz]) ? twistLin.getZ() : measurement(StateMemberVz);
    measurement(StateMemberVroll) = (updateVector[StateMemberVroll]) ? measTwistRot.getX() : measurement(StateMemberVroll);
    measurement(StateMemberVpitch) = (updateVector[StateMemberVpitch]) ? measTwistRot.getY() : measurement(StateMemberVpitch);
    measurement(StateMemberVyaw) = (updateVector[StateMemberVyaw]) ? measTwistRot.getZ() : measurement(StateMemberVyaw);

    // Copy the covariances
    measurementCovariance.block(POSITION_V_OFFSET, POSITION_V_OFFSET, TWIST_SIZE, TWIST_SIZE) =
      covarianceRotated.block(0, 0, TWIST_SIZE, TWIST_SIZE);

    // 7. Handle 2D mode
    if (twoDMode_)
    {
      forceTwoD(measurement, measurementCovariance, updateVector);
    }
  }
  else
  {
    RF_DEBUG("Could not transform measurement into " << targetFrame << ". Ignoring...");
  }

  RF_DEBUG("\n----- /RosFilter::prepareTwist (" << topicName << ") ------\n");

  return canTransform;
}
// */

// original prepareAcceleration
/*
template<typename T>
bool FireflyRosFilter<T>::prepareAcceleration(const sensor_msgs::Imu::ConstPtr &msg,
                         const std::string &topicName,
                         const std::string &targetFrame,
                         std::vector<int> &updateVector,
                         Eigen::VectorXd &measurement,
                         Eigen::MatrixXd &measurementCovariance)
{
  RF_DEBUG("------ FireflyRosFilter::prepareAcceleration (" << topicName << ") ------\n");

  // 1. Get the measurement into a vector
  tf2::Vector3 accTmp(msg->linear_acceleration.x,
                                            msg->linear_acceleration.y,
                                            msg->linear_acceleration.z);

  // Set relevant header info
  std::string msgFrame = (msg->header.frame_id == "" ? baseLinkFrameId_ : msg->header.frame_id);

  // 2. robot_localization lets users configure which variables from the sensor should be
  //    fused with the filter. This is specified at the sensor level. However, the data
  //    may go through transforms before being fused with the state estimate. In that case,
  //    we need to know which of the transformed variables came from the pre-transformed
  //    "approved" variables (i.e., the ones that had "true" in their xxx_config parameter).
  //    To do this, we create a pose from the original upate vector, which contains only
  //    zeros and ones. This pose goes through the same transforms as the measurement. The
  //    non-zero values that result will be used to modify the updateVector.
  tf2::Matrix3x3 maskAcc(updateVector[StateMemberAx], 0, 0,
                         0, updateVector[StateMemberAy], 0,
                         0, 0, updateVector[StateMemberAz]);

  // 3. We'll need to rotate the covariance as well
  Eigen::MatrixXd covarianceRotated(ACCELERATION_SIZE, ACCELERATION_SIZE);
  covarianceRotated.setZero();

  this->copyCovariance(&(msg->linear_acceleration_covariance[0]),
                       covarianceRotated,
                       topicName,
                       updateVector,
                       POSITION_A_OFFSET,
                       ACCELERATION_SIZE);

  RF_DEBUG("Original measurement as tf object: " << accTmp <<
           "\nOriginal update vector:\n" << updateVector <<
           "\nOriginal covariance matrix:\n" << covarianceRotated << "\n");

  // 4. We need to transform this into the target frame (probably base_link)
  // It's unlikely that we'll get a velocity measurement in another frame, but
  // we have to handle the situation.
  tf2::Transform targetFrameTrans;  // from imu frame to probably base_link     by ZD
  bool canTransform = RosFilterUtilities::lookupTransformSafe(tfBuffer_,
                                                              targetFrame,
                                                              msgFrame,
                                                              msg->header.stamp,
                                                              targetFrameTrans);

  if (canTransform)
  {
    // We don't know if the user has already handled the removal
    // of normal forces, so we use a parameter
    if (removeGravitationalAcc_[topicName])
    {
      tf2::Vector3 normAcc(0, 0, 9.80665);
      tf2::Quaternion curAttitude;
      tf2::Transform trans;

      if(::fabs(msg->orientation_covariance[0] + 1) < 1e-9)
      {
        // Imu message contains no orientation, so we should use orientation
        // from filter state to transform and remove acceleration
        const Eigen::VectorXd &state = filter_.getState();
        tf2::Vector3 stateTmp(state(StateMemberRoll),
                                                     state(StateMemberPitch),
                                                     state(StateMemberYaw));
        // transform state orientation to IMU frame
        tf2::Transform imuFrameTrans;
        RosFilterUtilities::lookupTransformSafe(tfBuffer_,
                                                msgFrame,
                                                targetFrame,
                                                msg->header.stamp,
                                                imuFrameTrans);
        stateTmp = imuFrameTrans.getBasis() * stateTmp;
        curAttitude.setRPY(stateTmp.getX(), stateTmp.getY(), stateTmp.getZ());
      }
      else
      {
        tf2::fromMsg(msg->orientation, curAttitude);
      }
      trans.setRotation(curAttitude);
      tf2::Vector3 rotNorm = trans.getBasis().inverse() * normAcc;
      accTmp.setX(accTmp.getX() - rotNorm.getX());   // plus forwards accel due to gravity
      accTmp.setY(accTmp.getY() - rotNorm.getY());   // plus rightwards accel due to gravity
      accTmp.setZ(accTmp.getZ() - rotNorm.getZ());   // minus downwards accel due to gravity

      RF_DEBUG("Orientation is " << curAttitude <<
               "Acceleration due to gravity is " << rotNorm <<
               "After removing acceleration due to gravity, acceleration is " << accTmp << "\n");
    }

    // Transform to correct frame
    // @todo: This needs to take into account offsets from the origin. Right now,
    // it assumes that if the sensor is placed at some non-zero offset from the
    // vehicle's center, that the vehicle turns with constant velocity. This needs
    // to be something like
    // accTmp = targetFrameTrans.getBasis() * accTmp - targetFrameTrans.getOrigin().cross(rotation_acceleration);
    // We can get rotational acceleration by differentiating the rotational velocity
    // (if it's available)
    accTmp = targetFrameTrans.getBasis() * accTmp;
    maskAcc = targetFrameTrans.getBasis() * maskAcc;

    // Now use the mask values to determine which update vector values should be true
    updateVector[StateMemberAx] = static_cast<int>(
      maskAcc.getRow(StateMemberAx - POSITION_A_OFFSET).length() >= 1e-6);
    updateVector[StateMemberAy] = static_cast<int>(
      maskAcc.getRow(StateMemberAy - POSITION_A_OFFSET).length() >= 1e-6);
    updateVector[StateMemberAz] = static_cast<int>(
      maskAcc.getRow(StateMemberAz - POSITION_A_OFFSET).length() >= 1e-6);

    RF_DEBUG(msg->header.frame_id << "->" << targetFrame << " transform:\n" << targetFrameTrans <<
             "\nAfter applying transform to " << targetFrame << ", update vector is:\n" << updateVector <<
             "\nAfter applying transform to " << targetFrame << ", measurement is:\n" << accTmp << "\n");

    // 5. Now rotate the covariance: create an augmented
    // matrix that contains a 3D rotation matrix in the
    // upper-left and lower-right quadrants, and zeros
    // elsewhere
    tf2::Matrix3x3 rot(targetFrameTrans.getRotation());
    Eigen::MatrixXd rot3d(ACCELERATION_SIZE, ACCELERATION_SIZE);
    rot3d.setIdentity();

    for (size_t rInd = 0; rInd < ACCELERATION_SIZE; ++rInd)
    {
      rot3d(rInd, 0) = rot.getRow(rInd).getX();
      rot3d(rInd, 1) = rot.getRow(rInd).getY();
      rot3d(rInd, 2) = rot.getRow(rInd).getZ();
    }

    // Carry out the rotation
    covarianceRotated = rot3d * covarianceRotated.eval() * rot3d.transpose();  // matrix rotation is R*Matrix*R.transpose, not like vector rotation            by ZD

    RF_DEBUG("Transformed covariance is \n" << covarianceRotated << "\n");

    // 6. Store our corrected measurement and covariance
    measurement(StateMemberAx) = accTmp.getX();
    measurement(StateMemberAy) = accTmp.getY();
    measurement(StateMemberAz) = accTmp.getZ();

    // Copy the covariances
    measurementCovariance.block(POSITION_A_OFFSET, POSITION_A_OFFSET, ACCELERATION_SIZE, ACCELERATION_SIZE) =
      covarianceRotated.block(0, 0, ACCELERATION_SIZE, ACCELERATION_SIZE);

    // 7. Handle 2D mode
    if (twoDMode_)
    {
      forceTwoD(measurement, measurementCovariance, updateVector);
    }
  }
  else
  {
    RF_DEBUG("Could not transform measurement into " << targetFrame << ". Ignoring...\n");
  }

  RF_DEBUG("\n----- /RosFilter::prepareAcceleration(" << topicName << ") ------\n");

  return canTransform;
}
// */

// modified prepareAcceleration
// /*
template<typename T>
bool FireflyRosFilter<T>::prepareAcceleration(const sensor_msgs::Imu::ConstPtr &msg,
                         const std::string &topicName,
                         const std::string &targetFrame,
                         std::vector<int> &updateVector,
                         Eigen::VectorXd &measurement,
                         Eigen::MatrixXd &measurementCovariance)
{
  RF_DEBUG("------ FireflyRosFilter::prepareAcceleration (" << topicName << ") ------\n");

  // 1. Get the measurement into a vector
  tf2::Vector3 accTmp(msg->linear_acceleration.x,
                                            msg->linear_acceleration.y,
                                            msg->linear_acceleration.z);

  // Set relevant header info
  std::string msgFrame = (msg->header.frame_id == "" ? baseLinkFrameId_ : msg->header.frame_id);

  // 2. robot_localization lets users configure which variables from the sensor should be
  //    fused with the filter. This is specified at the sensor level. However, the data
  //    may go through transforms before being fused with the state estimate. In that case,
  //    we need to know which of the transformed variables came from the pre-transformed
  //    "approved" variables (i.e., the ones that had "true" in their xxx_config parameter).
  //    To do this, we create a pose from the original upate vector, which contains only
  //    zeros and ones. This pose goes through the same transforms as the measurement. The
  //    non-zero values that result will be used to modify the updateVector.
  tf2::Matrix3x3 maskAcc(updateVector[StateMemberAx], 0, 0,
                         0, updateVector[StateMemberAy], 0,
                         0, 0, updateVector[StateMemberAz]);

  // 3. We'll need to rotate the covariance as well
  Eigen::MatrixXd covarianceRotated(ACCELERATION_SIZE, ACCELERATION_SIZE);
  covarianceRotated.setZero();

  this->copyCovariance(&(msg->linear_acceleration_covariance[0]),
                       covarianceRotated,
                       topicName,
                       updateVector,
                       POSITION_A_OFFSET,
                       ACCELERATION_SIZE);

  RF_DEBUG("Original measurement as tf object: " << accTmp <<
           "\nOriginal update vector:\n" << updateVector <<
           "\nOriginal covariance matrix:\n" << covarianceRotated << "\n");

  // 4. We need to transform this into the target frame (probably base_link)
  // It's unlikely that we'll get a velocity measurement in another frame, but
  // we have to handle the situation.
  tf2::Transform targetFrameTrans;  // from imu frame to probably base_link     by ZD
  bool canTransform = RosFilterUtilities::lookupTransformSafe(tfBuffer_,
                                                              targetFrame,
                                                              msgFrame,
                                                              msg->header.stamp,
                                                              targetFrameTrans);

  if (canTransform)
  {
    // We don't know if the user has already handled the removal
    // of normal forces, so we use a parameter
    if (removeGravitationalAcc_[topicName])
    {
      tf2::Vector3 normAcc(0, 0, 9.80665);
      tf2::Quaternion curAttitude;
      tf2::Transform trans;

      if(::fabs(msg->orientation_covariance[0] + 1) < 1e-9)
      {
        // Imu message contains no orientation, so we should use orientation
        // from filter state to transform and remove acceleration
        const Eigen::VectorXd &state = filter_.getState();
        tf2::Vector3 stateTmp(state(StateMemberRoll),
                                                     state(StateMemberPitch),
                                                     state(StateMemberYaw));
        // transform state orientation to IMU frame
        tf2::Transform imuFrameTrans;
        RosFilterUtilities::lookupTransformSafe(tfBuffer_,
                                                msgFrame,
                                                targetFrame,
                                                msg->header.stamp,
                                                imuFrameTrans);
        stateTmp = imuFrameTrans.getBasis() * stateTmp;
        curAttitude.setRPY(stateTmp.getX(), stateTmp.getY(), stateTmp.getZ());
      }
      else
      {
        tf2::fromMsg(msg->orientation, curAttitude);
      }
      trans.setRotation(curAttitude);
      tf2::Vector3 rotNorm = trans.getBasis().inverse() * normAcc;
      accTmp.setX(accTmp.getX() - rotNorm.getX());   // plus forwards accel due to gravity
      accTmp.setY(accTmp.getY() - rotNorm.getY());   // plus rightwards accel due to gravity
      accTmp.setZ(accTmp.getZ() - rotNorm.getZ());   // minus downwards accel due to gravity

     /*  geometry_msgs::PointStamped correctAcceleration;
      correctAcceleration.point.x = accTmp.getX();
      correctAcceleration.point.y = accTmp.getY();
      correctAcceleration.point.z = accTmp.getZ() + rotNorm.getZ();
      correctAcceleration.header = msg->header;
      imuPubs_[topicName].publish(correctAcceleration); */

      RF_DEBUG("Orientation is " << curAttitude <<
               "Acceleration due to gravity is " << rotNorm <<
               "After removing acceleration due to gravity, acceleration is " << accTmp << "\n");
    }

    // Transform to correct frame
    // @todo: This needs to take into account offsets from the origin. Right now,
    // it assumes that if the sensor is placed at some non-zero offset from the
    // vehicle's center, that the vehicle turns with constant velocity. This needs
    // to be something like
    // accTmp = targetFrameTrans.getBasis() * accTmp - targetFrameTrans.getOrigin().cross(rotation_acceleration);
    // We can get rotational acceleration by differentiating the rotational velocity
    // (if it's available)
    accTmp = targetFrameTrans.getBasis() * accTmp;
    maskAcc = targetFrameTrans.getBasis() * maskAcc;

    // Now use the mask values to determine which update vector values should be true
    updateVector[StateMemberAx] = static_cast<int>(
      maskAcc.getRow(StateMemberAx - POSITION_A_OFFSET).length() >= 1e-6);
    updateVector[StateMemberAy] = static_cast<int>(
      maskAcc.getRow(StateMemberAy - POSITION_A_OFFSET).length() >= 1e-6);
    updateVector[StateMemberAz] = static_cast<int>(
      maskAcc.getRow(StateMemberAz - POSITION_A_OFFSET).length() >= 1e-6);

    RF_DEBUG(msg->header.frame_id << "->" << targetFrame << " transform:\n" << targetFrameTrans <<
             "\nAfter applying transform to " << targetFrame << ", update vector is:\n" << updateVector <<
             "\nAfter applying transform to " << targetFrame << ", measurement is:\n" << accTmp << "\n");

    // 5. Now rotate the covariance: create an augmented
    // matrix that contains a 3D rotation matrix in the
    // upper-left and lower-right quadrants, and zeros
    // elsewhere
    tf2::Matrix3x3 rot(targetFrameTrans.getRotation());
    Eigen::MatrixXd rot3d(ACCELERATION_SIZE, ACCELERATION_SIZE);
    rot3d.setIdentity();

    for (size_t rInd = 0; rInd < ACCELERATION_SIZE; ++rInd)
    {
      rot3d(rInd, 0) = rot.getRow(rInd).getX();
      rot3d(rInd, 1) = rot.getRow(rInd).getY();
      rot3d(rInd, 2) = rot.getRow(rInd).getZ();
    }

    // Carry out the rotation
    covarianceRotated = rot3d * covarianceRotated.eval() * rot3d.transpose();  // matrix rotation is R*Matrix*R.transpose, not like vector rotation            by ZD

    RF_DEBUG("Transformed covariance is \n" << covarianceRotated << "\n");

    // 6. Store our corrected measurement and covariance
    measurement(StateMemberAx) = updateVector[StateMemberAx] ? accTmp.getX() : measurement(StateMemberAx);
    measurement(StateMemberAy) = updateVector[StateMemberAy] ? accTmp.getY() : measurement(StateMemberAy);
    measurement(StateMemberAz) = updateVector[StateMemberAz] ? accTmp.getZ() : measurement(StateMemberAz);

    // Copy the covariances
    measurementCovariance.block(POSITION_A_OFFSET, POSITION_A_OFFSET, ACCELERATION_SIZE, ACCELERATION_SIZE) =
      covarianceRotated.block(0, 0, ACCELERATION_SIZE, ACCELERATION_SIZE);

    // 7. Handle 2D mode
    if (twoDMode_)
    {
      forceTwoD(measurement, measurementCovariance, updateVector);
    }
  }
  else
  {
    RF_DEBUG("Could not transform measurement into " << targetFrame << ". Ignoring...\n");
  }

  RF_DEBUG("\n----- /RosFilter::prepareAcceleration(" << topicName << ") ------\n");

  return canTransform;
}
// */

template<typename T>
void FireflyRosFilter<T>::saveFilterState(FireflyFilterBase& filter)
{
  FilterStatePtr state = FilterStatePtr(new FilterState());
  state->state_ = Eigen::VectorXd(filter.getState());
  state->estimateErrorCovariance_ = Eigen::MatrixXd(filter.getEstimateErrorCovariance());
  state->lastMeasurementTime_ = filter.getLastMeasurementTime();
  state->latestControl_ = Eigen::VectorXd(filter.getControl());
  state->latestControlTime_ = filter.getControlTime();
  filterStateHistory_.push_back(state);
  RF_DEBUG("Saved state with timestamp " << std::setprecision(20) << state->lastMeasurementTime_ <<
           " to history. " << filterStateHistory_.size() << " measurements are in the queue.\n");
}

template<typename T>
bool FireflyRosFilter<T>::revertTo(const double time)
{
  RF_DEBUG("\n----- FireflyRosFilter::revertTo -----\n");
  RF_DEBUG("\nRequested time was " << std::setprecision(20) << time << "\n")

  // Walk back through the queue until we reach a filter state whose time stamp is less than or equal to the requested time.
  // Since every saved state after that time will be overwritten/corrected, we can pop from the queue.
  while (!filterStateHistory_.empty() && filterStateHistory_.back()->lastMeasurementTime_ > time)
  {
    filterStateHistory_.pop_back();
  }

  // The state and measurement histories are stored at the same time, so if we have insufficient state history, we
  // will also have insufficient measurement history.
  if (filterStateHistory_.empty())
  {
    RF_DEBUG("Insufficient history to revert to time " << time << "\n");

    return false;
  }

  // Reset filter to the latest state from the queue.
  const FilterStatePtr &state = filterStateHistory_.back();
  filter_.setState(state->state_);
  filter_.setEstimateErrorCovariance(state->estimateErrorCovariance_);
  filter_.setLastMeasurementTime(state->lastMeasurementTime_);

  RF_DEBUG("Reverted to state with time " << state->lastMeasurementTime_ << "\n");

  // Repeat for measurements, but push every measurement onto the measurement queue as we go
  int restored_measurements = 0;
  while (!measurementHistory_.empty() && measurementHistory_.back()->time_ > time)
  {
    measurementQueue_.push(measurementHistory_.back()); // for measurementQueue, top is the latest; for history back is the latest   by ZD
    measurementHistory_.pop_back();
    restored_measurements++;
  }

  RF_DEBUG("Restored " << restored_measurements << " to measurement queue.\n");

  RF_DEBUG("\n----- /RosFilter::revertTo\n");

  return true;
}

template<typename T>
void FireflyRosFilter<T>::clearExpiredHistory(const double cutOffTime)
{
  RF_DEBUG("\n----- FireflyRosFilter::clearExpiredHistory -----" <<
           "\nCutoff time is " << cutOffTime << "\n");

  int poppedMeasurements = 0;
  int poppedStates = 0;

  while (!measurementHistory_.empty() && measurementHistory_.front()->time_ < cutOffTime)
  {
    measurementHistory_.pop_front();
    poppedMeasurements++;
  }

  while (!filterStateHistory_.empty() && filterStateHistory_.front()->lastMeasurementTime_ < cutOffTime)
  {
    filterStateHistory_.pop_front();
    poppedStates++;
  }

  RF_DEBUG("\nPopped " << poppedMeasurements << " measurements and " <<
           poppedStates << " states from their respective queues." <<
           "\n---- /RosFilter::clearExpiredHistory ----\n" );
}

}

// Instantiations of classes is required when template class code
// is placed in a .cpp file.
template class RobotLocalization::FireflyRosFilter<RobotLocalization::FireflyEkf>;
