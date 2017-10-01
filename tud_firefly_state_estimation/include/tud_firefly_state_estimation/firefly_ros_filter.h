#ifndef FIREFLY_EKF_LANDING_NODE_H
#define FIREFLY_EKF_LANDING_NODE_H

#include <robot_localization/ros_filter.h>
#include "tud_firefly_state_estimation/firefly_filter_common.h"
#include "tud_firefly_state_estimation/firefly_filter_base.h"

#include <tud_firefly_control/tud_firefly_common.h>
#include <rotors_control/parameters_ros.h>
#include <tud_firefly_control/differential_tracker.h>

#include <boost/bind.hpp>
#include <stdio.h>

#include <rosgraph_msgs/Clock.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/AttitudeThrust.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>

#include <ros/callback_queue.h>

namespace RobotLocalization {

template<class T>
class FireflyRosFilter
{
 public:
  //! @brief  Constructor
  //!
  //! The FireflyRosFilter constructor makes sure that anyone using
  //! this template is doing so with the correct object type
  //!
  explicit FireflyRosFilter(std::vector<double> args = std::vector<double>());

  //! @brief Destructor
  //!
  //! Clears out the message filters and topic subscribers.
  //!
  ~FireflyRosFilter();

  ////////////////////////////////////////////////////////////////////////////////////////////////////
  /// Non callback functions
  ////////////////////////////////////////////////////////////////////////////////////////////////////
  // Most functions are succeeded from RosFilter
  //! @brief Loads all parameters from file
  //!
  void loadParams();

  //! @brief Main run method
  //!
  void run();

  //! @brief Processes all measurements in the measurement queue, in temporal order
  //!
  //! @param[in] currentTime - The time at which to carry out integration (the current time)
  //!
  void integrateMeasurements(const ros::Time &currentTime);

  //! @brief Adds a measurement to the queue of measurements to be processed
  //!
  //! @param[in] topicName - The name of the measurement source (only used for debugging)
  //! @param[in] measurement - The measurement to enqueue
  //! @param[in] measurementCovariance - The covariance of the measurement
  //! @param[in] updateVector - The boolean vector that specifies which variables to update from this measurement
  //! @param[in] mahalanobisThresh - Threshold, expressed as a Mahalanobis distance, for outlier rejection
  //! @param[in] time - The time of arrival (in seconds)
  //!
  void enqueueMeasurement(const std::string &topicName,
                          const Eigen::VectorXd &measurement,
                          const Eigen::MatrixXd &measurementCovariance,
                          const std::vector<int> &updateVector,
                          const double mahalanobisThresh,
                          const ros::Time &time);

  //! @brief Method for zeroing out 3D variables within measurements
  //! @param[out] measurement - The measurement whose 3D variables will be zeroed out
  //! @param[out] measurementCovariance - The covariance of the measurement
  //! @param[out] updateVector - The boolean update vector of the measurement
  //!
  //! If we're in 2D mode, then for every measurement from every sensor, we call this.
  //! It sets the 3D variables to 0, gives those variables tiny variances, and sets
  //! their updateVector values to 1.
  //!
  void forceTwoD(Eigen::VectorXd &measurement,
                 Eigen::MatrixXd &measurementCovariance,
                 std::vector<int> &updateVector);

  //! @brief Retrieves the EKF's output for broadcasting
  //! @param[out] message - The standard ROS odometry message to be filled
  //! @return true if the filter is initialized, false otherwise
  //!
  bool getFilteredOdometryMessage(nav_msgs::Odometry &message);

  //! @brief Retrieves the EKF's acceleration output for broadcasting
  //! @param[out] message - The standard ROS acceleration message to be filled
  //! @return true if the filter is initialized, false otherwise
  //!
  //bool getFilteredAccelMessage(geometry_msgs::AccelWithCovarianceStamped &message);
  bool getFilteredAccelMessage(geometry_msgs::PointStamped &message);

  //! @brief Converts tf message filter failures to strings
  //! @param[in] reason - The failure reason object
  //! @return a string explanation of the failure
  std::string tfFailureReasonString(const tf2_ros::FilterFailureReason reason);

  ////////////////////////////////////////////////////////////////////////////////////////////////////
  ///  Callback functions
  ////////////////////////////////////////////////////////////////////////////////////////////////////
  //! @brief Callback method for receiving clock input, the function that triggers the correction
  //! @param[in] msg - The clock message to take in
  //!
  void clockCallback(const rosgraph_msgs::Clock& msg);

  //! @brief Callback method for receiving clock input.
  //! @param[in] msg - The clock message to take in
  //!
  void motorCallback(const std_msgs::Bool& msg);

  //! @brief Callback method for receiving motor status.
  //! @param[in] msg - The mode message to take in
  //!
  void modeCallback(const std_msgs::Bool& msg);

  //! @brief Callback method for receiving stamped actuator input, the function that triggers the prediction
  //! @param[in] msg - The ROS actuator message to take in
  //!
  void actuatorCallback(const mav_msgs::ActuatorsConstPtr& msg);

  void rayCallback(const sensor_msgs::Range& msg, const std::string &topicName,
                   const CallbackData &poseCallbackData, const CallbackData &twistCallbackData);

  //! @brief Callback method for receiving all odometry messages
  //! @param[in] msg - The ROS odometry message to take in.
  //! @param[in] topicName - The topic name for the odometry message (only used for debug output)
  //! @param[in] poseCallbackData - Relevant static callback data for pose variables
  //! @param[in] twistCallbackData - Relevant static callback data for twist variables
  //!
  //! This method simply separates out the pose and twist data into two new messages, and passes them into their
  //! respective callbacks
  //!
  void odometryCallback(const nav_msgs::Odometry::ConstPtr &msg, const std::string &topicName,
    const CallbackData &poseCallbackData, const CallbackData &twistCallbackData);

  //! @brief Callback method for receiving all pose messages
  //! @param[in] msg - The ROS stamped pose with covariance message to take in
  //! @param[in] callbackData - Relevant static callback data
  //! @param[in] targetFrame - The target frame_id into which to transform the data
  //! @param[in] imuData - Whether this data comes from an IMU
  //!
  void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg,
                    const CallbackData &callbackData,
                    const std::string &targetFrame,
                    const bool imuData);

  //! @brief Callback method for receiving all twist messages
  //! @param[in] msg - The ROS stamped twist with covariance message to take in.
  //! @param[in] callbackData - Relevant static callback data
  //! @param[in] targetFrame - The target frame_id into which to transform the data
  //!
  void twistCallback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr &msg,
                     const CallbackData &callbackData,
                     const std::string &targetFrame);

  //! @brief Callback method for receiving all IMU messages
  //! @param[in] msg - The ROS IMU message to take in.
  //! @param[in] topicName - The topic name for the IMU message (only used for debug output)
  //! @param[in] poseCallbackData - Relevant static callback data for orientation variables
  //! @param[in] twistCallbackData - Relevant static callback data for angular velocity variables
  //! @param[in] accelCallbackData - Relevant static callback data for linear acceleration variables
  //!
  //! This method separates out the orientation, angular velocity, and linear acceleration data and
  //! passed each on to its respective callback.
  //!
  void imuCallback(const sensor_msgs::Imu::ConstPtr &msg, const std::string &topicName,
    const CallbackData &poseCallbackData, const CallbackData &twistCallbackData,
    const CallbackData &accelCallbackData);

  //! @brief Callback method for receiving all acceleration (IMU) messages
  //! @param[in] msg - The ROS IMU message to take in.
  //! @param[in] callbackData - Relevant static callback data
  //! @param[in] targetFrame - The target frame_id into which to transform the data
  //!
  void accelerationCallback(const sensor_msgs::Imu::ConstPtr &msg,
                            const CallbackData &callbackData,
                            const std::string &targetFrame);

  //! @brief Callback method for manually setting/resetting the internal pose estimate
  //! @param[in] msg - The ROS stamped pose with covariance message to take in
  //!
  void setPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

  //! @brief Service callback for manually setting/resetting the internal pose estimate
  //!
  //! @param[in] request - Custom service request with pose information
  //! @return true if successful, false if not
  bool setPoseSrvCallback(robot_localization::SetPose::Request& request,
                          robot_localization::SetPose::Response&);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
protected:
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    ///  Protected functions
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    //! @brief Finds the latest filter state before the given timestamp and makes it the current state again.
    //!
    //! This method also inserts all measurements between the older filter timestamp and now into the measurements
    //! queue.
    //!
    //! @param[in] time - The time to which the filter state should revert
    //! @return True if restoring the filter succeeded. False if not.
    //!
    bool revertTo(const double time);

    //! @brief Saves the current filter state in the queue of previous filter states
    //!
    //! These measurements will be used in backwards smoothing in the event that older measurements come in.
    //! @param[in] filter - The filter base object whose state we want to save
    //!
    void saveFilterState(FireflyFilterBase &filter);

    //! @brief Removes measurements and filter states older than the given cutoff time.
    //! @param[in] cutoffTime - Measurements and states older than this time will be dropped.
    //!
    void clearExpiredHistory(const double cutoffTime);

    //! @brief Adds a diagnostic message to the accumulating map and updates the error level
    //! @param[in] errLevel - The error level of the diagnostic
    //! @param[in] topicAndClass - The sensor topic (if relevant) and class of diagnostic
    //! @param[in] message - Details of the diagnostic
    //! @param[in] staticDiag - Whether or not this diagnostic information is static
    //!
    void addDiagnostic(const int errLevel,
                       const std::string &topicAndClass,
                       const std::string &message,
                       const bool staticDiag);

    //! @brief Aggregates all diagnostics so they can be published
    //! @param[in] wrapper - The diagnostic status wrapper to update
    //!
    void aggregateDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &wrapper);

    //! @brief Utility method for copying covariances from ROS covariance arrays
    //! to Eigen matrices
    //!
    //! This method copies the covariances and also does some data validation, which is
    //! why it requires more parameters than just the covariance containers.
    //! @param[in] arr - The source array for the covariance data
    //! @param[in] covariance - The destination matrix for the covariance data
    //! @param[in] topicName - The name of the source data topic (for debug purposes)
    //! @param[in] updateVector - The update vector for the source topic
    //! @param[in] offset - The "starting" location within the array/update vector
    //! @param[in] dimension - The number of values to copy, starting at the offset
    //!
    void copyCovariance(const double *arr,
                        Eigen::MatrixXd &covariance,
                        const std::string &topicName,
                        const std::vector<int> &updateVector,
                        const size_t offset,
                        const size_t dimension);

    //! @brief Utility method for copying covariances from Eigen matrices to ROS
    //! covariance arrays
    //!
    //! @param[in] covariance - The source matrix for the covariance data
    //! @param[in] arr - The destination array
    //! @param[in] dimension - The number of values to copy
    //!
    void copyCovariance(const Eigen::MatrixXd &covariance,
                        double *arr,
                        const size_t dimension);

    //! @brief Loads fusion settings from the config file
    //! @param[in] topicName - The name of the topic for which to load settings
    //! @return The boolean vector of update settings for each variable for this topic
    //!
    std::vector<int> loadUpdateConfig(const std::string &topicName);

    //! @brief Prepares an IMU message's linear acceleration for integration into the filter
    //! @param[in] msg - The IMU message to prepare
    //! @param[in] topicName - The name of the topic over which this message was received
    //! @param[in] targetFrame - The target tf frame
    //! @param[in] updateVector - The update vector for the data source
    //! @param[in] measurement - The twist data converted to a measurement
    //! @param[in] measurementCovariance - The covariance of the converted measurement
    //!
    bool prepareAcceleration(const sensor_msgs::Imu::ConstPtr &msg,
                             const std::string &topicName,
                             const std::string &targetFrame,
                             std::vector<int> &updateVector,
                             Eigen::VectorXd &measurement,
                             Eigen::MatrixXd &measurementCovariance);

    //! @brief Prepares a pose message for integration into the filter
    //! @param[in] msg - The pose message to prepare
    //! @param[in] topicName - The name of the topic over which this message was received
    //! @param[in] targetFrame - The target tf frame
    //! @param[in] differential - Whether we're carrying out differential integration
    //! @param[in] relative - Whether this measurement is processed relative to the first
    //! @param[in] imuData - Whether this measurement is from an IMU
    //! @param[in,out] updateVector - The update vector for the data source
    //! @param[out] measurement - The pose data converted to a measurement
    //! @param[out] measurementCovariance - The covariance of the converted measurement
    //! @return true indicates that the measurement was successfully prepared, false otherwise
    //!
    bool preparePose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg,
                     const std::string &topicName,
                     const std::string &targetFrame,
                     const bool differential,
                     const bool relative,
                     const bool imuData,
                     std::vector<int> &updateVector,
                     Eigen::VectorXd &measurement,
                     Eigen::MatrixXd &measurementCovariance);

    //! @brief Prepares a twist message for integration into the filter
    //! @param[in] msg - The twist message to prepare
    //! @param[in] topicName - The name of the topic over which this message was received
    //! @param[in] targetFrame - The target tf frame
    //! @param[in,out] updateVector - The update vector for the data source
    //! @param[out] measurement - The twist data converted to a measurement
    //! @param[out] measurementCovariance - The covariance of the converted measurement
    //! @return true indicates that the measurement was successfully prepared, false otherwise
    //!
    bool prepareTwist(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr &msg,
                      const std::string &topicName,
                      const std::string &targetFrame,
                      std::vector<int> &updateVector,
                      Eigen::VectorXd &measurement,
                      Eigen::MatrixXd &measurementCovariance);

    //! @brief Prepares a imu message for integration into the filter
    //! @param[in] msg - The message to prepare
    //! @param[in] topicName - The name of the topic over which this message was received
    //! @param[in] targetFrame - The target tf frame
    //! @param[in,out] updateVector - The update vector for the data source
    //! @param[out] measurement - The twist data converted to a measurement
    //! @param[out] measurementCovariance - The covariance of the converted measurement
    //! @return true indicates that the measurement was successfully prepared, false otherwise
    //!
    bool prepareImu(const sensor_msgs::Imu::ConstPtr &msg,
                      const std::string &topicName,
                      const std::string &targetFrame,
                      std::vector<int> &updateVector,
                      Eigen::VectorXd &measurement,
                      Eigen::MatrixXd &measurementCovariance);


    ////////////////////////////////////////////////////////////////////////////////////////////////////
    ///  RosFilter variables
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    //! @brief tf frame name for the robot's body frame
    //!
    std::string baseLinkFrameId_;

    //! @brief Subscribes to the control input topic
    //!
    ros::Subscriber controlSub_;

    //! @brief Motor status subscriber.
    //!
    ros::Subscriber motorSub_;

    //! @brief Subscribes to the clock topic
    //!
    ros::Subscriber clockSub_;

    //! @brief Subscribes to the mode topic
    //!
    ros::Subscriber modeSub_;

    //! @brief This object accumulates static diagnostics, e.g., diagnostics relating
    //! to the configuration parameters.
    //!
    //! The values are treated as static and always reported (i.e., this object is never cleared)
    //!
    std::map<std::string, std::string> staticDiagnostics_;

    //! @brief This object accumulates dynamic diagnostics, e.g., diagnostics relating
    //! to sensor data.
    //!
    //! The values are considered transient and are cleared at every iteration.
    //!
    std::map<std::string, std::string> dynamicDiagnostics_;

    //! @brief Used for outputting debug messages
    //!
    std::ofstream debugStream_;

    //! @brief The max (worst) dynamic diagnostic level.
    //!
    int dynamicDiagErrorLevel_;

    //! @brief Used for updating the diagnostics
    //!
    diagnostic_updater::Updater diagnosticUpdater_;

    //! @brief Our filter (EKF, UKF, etc.)
    //!
    T filter_;

    //! @brief The frequency of the run loop
    //!
    double frequency_;

    //! @brief The depth of the history we track for smoothing/delayed measurement processing
    //!
    //! This is the guaranteed minimum buffer size for which previous states and measurements are kept.
    //!
    double historyLength_;

    //! @brief The most recent control input
    //!
    Eigen::VectorXd latestControl_;

    //! @brief The time of the most recent control input
    //!
    ros::Time latestControlTime_;

    //! @brief Vector to hold our subscribers until they go out of scope
    //!
    std::vector<ros::Subscriber> topicSubs_;

    //! @brief Map to hold our publishers of body linear accelerations
    //! whose effect from gravity gets removed.
    //!
    std::map<std::string, ros::Publisher> imuPubs_;

    //! @brief Stores the first measurement from each topic for relative measurements
    //!
    //! When a given sensor is being integrated in relative mode, its first measurement
    //! is effectively treated as an offset, and future measurements have this first
    //! measurement removed before they are fused. This variable stores the initial
    //! measurements. Note that this is different from using differential mode, as in
    //! differential mode, pose data is converted to twist data, resulting in boundless
    //! error growth for the variables being fused. With relative measurements, the
    //! vehicle will start with a 0 heading and position, but the measurements are still
    //! fused absolutely.
    std::map<std::string, tf2::Transform> initialMeasurements_;

    //! @brief Store the last time a message from each topic was received
    //!
    //! If we're getting messages rapidly, we may accidentally get
    //! an older message arriving after a newer one. This variable keeps
    //! track of the most recent message time for each subscribed message
    //! topic. We also use it when listening to odometry messages to
    //! determine if we should be using messages from that topic.
    //!
    std::map<std::string, ros::Time> lastMessageTimes_;

    //! @brief Store the last time set pose message was received
    //!
    //! If we receive a setPose message to reset the filter, we can get in
    //! strange situations wherein we process the pose reset, but then even
    //! after another spin cycle or two, we can get a new message with a time
    //! stamp that is *older* than the setPose message's time stamp. This means
    //! we have to check the message's time stamp against the lastSetPoseTime_.
    ros::Time lastSetPoseTime_;

    //! @brief tf frame name for the robot's map (world-fixed) frame
    //!
    std::string mapFrameId_;

    //! @brief We process measurements by queueing them up in
    //! callbacks and processing them all at once within each
    //! iteration
    //!
    MeasurementQueue measurementQueue_;

    //! @brief Node handle
    //!
    ros::NodeHandle nh_;

    //! @brief Local node handle (for params)
    //!
    ros::NodeHandle nhLocal_;

    //! @brief tf frame name for the robot's odometry (world-fixed) frame
    //!
    std::string odomFrameId_;

    //! @brief Stores the last measurement from a given topic for differential integration
    //!
    //! To carry out differential integration, we have to (1) transform
    //! that into the target frame (probably the frame specified by
    //! @p odomFrameId_), (2) "subtract"  the previous measurement from
    //! the current measurement, and then (3) transform it again by the previous
    //! state estimate. This holds the measurements used for step (2).
    //!
    std::map<std::string, tf2::Transform> previousMeasurements_;

    //! @brief We also need the previous covariance matrix for differential data
    //!
    std::map<std::string, Eigen::MatrixXd> previousMeasurementCovariances_;

    //! @brief Whether or not we print diagnostic messages to the /diagnostics topic
    //!
    bool printDiagnostics_;

    //! @brief If including acceleration for each IMU input, whether or not we remove acceleration due to gravity
    //!
    std::map<std::string, bool> removeGravitationalAcc_;

    //! @brief Subscribes to the set_pose topic (usually published from rviz). Message
    //! type is geometry_msgs/PoseWithCovarianceStamped.
    //!
    ros::Subscriber setPoseSub_;

    //! @brief Service that allows another node to change the current state and recieve a confirmation. Uses
    //! a custom SetPose service.
    //!
    ros::ServiceServer setPoseSrv_;

    //! @brief Whether or not we use smoothing
    //!
    bool smoothLaggedData_;

    //! @brief Motor status
    //!
    bool motorStatus_;

    //! @brief Contains the state vector variable names in string format
    //!
    std::vector<std::string> stateVariableNames_;

    //! @brief The max (worst) static diagnostic level.
    //!
    int staticDiagErrorLevel_;

    //! @brief Transform buffer for managing coordinate transforms
    //!
    tf2_ros::Buffer tfBuffer_;

    //! @brief Transform listener for receiving transforms
    //!
    tf2_ros::TransformListener tfListener_;

    //! @brief For future (or past) dating the world_frame->base_link_frame transform
    //!
    ros::Duration tfTimeOffset_;

    //! @brief Whether or not we're in 2D mode
    //!
    //! If this is true, the filter binds all 3D variables (Z,
    //! roll, pitch, and their respective velocities) to 0 for
    //! every measurement.
    //!
    bool twoDMode_;

    //! @brief Message that contains our latest transform (i.e., state)
    //!
    //! We use the vehicle's latest state in a number of places, and often
    //! use it as a transform, so this is the most convenient variable to
    //! use as our global "current state" object
    //!
    geometry_msgs::TransformStamped worldBaseLinkTransMsg_;

    //! @brief tf frame name that is the parent frame of the transform that this node will calculate and broadcast.
    //!
    std::string worldFrameId_;

    //! @brief Whether we publish the transform from the world_frame to the base_link_frame
    //!
    bool publishTransform_;

    //! @brief Whether we publish the acceleration
    //!
    bool publishAcceleration_;

    //! @brief An implicitly time ordered queue of past filter states used for smoothing.
    //!
    FilterStateHistoryDeque filterStateHistory_;

    //! @brief A deque of previous measurements which is implicitly ordered from earliest to latest measurement.
    //! when popped from the measurement priority queue.
    //!
    MeasurementHistoryDeque measurementHistory_;

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    /// Non Rosfilter variables
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    //! @brief Mode topic from the UAV.
    //!
    std::string modeTopic_;

    //! @brief Topic of motor status.
    //!
    std::string motorTopic_;

    //! @brief Topic of update event.
    //!
    std::string clockTopic_;

    //! @brief Topic of actuator signals.
    //!
    std::string actuatorTopic_;

    //! @brief Whether the measurement of xy position in [W] coordinate is available
    //!
    bool xyOdomAvailable_;

    //! @brief Whether the estimation of xy position odometry is activated or not
    //!
    bool estimatingXY_;

    //! @brief Whether the update delay is permitted, if not, the updateQueue_ will be cleared by every clock event
    //!
    bool permittDelay_;

    //! @brief Whether to use initialized measurement_noise_covariance or not
    //!
    bool useMeasNoiseCovariance_;

    //! @brief UAV parameters in rotors_control::VehicleParameters form.
    //!
    rotors_control::VehicleParameters UAV_parameters_;

    //! @brief Allocation matrix to calculate rotor velocities square into UAV total.
    //!  forces and torques in [B] frame
    //!
    Eigen::Matrix4Xd allocation_matrix_;

    //! @brief Whether the measurement of xy position in [W] coordinate is available.
    //!
    Eigen::Matrix4d inertia_mass_matrix_;

    //! @brief Whether the measurement of xy position in [W] coordinate is available.
    //!
    Eigen::Matrix4Xd rotor_velocities_to_UAV_accelerations_;

    //! @brief Mutex for possible multithread-designs.
    //!
    boost::mutex command_mutex_;

    //! @brief The template vector to storage actuator signals.
    //!
    Eigen::VectorXd ref_rotor_velocities;

    //! @brief Time of current update.
    //!
    ros::Time currentTime_;

    //! @brief Update queue
    //!
    std::deque<ros::Time> updateQueue_;

    //! \brief This is the transformation from imu to robot base_link, that man can initiate it,
    //! so that it is not necessary to look up the transformation in tf every time when imu msg
    //! comes.
    //!
    tf2::Transform imu2BaselinkTrans_;

    //! @brief Template measurement struct, this design is different from original Rosfilter,
    //! that all new measurements will be fused into this variable, and only this variable will be
    //! integrated into the filterBase.
    //!
    Measurement curMeasurement_;

    //! @brief This object accumulates differential trackers pointers to the configuration parameters.
    //!
    std::map<std::string, rotors_control::DifferentialTracker*> differentialTrackers_;
};

}

#endif // FIREFLY_EKF_LANDING_NODE_H
