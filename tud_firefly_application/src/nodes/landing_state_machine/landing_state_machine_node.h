#ifndef LANDING_STATE_MACHINE_NODE_H
#define LANDING_STATE_MACHINE_NODE_H

#include <ros/ros.h>
#include <ros/timer.h>
#include <ros/subscriber.h>

#include <tf/transform_listener.h>
#include <boost/thread.hpp>

#include <rosgraph_msgs/Clock.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/Actuators.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

#include <acado/acado_integrators.hpp>
#include <acado/acado_gnuplot.hpp>
#include <eigen3/unsupported/Eigen/src/Splines/SplineFwd.h>

#include <tf/transform_broadcaster.h>

#include <tf_conversions/tf_eigen.h>
#include <boost/math/special_functions/sign.hpp>
#include <boost/math/tools/polynomial.hpp>

#include <rotors_control/parameters_ros.h>
#include <tud_firefly_control/tud_firefly_common.h>
#include <tud_firefly_application/landing_predictor.h>
#include <tud_firefly_control/differential_tracker.h>
#include <tud_firefly_control/landing_controller.h>

#include <robot_localization/ros_filter_utilities.h>

namespace landing_control{

struct CurveRequest {
  inline CurveRequest(const CurveRequest& other)
  {
    type_ = other.type_;
    duration_ = other.duration_;
    endState_ = other.endState_;
    rearrange_ = other.rearrange_;
  }

  CurveRequest(const int& type,
                               const double& duration,
                               const bool& rearrange,
                               const Eigen::VectorXd& endState):
    type_(type),
    duration_(duration),
    rearrange_(rearrange)
  {
    endState_ = endState;
  }
  int type_; // type_ = mode_
  double duration_;
  Eigen::VectorXd endState_;
  bool rearrange_;
};

struct CmdCurve {
  CmdCurve(): tStart(0.0),
                            tEnd(0.0) {}
  CmdCurve(const CmdCurve& other)
  {
    tStart = other.tStart;
    tEnd = other.tEnd;
    pStart = other.pStart;
    pEnd =other.pEnd;
    curveX = other.curveX;
    curveY = other.curveY;
    curveZ = other.curveZ;
  }
  bool rearrange;
  double tEnd;
  double tStart;
  Eigen::Vector3d pStart;
  Eigen::Vector3d pEnd;
  ACADO::Curve curveX;
  ACADO::Curve curveY;
  ACADO::Curve curveZ;
};

class LandingStateMachineNode{
public:
  LandingStateMachineNode();
  ~LandingStateMachineNode();

  //! \brief Initialize function.
  //!
  void init();
  //!
  //! \brief Reset function.
  //!
  void reset();
  //!
  //! \brief Main execute function.
  //!
  void run();
  //!
  //! \brief Beginn custom control.
  //!
  void beginnCustom();
  //!
  //! \brief Shut down custom control.
  //!
  void shutdownCustom();

  //! \brief Calculate the maximum tilt angle according to the current target position
  //! to avoid losing target from the FOV.
  //!
  void calculateMaxTiltAngle();

  /// Control signal callbacks
  //! \brief Clock callback for commands.
  //! \param msg - Twist control msg from joy stick
  //!
  void clockCallback(const rosgraph_msgs::Clock& clock); // for control
  //!
  //! \brief Callback for twist control signal
  //! \param msg - Twist control msg from joy stick
  //!
  void motorStatusCallback(const std_msgs::Bool& msg);
  //!
  //! \brief Callback for twist control signal
  //! \param msg - Twist control msg from joy stick
  //!
  void landActivCallback(const std_msgs::Bool& msg);
  //!
  //! \brief Callback for completely lost.
  //! \param e - Time event.
  //!
  void stateResTimerCallback(const ros::TimerEvent& e);
  //!
  //! \brief Callback after completing land preparation.
  //! \param e - Time event.
  //!
  void landPrepTimerCallback(const ros::TimerEvent& e);
  //!
  //! \brief Callback after completing target approach.
  //! \param e - Time event.
  //!
  void approachedTimerCallback(const ros::TimerEvent& e);
  //!
  //! \brief Callback for twist control signal
  //! \param msg - Twist control msg from joy stick
  //!
  void commandTwistCallback(const geometry_msgs::Twist& msg);

  /// Sensor callbacks
  //! \brief UAV odometry callback function.
  //! \param msg - Odometry message.
  //!
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
  //!
  //! \brief UAV odometry callback function.
  //! \param msg - Odometry message.
  //!
  void odomUGVCallback(const nav_msgs::OdometryConstPtr& msg);
  //!
  //! \brief Rough target odometry callback function.
  //! \param msg - Odometry message.
  //!
  void roughTarOdomCallback(const nav_msgs::OdometryConstPtr& msg);
  //!
  //! \brief Marker callback function.
  //! \param msg - Bool msg that indices if marker is in the FOV.
  //!
  void markerCallback(const std_msgs::Bool& msg);
  //!
  //! \brief Vision module callback function, through which the most important
  //! informations from the vision module will be saved into
  //! \param msg - Msg of transformation from target frame to camera frame.
  //!
  void tar2camTransCallback(const geometry_msgs::TransformStampedConstPtr& msg);

  /// Thread function
  //! \brief Trajectory planning thread function.
  //!
  void planThread();

  /// 5 curve creation method.
  //! \brief Curve creator.
  //!
  void (LandingStateMachineNode::*createCurve_[5]) (const CurveRequest& cReq);
  //!
  //! \brief mode 0 Create curve for landing preparation.
  //! \param cReq - The curve generation request.
  //!
  void landPrepCurve(const CurveRequest& cReq);
  //!
  //! \brief mode 1 Create curve for approaching.
  //!
  void trackingCurve(const CurveRequest& cReq);
  //!
  //! \brief mode 2 Predict a curve of target for approaching.
  //!
  void predictCurve(const CurveRequest& cReq);
  //!
  //! \brief mode 3 No curve for approaching mode.
  //!
  void noCurve(const CurveRequest& cReq);
  //!
  //! \brief mode 4 Create curve for landing.
  //!
  void landingCurve(const CurveRequest& cReq);

  /// 6 control update method.
  //! \brief Function pointer array to switch between different methods.
  //!
  void (LandingStateMachineNode::*switchMethod_[6]) ();  //
  //!
  //! \brief mode 5 Custom control mode.
  //!
  void customControl();
  //!
  //! \brief mode 0 Landing process initialization mode.
  //!
  void landPreparation();
  //!
  //! \brief mode 1 Target tracking mode.
  //!
  void tarTracking();
  //!
  //! \brief mode 2 Predator prey mode.
  //!
  void tarPredicting();
  //!
  //! \brief mode 3 Approaching mode.
  //!
  void tarApproaching();
  //!
  //! \brief mode 4 Landing mode.
  //!
  void tarLanding();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
protected:
  //! \brief Predictor of the x-position in {O}-frame.
  //!
  fit::Fit predictor_x_;
  //!
  //! \brief Predictor of the y-position in {O}-frame.
  //!
  fit::Fit predictor_y_;

  std::vector<double> time_tar_;
  std::vector<double> x_tar_;
  std::vector<double> y_tar_;
  std::vector<double> cx_tar_; // Fitted polynom coefficients
  std::vector<double> cy_tar_;
  bool tar_traj_predicted_;
  double t0_, tstart_;

  rotors_control::DifferentialTracker  predictReporter_;

  //! \ Desired mean velocity for a trajectory
  //!
  // double mTrajVelDes_;

  //! \brief Curve request queue.
  //!
  std::deque<CurveRequest> curveRequests_;

  //! \brief Command curve map with initial time offset.
  //!
  //std::map<int, ACADO::Curve*> curvePtrMap_;
  std::map<int, CmdCurve*> curvePtrMap_;

  //! \brief Main nodehandle for commands.
  //!
  ros::NodeHandle nh_;
  //!
  //! \brief Nodehandle for trajectory predictor.
  //!
  ros::NodeHandle nhPlan_;
  //!
  //! \brief Parameter nodehandle.
  //!
  ros::NodeHandle nhLocal_;

  //! \brief Callback queue for prediction.
  //!
  ros::CallbackQueue planQueue_;

  //! \brief Subscriber of motor activation.
  //!
  ros::Subscriber motorSub_;
  //!
  //! \brief Subscriber of landing process activation.
  //!
  ros::Subscriber landActivSub_;
  //!
  //! \brief Subscriber of marker lost or not issue.
  //!
  ros::Subscriber markerSub_;
  //!
  //! \brief Subscriber of tranform from target to camera frame.
  //!
  ros::Subscriber tar2camSub_;
  //!
  //! \brief tarOdomSub_
  //!
  ros::Subscriber tarOdomSub_;
  //!
  //! \brief Subscriber of UAV odometry.
  //!
  ros::Subscriber odometrySub_;
  //!
  //! \brief Subscriber of rough target odometry.
  //!
  ros::Subscriber roughTarOdomSub_;
  //!
  //! \brief Subscriber of command twist of custom control.
  //!
  ros::Subscriber cmdTwistSub_;
  //!
  //! \brief Clock subscriber for update commands.
  //!
  ros::Subscriber clockSub_;

  //! \brief landedPub_
  //!
  ros::Publisher landedPub_;

  //! \brief Max tilt angle publisher.
  //!
  ros::Publisher maxTiltAnglePub_;

  //! \brief Due to the bug that geometry_msgs doesn't transform the command
  //! yaw correctly, we have decided to send the cmd through float array.
  //!
  ros::Publisher cmdYawPub_;
  //!
  //! \brief Trajectory error point publisher.
  //!
  ros::Publisher trajErrorCmdAccelYawPointPub_;
  //!
  //! \brief Trajectory error publisher.
  //!
  ros::Publisher trajErrorCmdAccelYawPub_;

  //! \brief ROS timer to count down for completely lost.
  //!
  ros::Timer stateResTimer_;

  //! \brief ROS timer to count down for completing landing preparation.
  //!
  ros::Timer landPrepTimer_;

  //! \brief approachedTimer_
  //!
  ros::Timer approachedTimer_;

  //! \brief The control algorithm.
  //!
  rotors_control::LandingController landing_controller_;

  //! \brief Differential tracker that trying to evaluate the
  //! time derivative of the relative movement between the
  //! UAV and the target from the vision sensor.
  //!
  rotors_control::DifferentialTracker visionTracker_;
  rotors_control::DifferentialTracker cmdPosTracker_;
  rotors_control::DifferentialTracker cmdVelTracker_;
  rotors_control::DifferentialTracker cmdYawTracker_;

  /// Topics
  //! \brief Topic of motor activity.
  //!
  std::string motorTopic_;
  //!
  //! \brief Topic of landed.
  //!
  std::string landedTopic_;
  //!
  //! \brief Topic of landing process activity.
  //!
  std::string landActivTopic_;
  //!
  //! \brief Topic of marker which notices that if marker is in the FOV or not.
  //!
  std::string markerTopic_;
  //!
  //! \brief Topic for marker-camera spatial transform.
  //!
  std::string tar2camTopic_;
  //!
  //! \brief Topic for update.
  //!
  std::string clockTopic_;
  //!
  //! \brief Topic of command motor velocity vector.
  //!
  std::string cmdMotorVelTopic_;
  //!
  //! \brief Topic of odometry source.
  //!
  std::string odomTopic_;
  //!
  //! \brief Topic of UGV.
  //!
  std::string tarOdomTopic_;
  //!
  //! \brief Topic of  max tilt angle topic.
  //!
  std::string maxTiltAngleTopic_;
  //!
  //! \brief Topic of command yaw.
  //!
  std::string cmdYawTopic_;
  //!
  //! \brief Topic of trajectory error point.
  //!
  std::string trajErrorCmdAccelYawPointTopic_;
  //!
  //! \brief Topic of trajectory error.
  //!
  std::string trajErrorCmdAccelYawTopic_;
  //!
  //! \brief Topic of rough odometry of target.
  //!
  std::string roughTarOdomTopic_;
  //!
  //! \brief Topic of cutom control command.
  //!
  std::string cmdTwistTopic_;
  //!
  //! \brief Target frame name.
  //!
  std::string tarFrameId_;
  //!
  //! \brief UAV frame name.
  //!
  std::string uavFrameId_;
  //!
  //! \brief {O}-frame name.
  //!
  std::string odomFrameId_;

  /// parameters for state machine
  //! \brief The time that the target has been lost.
  //!
  double time_target_lost_;
  //!
  //! \brief The duration that the target be considered as completely lost.
  //!
  double time_target_totally_lost_;
  //!
  //! \brief Maximum tilt angle in order to avoid target lost from FOV.
  //!
  double max_tilt_angle_;
  //!
  //! \brief Bool that indices if the target is in the FOV.
  //!
  bool target_in_fov_;
  //!
  //! \brief Bool that indices if the target is completely lost.
  //!
  bool target_totally_lost_;
  //!
  //! \brief Bool that indices the new marker comes.
  //!
  // bool new_marker_comes_;
  //!
  //! \brief Bool that indices if the target has the rough odometry data from
  //! the target UGV, e.g. from GPS.
  //!
  bool auto_navigation_;
  //!
  //! \brief Indice of the activity of blind tracking mode.
  //!
  bool blind_track_;
  //!
  //! \brief Indice of the activity of UAV motors.
  //!
  bool motor_activated_;
  //!
  //! \brief Indice of the activity of landing process.
  //!
  bool landing_activated_;
  //!
  //! \brief Indices whether landing preparation is finished.
  //!
  bool tracking_permitted_;
  //!
  //! \brief Indices whether target approaching process is permitted.
  //!
  bool approaching_permitted_;
  //!
  //! \brief Indices whether landing process is permitted.
  //!
  bool landing_permitted_;
  //!
  //! \brief Indices whether landing process is over.
  //!
  bool landed_;
  //!
  //! \brief Whether to implement a predator-prey scenario.
  //!
  bool predator_prey_scenario_;
  //!
  //! \brief Indices whether publish the target transformation to tf.
  //!
  bool pubTargetTf_;

  /// {W}-World frame: Reference frame of the world.
  /// {O}-Odometry frame: "World frame" estimated by the UAV.
  /// {B}-Body frame: Reference frame bound to UAV orientation.
  /// {N}-Inertia frame: Rotation bound to {O}, translation bound to {B}.
  /// {Y}-Yaw frame: Lateral rotation bound to {O}, translation and vertical
  /// rotation bound to {B}.
  /// {C}-Camera frame: Reference frame bound to camera orientation.
  /// {T}-Target frame: Reference frame bound to Target orientation.
  //! \brief T represents Target, O represents Odometry, C represents
  //! Camera. E.g. tf_C2B_C_: tranformation from target frame to camera frame expressed
  //! in {C}-frame. (camera frame)
  //!
  tf::Transform tf_C2B_C_;
  /* //!
  //! \brief tf_T2C_T_ represents the tf transformation from target
  //! frameto camera frame expressed in target frame.
  //!
  tf::StampedTransform tf_T2C_T_;
  //!
  //! \brief tranformation from target frame to {B}-frame expressed
  //! in {B}-frame.
  //!
  tf::Transform tf_T2B_B_;
  //!
  //! \brief tranformation from target frame to {O}-frame expressed
  //! in {O}-frame.
  //!
  tf::Transform tf_T2O_O_;
  //!
  //! \brief tranformation from target frame to {Y}-frame expressed
  //! in {Y}-frame.
  //!
  tf::Transform tf_T2Y_Y_;
  //!
  //! \brief tranformation from target frame to {N}-frame expressed
  //! in {N}-frame.
  //!
  tf::Transform tf_T2N_N_;  // */
  //!
  //! \brief tf broadcaster to send out all the tf type transforms for
  //! debugging.
  //!
  tf::TransformBroadcaster tfBroadcaster_;

  //! \brief Transform buffer for managing coordinate transforms
  //!
  tf2_ros::Buffer tfBuffer_;

  //! \brief Transform listener for receiving transforms
  //!
  tf2_ros::TransformListener tfListener_;

  //! \brief UAV odometry source, normally the estimated states from
  //! firefly_state_estimation_node.
  //!
  rotors_control::EigenOdometry odometry_;
  //!
  //! \brief UGV ground truth odometry source.
  //!
  rotors_control::EigenOdometry gtOdomUGV_;
  //!
  //! \brief UGV odometry source.
  //!
  rotors_control::EigenOdometry odomUGV_;
  //!
  //! \brief Target trajectory point, which is estimated transform sent by ar_sys
  //!
  mav_msgs::EigenTrajectoryPoint tarTrajPoint_;
  //!
  //! \brief Target trajectory point vector for trajectory prediction.
  //!
  std::vector<mav_msgs::EigenTrajectoryPoint> tarTrajPointVec_;
  //!
  //! \brief The relative trajectory point between target and UAV
  //!
  mav_msgs::EigenTrajectoryPoint relTrajPoint_;
  //!
  //! \brief The deque of relative trajectory point between target and UAV
  //!
  std::deque<mav_msgs::EigenTrajectoryPoint> relTrajPointDeque_;

  //! \brief UAV rotation matrix.
  //!
  Eigen::Matrix3d R_;

  //! \brief UAV heading error relative to the direction where target is located.
  //!
  // double heading_error_;
  //!
  //! \brief UAV yaw relative to the target UGV.
  //!
  // double delta_yaw_;
  //!
  //! \brief Weight constant for error_heading_ and delta_yaw_ depend on lateral
  //! distance between target and UAV.
  //!
  double yawWeight_;
  //!
  //! \brief UAV orientation data expressed as Euler angles.
  //!
  double roll_, pitch_, yaw_, latestYaw_;
  //!
  //! \brief Ratio between thrust and gravity = 1/cos(pitch).
  //!
  double load_factor_inverse_;
  //!
  //! \brief UAV orientation data expressed as quaternion, and its multiplicated values
  //!
  double q0_, q1_, q2_, q3_;
  //!
  //! \brief Norm of the orientation data
  //!
  double q_norm_;
  //!
  //! \brief Time derivative of the orientation data derived from the filter
  //!
  double dq0_, dq1_, dq2_, dq3_;
  //!
  //! \brief Multiplications of each two quaternion element for saving calculation resources
  //!
  double q0q0_, q0q1_, q0q2_, q0q3_;
  double q1q1_, q1q2_, q1q3_;
  double q2q2_, q2q3_;
  double q3q3_;

  //! \brief Control UAV position and velocity error with command acceleration and yaw.
  //!
  mav_msgs::EigenTrajectoryPoint trajErrorCmdAccelYaw_;

  //! \brief Command trajectory point.
  //!
  mav_msgs::EigenTrajectoryPoint cmdTrajPoint_;

  //! \brief Custom command trajectory point.
  //!
  mav_msgs::EigenTrajectoryPoint customTrajPoint_;

  //! \brief Rough target trajectory point.
  //!
  mav_msgs::EigenTrajectoryPoint tarRoughTrajPoint_;

  //! \brief UAV command yaw.
  //!
  // double cmd_yaw_, cmd_yaw_rate_;

  //! \brief Initial height for landing.
  //!
  double landing_initial_height_;

  //! \brief approaching_height_
  //!
  double approaching_height_;

  //! \brief Mutex for multithread command callbacks.
  //!
  boost::mutex command_mutex_;
  //!
  //! \brief Mutex for multithread measurement callbacks.
  //!
  boost::mutex measurement_mutex_;
  //!
  //! \brief Mutex for multithread curve relating actions.
  //!
  boost::mutex curve_mutex_;

  //! \brief Updating period.
  //!
  ros::Duration dt_;
  //!
  //! \brief Latest updating time.
  //!
  ros::Time t_;
  //!
  //! \brief Updating period of the target infos.
  //!
  ros::Duration dt_tar_;
  //!
  //! \brief Latest updating time of the target infos.
  //!
  ros::Time t_tar_;

  //! \brief Indice of the current state of the statemachine.
  //! 0: Landing preparation mode, 1: Tracking mode,
  //! 2: Predator-prey mode,              3: Approaching mode,
  //! 4: Landing mode,                          5: Custom control mode.
  //!
  int mode_;

  bool test_active_;
  void testCallback(const std_msgs::Bool& test);
  ros::Subscriber testSub_;

  double vertical_curve_factor_;
  double curve_factor_;

  double tTrick_;


};

}

#endif // LANDING_STATE_MACHINE_NODE_H
