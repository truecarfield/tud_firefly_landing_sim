#include "landing_state_machine_node.h"

namespace landing_control {

LandingStateMachineNode::LandingStateMachineNode()
  : nhLocal_("~"),
    mode_(5),
    tfListener_(tfBuffer_),
    motor_activated_(false),
    landing_activated_(false),
    auto_navigation_(false),
    approaching_permitted_(false),
    tracking_permitted_(false),
    landing_permitted_(false),
    target_in_fov_(false),
    target_totally_lost_(false),
    predator_prey_scenario_(false),
    blind_track_(false),
    pubTargetTf_(true),
    tar_traj_predicted_(false),
    landing_initial_height_(2.8),
    approaching_height_(1.0),
    yawWeight_(1.0),
    vertical_curve_factor_(1.0),
    clockTopic_("clock"),
    cmdMotorVelTopic_(mav_msgs::default_topics::COMMAND_ACTUATORS),
    cmdTwistTopic_("firefly/cmd_vel"),
    landedTopic_("firefly/landed"),
    motorTopic_("firefly/motor_status"),
    landActivTopic_("firefly/landing_activation"),
    tar2camTopic_("firefly_image_processor/transform"),
    markerTopic_("firefly_image_processor/marker_detected"),
    tarOdomTopic_("UGV_odometry"),
    odomTopic_(mav_msgs::default_topics::ODOMETRY),
    maxTiltAngleTopic_("firefly/max_tilt_angle"),
    cmdYawTopic_("firefly/traj_cmd_yaw"),
    trajErrorCmdAccelYawPointTopic_("firefly/traj_error_point"),
    trajErrorCmdAccelYawTopic_("firefly/traj_error"),
    uavFrameId_("firefly/base_link"),
    tarFrameId_("target_UGV"),
    odomFrameId_("odometry/filtered"),
    time_target_totally_lost_(5.0),
    curve_factor_(1.0),
    t_(0),
    dt_(0.001)
{
  init();
}

LandingStateMachineNode::~LandingStateMachineNode() {}

void LandingStateMachineNode::init()
{
  // Define the transformation from {C}-frame to {B}-frame.
  tf::Vector3 v(0.1, 0.02, 0.0);
  tf_C2B_C_.setOrigin(v);
  tf::Quaternion quat(0.0, 1.0, 0.0, 0.0);
  tf_C2B_C_.setRotation(quat);

  // Parameters string
  nhLocal_.param("UAV_frame_id", uavFrameId_, uavFrameId_);
  nhLocal_.param("tar_frame_id", tarFrameId_, tarFrameId_);

  // Parameters double
  nhLocal_.param("vertical_curve_factor", vertical_curve_factor_, vertical_curve_factor_);
  nhLocal_.param("curve_factor", curve_factor_, curve_factor_);
  nhLocal_.param("approaching_height", approaching_height_, approaching_height_);
  nhLocal_.param("landing_initial_height", landing_initial_height_, landing_initial_height_);
  nhLocal_.param("yaw_weight", yawWeight_, yawWeight_);
  if (yawWeight_ <= 0.0)
  {
    ROS_WARN_STREAM_NAMED("landing_state_machine","Invalid yawWeight_:"<<yawWeight_<<", replaced by "<<1.0);
    yawWeight_ = 1.0;
  }
  if (predator_prey_scenario_)
  {
    int predictor_set_size;
    nhLocal_.param("predictor_set_size", predictor_set_size, 300);
    tarTrajPointVec_.resize(predictor_set_size);
  }
  time_tar_.clear();
  x_tar_.clear();
  y_tar_.clear();
  t0_ = 0.0;

  // Parameters bool
  nhLocal_.param("auto_navigation", auto_navigation_, auto_navigation_);
  nhLocal_.param("pub_target_tf", pubTargetTf_, pubTargetTf_);
  nhLocal_.param("predator_prey_scenario", predator_prey_scenario_, predator_prey_scenario_);
  nhLocal_.param("blind_track", blind_track_, blind_track_);

  // Sub topics
  nhLocal_.param("clock_topic", clockTopic_, clockTopic_);
  nhLocal_.param("cmd_motor_vel_topic", cmdMotorVelTopic_, cmdMotorVelTopic_);
  nhLocal_.param("motor_topic", motorTopic_, motorTopic_);
  nhLocal_.param("marker_topic", markerTopic_, markerTopic_);
  nhLocal_.param("tar_2_cam_topic", tar2camTopic_, tar2camTopic_);
  nhLocal_.param("odom_topic", odomTopic_, odomTopic_);
  nhLocal_.param("tar_odom_topic", tarOdomTopic_, tarOdomTopic_);
  nhLocal_.param("rough_tar_odom_topic", roughTarOdomTopic_, roughTarOdomTopic_);
  nhLocal_.param("command_twist_topic", cmdTwistTopic_, cmdTwistTopic_);
  nhLocal_.param("land_active_topic", landActivTopic_, landActivTopic_);
  nhLocal_.param("time_target_totally_lost", time_target_totally_lost_, time_target_totally_lost_);

  // Pub topics
  nhLocal_.param("max_tilt_angle_topic", maxTiltAngleTopic_, maxTiltAngleTopic_);
  nhLocal_.param("traj_error_point_topic", trajErrorCmdAccelYawPointTopic_, trajErrorCmdAccelYawPointTopic_);
  nhLocal_.param("traj_error_topic", trajErrorCmdAccelYawTopic_, trajErrorCmdAccelYawTopic_);

  // Order subscribers
  clockSub_ = nh_.subscribe(clockTopic_, 1, &LandingStateMachineNode::clockCallback, this);
  motorSub_ = nh_.subscribe(motorTopic_, 1, &LandingStateMachineNode::motorStatusCallback, this);
  landActivSub_ = nh_.subscribe(landActivTopic_, 1, &LandingStateMachineNode::landActivCallback, this);

  // tfListener_.tf.waitForTransform("world", "marker", ros::Time(), ros::Duration(3.0));

  testSub_ = nh_.subscribe("firefly/test_activation", 1, &LandingStateMachineNode::testCallback, this);

  // Data subscribers
  stateResTimer_ = nh_.createTimer(ros::Duration(time_target_totally_lost_),
                                   &LandingStateMachineNode::stateResTimerCallback, this, true, false);  // The last two parameters are oneshot and autostart

  landPrepTimer_ = nh_.createTimer(ros::Duration(1.0), // If land preparation finished, wait for 1seconds
                                   &LandingStateMachineNode::landPrepTimerCallback, this, true, false);  // The last two parameters are oneshot and autostart

  approachedTimer_ = nh_.createTimer(ros::Duration(1.0), // If land preparation finished, wait for 1seconds
                                     &LandingStateMachineNode::approachedTimerCallback, this, true, false);  // The last two parameters are oneshot and autostart

  markerSub_ = nh_.subscribe(markerTopic_, 1, &LandingStateMachineNode::markerCallback, this);
  tar2camSub_ = nh_.subscribe<geometry_msgs::TransformStamped>(
                                tar2camTopic_, 1, &LandingStateMachineNode::tar2camTransCallback, this);
  // beginnCustom(); // Subscription of custom control signals;
  if (auto_navigation_)
  {
    roughTarOdomSub_ = nh_.subscribe(roughTarOdomTopic_, 1,
                                                &LandingStateMachineNode::roughTarOdomCallback, this);
  }

  tarOdomSub_ = nh_.subscribe(tarOdomTopic_, 1, &LandingStateMachineNode::odomUGVCallback, this);

  odometrySub_ = nh_.subscribe(odomTopic_, 1, &LandingStateMachineNode::odometryCallback, this);

  // Publishers
  landedPub_ = nh_.advertise<std_msgs::Bool>(landedTopic_, 1);

  maxTiltAnglePub_ = nh_.advertise<std_msgs::Float32>(maxTiltAngleTopic_, 1);

  cmdYawPub_ = nh_.advertise<std_msgs::Float32>(cmdYawTopic_,1);

  // trajectory error with cmd acceleration and yaw point publisher
  trajErrorCmdAccelYawPointPub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>
                                                                        (trajErrorCmdAccelYawPointTopic_, 1);
  // trajectory error with cmd acceleration and yaw point publisher
  trajErrorCmdAccelYawPub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>
                                                                        (trajErrorCmdAccelYawTopic_, 1);

  // Differential trackers
  visionTracker_.init(nhLocal_, "vision_tracker");
  cmdPosTracker_.init(nhLocal_, "cmd_position");
  cmdVelTracker_.init(nhLocal_, "cmd_velocity");
  cmdYawTracker_.init(nhLocal_, "cmd_yaw");
  predictReporter_.init(nhLocal_, "predict_reporter");

  // Allocate each main curve creating function to the corresponding mode_
  createCurve_[0] = &LandingStateMachineNode::landPrepCurve;
  createCurve_[1] = &LandingStateMachineNode::trackingCurve;
  createCurve_[2] = &LandingStateMachineNode::predictCurve;
  createCurve_[3] = &LandingStateMachineNode::noCurve;
  createCurve_[4] = &LandingStateMachineNode::landingCurve;

  // Allocate each main update function to the corresponding mode_
  switchMethod_[0] = &LandingStateMachineNode::landPreparation;
  switchMethod_[1] = &LandingStateMachineNode::tarTracking;
  switchMethod_[2] = &LandingStateMachineNode::tarPredicting;
  switchMethod_[3] = &LandingStateMachineNode::tarApproaching;
  switchMethod_[4] = &LandingStateMachineNode::tarLanding;
  switchMethod_[5] = &LandingStateMachineNode::customControl;
}

void LandingStateMachineNode::testCallback(const std_msgs::Bool& msg)
{
  test_active_ = msg.data;
}

void LandingStateMachineNode::reset()
{
  beginnCustom();

  mode_ = 5;

  max_tilt_angle_ = 0.5;

  landing_activated_ = false;
  approaching_permitted_ = false;
  tracking_permitted_ = false;
  landing_permitted_ = false;
  target_totally_lost_ = false;
  landed_ = false;
  tar_traj_predicted_ = false;

  time_tar_.clear();
  x_tar_.clear();
  y_tar_.clear();
  t0_ = 0.0;

  cmdTrajPoint_.velocity_W.setZero();
  cmdTrajPoint_.acceleration_W.setZero();

  customTrajPoint_.velocity_W.setZero();
  customTrajPoint_.setFromYawRate(0.0);

  relTrajPointDeque_.clear();
  tarTrajPointVec_.clear();

  predictor_x_.reset();
  predictor_y_.reset();

  cmdPosTracker_.reset();
  cmdVelTracker_.reset();
  cmdYawTracker_.reset();

  curveRequests_.clear();
  curvePtrMap_.clear();

  stateResTimer_.stop();
  landPrepTimer_.stop();  
}

void LandingStateMachineNode::beginnCustom()
{
  cmdTwistSub_ = nh_.subscribe(cmdTwistTopic_, 1, &LandingStateMachineNode::commandTwistCallback, this);
}

void LandingStateMachineNode::shutdownCustom()
{
  cmdTwistSub_.shutdown();
}

void LandingStateMachineNode::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
{
  boost::mutex::scoped_lock lock(measurement_mutex_);
  ROS_INFO_ONCE_NAMED("landing_state_machine", "got first UAV odometry message.");
  rotors_control::eigenOdometryFromMsg(msg, &odometry_);

  R_ = odometry_.orientation.toRotationMatrix();
  q0_ = odometry_.orientation.w();
  q1_ =  odometry_.orientation.x();
  q2_ =  odometry_.orientation.y();
  q3_ = odometry_.orientation.z();
  q0q0_ = q0_ * q0_;
  q0q1_ = q0_ * q1_;
  q0q2_ = q0_ * q2_;
  q0q3_ = q0_ * q3_;
  q1q1_ = q1_ * q1_;
  q1q2_ = q1_ * q2_;
  q1q3_ = q1_ * q3_;
  q2q2_ = q2_ * q2_;
  q2q3_ = q2_ * q3_;
  q3q3_ = q3_ * q3_;

  load_factor_inverse_ = q0q0_ - q1q1_ - q2q2_ + q3q3_;
  roll_  =  atan2(2.*(q2q3_ + q0q1_), 1. - 2.*(q2q2_ + q1q1_));
  pitch_ = -asin(2.*(q1q3_ - q0q2_));
  yaw_ =  atan2(2.*(q1q2_ + q0q3_), 1. - 2.*(q3q3_ + q2q2_)); // */
}

void LandingStateMachineNode::odomUGVCallback(const nav_msgs::OdometryConstPtr &msg)
{
  boost::mutex::scoped_lock lock(measurement_mutex_);
  ROS_INFO_ONCE_NAMED("landing_state_machine", "got first UGV odometry message.");
  rotors_control::eigenOdometryFromMsg(msg, &gtOdomUGV_);
}

void LandingStateMachineNode::roughTarOdomCallback(const nav_msgs::OdometryConstPtr &msg)
{
  boost::mutex::scoped_lock lock(command_mutex_);
  rotors_control::EigenOdometry tarOdometry;
  rotors_control::eigenOdometryFromMsg(msg, &tarOdometry);
  tarRoughTrajPoint_.position_W = tarOdometry.position;
  tarRoughTrajPoint_.orientation_W_B = tarOdometry.orientation;
}

void LandingStateMachineNode::motorStatusCallback(const std_msgs::Bool& msg)
{
  boost::mutex::scoped_lock lock(command_mutex_);
  motor_activated_ = msg.data;
  if (motor_activated_)
  {
    reset();

    /* Eigen::Vector3d tarPos;
    tarPos << 0.0, 0.0, 0.3;
    curveRequests_.push_back(CurveRequest(2, 1.0, false, tarPos));
    std::cout<<"hey!!!!!!!!!!!!!!!!"<<std::endl; //*/

    // /*
    cmdTrajPoint_.position_W(0) = odometry_.position(0);
    cmdTrajPoint_.position_W(1) = odometry_.position(1);
    cmdTrajPoint_.position_W(2) = 1.0;// + odometry_.position(2);  //*/
    tTrick_ = t_.toSec();

    cmdTrajPoint_.setFromYaw(yaw_);

    customTrajPoint_.position_W = cmdTrajPoint_.position_W;
    customTrajPoint_.setFromYaw(yaw_);

    tarTrajPoint_.setFromYaw(yaw_);
    relTrajPoint_.setFromYaw(0.0);

    ROS_INFO_NAMED("landing_state_machine","UAV motor on, mode %d activated!", mode_);
  }
  else
  {
    reset();
    ROS_INFO_NAMED("landing_state_machine","UAV motor off, mode reset to %d!", mode_);
  }
}

void LandingStateMachineNode::landActivCallback(const std_msgs::Bool &msg)
{
  boost::mutex::scoped_lock lock(command_mutex_);
  if (motor_activated_)
  {
    landing_activated_ = msg.data;
    if (landing_activated_)
    {
      if (mode_ == 5) // the mode_ is turned from 5 to 0 here only
      {
        shutdownCustom();
        mode_ = 0;

        // Create curve request
        // duration = height, which means the mean velocity is set to 1.0 m/s
        double duration;
        Eigen::Vector3d initPosition; // currentHeight;
        // If there is a rough target position data like gps etc.
        if (auto_navigation_)
        {
          duration = std::max(sqrt((tarRoughTrajPoint_.position_W(0) - odometry_.position(0))*
                                                            (tarRoughTrajPoint_.position_W(0) - odometry_.position(0))
                                                         +(tarRoughTrajPoint_.position_W(1) - odometry_.position(1))*
                                                            (tarRoughTrajPoint_.position_W(1) - odometry_.position(1))),
                                                  fabs(landing_initial_height_ - odometry_.position(2)));
          initPosition << tarRoughTrajPoint_.position_W(0),
                                        tarRoughTrajPoint_.position_W(1),
                                        landing_initial_height_; // Tracker required
        }
        else
        {
          duration = fabs(landing_initial_height_ - odometry_.position(2));
          // currentHeight.push_back(odometry_.position(2));
          initPosition << 0.0, 0.0, landing_initial_height_;
        }
       curveRequests_.push_back(CurveRequest(0, duration, false, initPosition));
      }
      ROS_INFO_NAMED("landing_state_machine","Landing process activated, mode = %d!", mode_); //*/
    }
    else
    {
      // cancel the landing process at any condition if landing_process_permitted == 0.
      // I think this one should be added to the main circle for every iteration.
      reset();
      ROS_INFO_NAMED("landing_state_machine","Landing process deactivated, mode = %d!", mode_);
    }
  }
  else
  {
    reset();
  }
}

void LandingStateMachineNode::markerCallback(const std_msgs::Bool& msg)
{
  boost::mutex::scoped_lock lock(measurement_mutex_);
  target_in_fov_ = msg.data;
  if (target_in_fov_)
  {
      ROS_INFO_NAMED("landing_state_machine","Target detected, vertical distance is %f. ", odometry_.position(2) - tarTrajPoint_.position_W(2));
  }
  else
  {
    if (landing_activated_)
    {
      // ROS_INFO_NAMED("landing_state_machine",
      //               "Target get lost from the FOV while landing activated, odometry reset in 5 seconds!");
      ROS_INFO_NAMED("landing_state_machine",
                     "Target get lost, vertical distance is %f. ", odometry_.position(2) - tarTrajPoint_.position_W(2));
      stateResTimer_.start();
    }
    else
    {
      ROS_INFO_NAMED("landing_state_machine","Target get lost while landing activated, odometry reset in 5 seconds!, vertical distance is %f. ",
                     odometry_.position(2) - tarTrajPoint_.position_W(2));
    }
  }
}

void LandingStateMachineNode::stateResTimerCallback(const ros::TimerEvent& e)
{
  boost::mutex::scoped_lock lock(measurement_mutex_);
  target_totally_lost_ = true;
  if (mode_ == 0 || mode_ == 1)
  {
    reset();
  }
  ROS_INFO_NAMED("landing_state_machine", "Target UGV get completely lost from the FOV, state estimation module reset!");

  // here must also include the reset of the state estimation

  stateResTimer_.stop();
  target_totally_lost_ = false; // here I want target_totally_lost_ reset right after prediction and state estimation reset
}

void LandingStateMachineNode::landPrepTimerCallback(const ros::TimerEvent &e)
{
  boost::mutex::scoped_lock lock(measurement_mutex_);
  tracking_permitted_ = true;
}

void LandingStateMachineNode::approachedTimerCallback(const ros::TimerEvent &e)
{
  boost::mutex::scoped_lock lock(measurement_mutex_);
  landing_permitted_ = true;
}

void LandingStateMachineNode::tar2camTransCallback(const geometry_msgs::TransformStampedConstPtr& msg)
{
  boost::mutex::scoped_lock lock(measurement_mutex_);

  /* // 1. Save msg and transform it into {B} frame.
  tf::StampedTransform tf_T2C_T;
  tf::transformStampedMsgToTF(*msg, tf_T2C_T);
  tf::Transform tf_T2B_B(tf_C2B_C_*tf_T2C_T);

  // 2. Transform the tf_T2B_B into data that can be saved in tarTrajectoryPoint and relTrajPoint.
  // 2.a. Firstly lets dual with target position and orientation, and relative position.
  // Calculate the target tf into {W}-frame.
  tf::Vector3 v_uav;
  tf::vectorEigenToTF(odometry_.position, v_uav);
  tf::Quaternion q_uav;
  tf::quaternionEigenToTF(odometry_.orientation, q_uav);

  // Calculate the transform from target frame to {B}-frame expressed in {N}-frame.
  tf::Transform tf_B2N_N(q_uav, tf::Vector3(0.0, 0.0, 0.0));
  tf::Transform tf_T2N_N(tf_B2N_N*tf_T2B_B);

  // Transform the tf data into eigen.
  Eigen::Vector3d tar_position_W;
  Eigen::Vector3d minus_rel_position_W;
  Eigen::Quaterniond tar_orientation_W_B;
  tf::vectorTFToEigen(tf::Transform(q_uav, v_uav)*tf_T2B_B.getOrigin(), tar_position_W);
  tf::quaternionTFToEigen(tf::Transform(q_uav, v_uav)*tf_T2B_B.getRotation(), tar_orientation_W_B);
  tf::vectorTFToEigen(tf_T2N_N.getOrigin(), minus_rel_position_W);

  // 2.b. Now get the yaw_error for control.
  // Calculate the transform from target frame to {Y}-frame expressed in {Y}-frame.
  tf::Quaternion q_uav_xy;
  q_uav_xy.setRPY(roll_, pitch_, 0.0);
  tf::Transform tf_B2Y_Y(q_uav_xy, tf::Vector3(0.0, 0.0, 0.0));
  tf::Transform tf_T2Y_Y = tf_B2Y_Y*tf_T2B_B;

  // Reference direction of the target in {Y}-frame
  double heading_error = -atan2(tf_T2Y_Y.getOrigin().y(), tf_T2Y_Y.getOrigin().x());
  // yaw of UAV in target frame, which is yaw_uav - yaw_target in {W}-frame.
  double delta_yaw  = tf::getYaw(tf_T2B_B.inverse().getRotation());
  // std::cout<<"error_heading:"<<error_heading_<<std::endl;

  // !!! Weigh the two value above and derive the final yaw_error for control.
  double tf_T2Y_Ynorm_xy_ = sqrt(tf_T2Y_Y.getOrigin().x()*tf_T2Y_Y.getOrigin().x()
                                                                   + tf_T2Y_Y.getOrigin().y()*tf_T2Y_Y.getOrigin().y());
  double yaw_error = // heading_error*(1 - exp(-tf_T2Y_Ynorm_xy_/yawWeight_)) +
                                          yaw_ - 0.0; //delta_yaw;// * exp(-tf_T2Y_Ynorm_xy_/yawWeight_);

  // Estimate the relative velocity between the target and the UAV in {N}-frame. */

  // std::cout<<"xtar:"<< gtOdomUGV_.position(0)<<", ytar:"<< gtOdomUGV_.position(1)<<std::endl;
  mav_msgs::EigenTrajectoryPoint relTrajP;
  relTrajP.position_W = odometry_.position - gtOdomUGV_.position;
  // relTrajP.setFromYaw(0.0);
  relTrajP.orientation_W_B = odometry_.orientation*gtOdomUGV_.orientation.inverse();

  // Reference direction of the target in {Y}-frame
  Eigen::Vector2d rel_position_Y;
  rel_position_Y << -cos(yaw_)*(relTrajP.position_W(0)) - sin(yaw_)*(relTrajP.position_W(1)),
                                       sin(yaw_)*(relTrajP.position_W(0)) - cos(yaw_)*(relTrajP.position_W(1));

  double heading_error = -atan2(rel_position_Y(1), rel_position_Y(0));
  // yaw of UAV in target frame, which is yaw_uav - yaw_target in {W}-frame.
  double delta_yaw  = relTrajP.getYaw();

  // !!! Weigh the two value above and derive the final yaw_error for control.
  double tf_T2Y_Ynorm_xy_ = sqrt(relTrajP.position_W(0)*relTrajP.position_W(0)
                                                                   + relTrajP.position_W(1)*relTrajP.position_W(1));
  double yaw_error = // heading_error*(1 - exp(-tf_T2Y_Ynorm_xy_/yawWeight_)) +
                                         0.0;// delta_yaw;// * exp(-tf_T2Y_Ynorm_xy_/yawWeight_);

  ros::Duration dt_tar = msg->header.stamp - t_tar_;

  Eigen::VectorXd tar_velocity_W(3), rel_velocity_W(3);
  if (dt_tar.toSec() > 0.5)
  {
    tar_velocity_W.setZero();
    rel_velocity_W.setZero();
    relTrajPoint_.velocity_W.setZero();
    // clear the target odometry deque if vision module time out.
  }
  else
  {
    // visionTracker_.track(-minus_rel_position_W, rel_velocity_W, dt_tar.toSec());
    relTrajPoint_.velocity_W = (relTrajP.position_W - relTrajPoint_.position_W)/dt_tar.toSec();
  }
  relTrajPoint_.position_W = relTrajP.position_W;
  relTrajPoint_.setFromYaw(0.0);

  tarTrajPoint_.position_W << odometry_.position(0) - relTrajPoint_.position_W(0),
                                                           odometry_.position(1) - relTrajPoint_.position_W(1),
                                                           0.15;
  tarTrajPoint_.velocity_W << gtOdomUGV_.orientation.toRotationMatrix()*gtOdomUGV_.velocity;
  tarTrajPoint_.orientation_W_B = odometry_.orientation*relTrajP.orientation_W_B.inverse();

  /*std::cout<<"xb:"<<rel_position_Y(0)
                   <<", yb:"<<rel_position_Y(1)
                   <<", error_heading:"<<heading_error
                   <<", delta_yaw:"<<delta_yaw
                   <<", yaw_error:"<<yaw_error
                   <<", yaw_error_f:"<<mav_msgs::yawFromQuaternion(yaw_error_quat_f)<<std::endl; // */

  t_tar_ = msg->header.stamp;

  // Stop the state reset timer.
  stateResTimer_.stop();
  relTrajPointDeque_.push_back(relTrajPoint_);

  /* // For debug
  if (pubTargetTf_)
  {
    tfBroadcaster_.sendTransform(
          tf::StampedTransform(tf_T2B_B, ros::Time::now(), uavFrameId_, tarFrameId_));
    tfBroadcaster_.sendTransform(
          tf::StampedTransform(tf::Transform(q_uav, tf::Vector3(0.0, 0.0, 0.0)).inverse(), ros::Time::now(), uavFrameId_, "Inertia"));
    tfBroadcaster_.sendTransform(
          tf::StampedTransform(tf::Transform(q_uav_xy, tf::Vector3(0.0, 0.0, 0.0)).inverse(), ros::Time::now(), uavFrameId_, "Yaw"));
    tfBroadcaster_.sendTransform(
          tf::StampedTransform(tf_T2C_T, ros::Time::now(), "firefly/camera_1_optical_frame", "Target_C"));
    tfBroadcaster_.sendTransform(
          tf::StampedTransform(tf_T2Y_Y, ros::Time::now(), "Yaw", "Target_Y"));
  } // */
}

void LandingStateMachineNode::calculateMaxTiltAngle()
{
  double l = relTrajPoint_.position_W.norm();
  double lz = relTrajPoint_.position_W(2);
  double temp = 0.785 - acos(lz/l);
  temp = std::max(0.0, temp);
  temp = std::min(0.6, temp);
  max_tilt_angle_ = temp;
  // std::cout<<"l:"<<l<<", lz:"<<lz<<", max tilt:"<<max_tilt_angle_<<std::endl;
}

void LandingStateMachineNode::commandTwistCallback(const geometry_msgs::Twist &msg)
{
  boost::mutex::scoped_lock lock(measurement_mutex_);
  customTrajPoint_.velocity_W << 2.0*((0.5 - q3q3_)*msg.linear.x - q0q3_*msg.linear.y),
                                                                         2.0*(q0q3_*msg.linear.x + (0.5 - q3q3_)*msg.linear.y),
                                                                         msg.linear.z;  // */

  customTrajPoint_.setFromYawRate(msg.angular.z);
}

void LandingStateMachineNode::customControl()
{
  // set error.
  //cmdTrajPoint_.position_W += customTrajPoint_.velocity_W*dt_.toSec();
  /* if (test_active_)
  {
    /* if (t_.toSec() > 9.0)
    {
      //cmdTrajPoint_.position_W(0) = 2.0;
      //cmdTrajPoint_.position_W(1) = 2.0;
    }
    else if (t_.toSec() > 6.0)
    {
      //cmdTrajPoint_.position_W(1) = 1.0;
    }
   if (t_.toSec() > 3.0)
    {
      cmdTrajPoint_.position_W(0) = 3.0; //*(t_.toSec()-3.0); // 2.0*cos(0.943*t_.toSec()); //
      cmdTrajPoint_.position_W(1) = 3.0; //2.0*sin(0.943*t_.toSec()); //
      cmdTrajPoint_.position_W(2) = 3.0; //0.2*t_.toSec();
    }
  } //*/

  // Set command yaw
  if (!relTrajPointDeque_.empty())
  {
    relTrajPointDeque_.pop_front();
  }

  double cmd_yaw = customTrajPoint_.getYaw();
  cmd_yaw += customTrajPoint_.getYawRate()*dt_.toSec();
  cmd_yaw = rotors_control::clampRotation(cmd_yaw);

  customTrajPoint_.setFromYaw(cmd_yaw);
  cmdTrajPoint_.setFromYaw(cmd_yaw);

  // std::cout<<"cmd Px:"<<cmdTrajPoint_.position_W(0)<<", Py:"<<cmdTrajPoint_.position_W(1)<<", Pz:"<<cmdTrajPoint_.position_W(2)<<std::endl;
}

void LandingStateMachineNode::landPreparation()
{
  // Initialize the UAV height for the vision module, we apply the cmdTrajPoint_
  if (!curvePtrMap_.count(0) == 0)
  {
     //
    double now = t_.toSec();
    if (now > curvePtrMap_[0]->tEnd)
    {
      cmdTrajPoint_.position_W(2) = landing_initial_height_;
      curvePtrMap_.erase(0);

      if (predator_prey_scenario_)
      {
        mode_ = 2;
        ROS_INFO_NAMED("landing_state_machine","Landing preparation successful, start target prediction!, mode set to %d!", mode_);
      }
    }
    else
    {
      if (auto_navigation_)
      {
        curvePtrMap_[0]->curveX.evaluate(now, &cmdTrajPoint_.position_W(0));
        curvePtrMap_[0]->curveY.evaluate(now, &cmdTrajPoint_.position_W(1));
      }
      curvePtrMap_[0]->curveZ.evaluate(now, &cmdTrajPoint_.position_W(2));
    }
  } // */
  /*
  cmdTrajPoint_.position_W(2) = 1.0;
  if (test_active_)
  {
    cmdTrajPoint_.position_W(0) = 1.0;
  } //*/

  if (!target_in_fov_)
  {
    time_tar_.clear();
    x_tar_.clear();
    y_tar_.clear();
  }

  // Set command yaw
  if (!relTrajPointDeque_.empty() && mode_ == 0)
  {
    // Target prediction
    time_tar_.push_back(t_.toSec());
    x_tar_.push_back(tarTrajPoint_.position_W(0));
    y_tar_.push_back(tarTrajPoint_.position_W(1));

    if (time_tar_.size() >= 50 && !tar_traj_predicted_)
    {
      for (int i = 0;i<time_tar_.size();i++)
      {
        t0_ = time_tar_[0];
        time_tar_[i] -= t0_;
      }
      predictor_x_.polyfit<double>(time_tar_, x_tar_, 4, true);
      predictor_x_.getFactor(cx_tar_);
      std::cout<<"Trajectory quality x is:"<<predictor_x_.getR_square()<<std::endl;
      predictor_y_.polyfit<double>(time_tar_, y_tar_, 4, true);
      predictor_y_.getFactor(cy_tar_);
      std::cout<<"Trajectory quality y is:"<<predictor_y_.getR_square()<<std::endl;

      tstart_ = t_.toSec();
      tar_traj_predicted_ = true;
    }

    // cmd yaw = odom yaw - rel yaw = odom yaw - (odom yaw - cmd yaw).
    mav_msgs::EigenTrajectoryPoint relTrajP = relTrajPointDeque_.front();
    mav_msgs::EigenTrajectoryPoint tarTrajP = tarTrajPoint_;

    cmdTrajPoint_.setFromYaw(tarTrajP.getYaw());
    // std::cout<<"cmd_yaw calcu:"<< relTrajP.getYaw()<<std::endl;

    calculateMaxTiltAngle();

    /*
    std::cout<<"pos error:"<<trajErrorCmdAccelYaw_.position_W.norm()
                     <<", yaw error:"<<fabs(relTrajP.getYaw())
                     <<", cos(tilt):"<<acos(load_factor_inverse_)
                     <<std::endl; // */
    if (trajErrorCmdAccelYaw_.position_W.norm() < 0.1
        && fabs(relTrajP.getYaw()) < 0.1
        && acos(load_factor_inverse_) < max_tilt_angle_  // load_factor_inverse = cos(tilt_angle)
        && tar_traj_predicted_
        )
    {
      landPrepTimer_.start();

      if (tracking_permitted_)
      {
        mode_ = 3; // 1: cubic trajectory tracking; 3: target tracking

        // Create curve request for mode 1
        Eigen::Vector3d tarPosition;
        tarPosition << tarTrajP.position_W(0), tarTrajP.position_W(1), approaching_height_;
        double duration = curve_factor_*sqrt(relTrajP.position_W(0)*relTrajP.position_W(0)
                                                   + relTrajP.position_W(1)*relTrajP.position_W(1));
        curveRequests_.push_back(CurveRequest(1, duration, false, tarPosition));  // */

        tracking_permitted_ = false;
        ROS_INFO_NAMED("landing_state_machine","Landing preparation successful, start tracking!, mode set to %d!", mode_);
      }
    }
    relTrajPointDeque_.pop_front();
  }
}

void LandingStateMachineNode::tarTracking() // not trajectory tracking
{
  // /*
  if (!curvePtrMap_.count(1) == 0)
  {
    double now = t_.toSec();
    if (now > curvePtrMap_[1]->tEnd)
    {
      curvePtrMap_.erase(1);
      mode_ = 3;
      ROS_INFO_NAMED("landing_state_machine",
                     "Tracking completed, start approaching without trajectory, mode switched to %d!", mode_);

      if (!relTrajPointDeque_.empty())
      {
        calculateMaxTiltAngle();

        // cmd yaw = odom yaw - rel yaw = odom yaw - (odom yaw - cmd yaw).
        mav_msgs::EigenTrajectoryPoint relTrajP = relTrajPointDeque_.front();
        mav_msgs::EigenTrajectoryPoint tarTrajP = tarTrajPoint_;

        cmdTrajPoint_.position_W(0) = tarTrajP.position_W(0);
        cmdTrajPoint_.position_W(1) = tarTrajP.position_W(1);
        cmdTrajPoint_.setFromYaw(tarTrajP.getYaw());
        // std::cout<<"cmd_yaw calcu:"<< relTrajP.getYaw()<<std::endl;

        relTrajPointDeque_.pop_front();
      }
      cmdTrajPoint_.position_W(2) = approaching_height_;
    }
    else
    {
      double cmdX, cmdY, cmdZ;
      curvePtrMap_[1]->curveX.evaluate(now, &cmdX);
      curvePtrMap_[1]->curveY.evaluate(now, &cmdY);
      curvePtrMap_[1]->curveZ.evaluate(now, &cmdZ);
      cmdTrajPoint_.position_W(0) = cmdX;
      cmdTrajPoint_.position_W(1) = cmdY;
      cmdTrajPoint_.position_W(2) = cmdZ;

      // If target is in the FOV and target position is changing over 5 cm.
      if (!relTrajPointDeque_.empty())
      {
        mav_msgs::EigenTrajectoryPoint tarTrajP = tarTrajPoint_;
        double lateral_traj_error =
            sqrt((tarTrajP.position_W(0) - curvePtrMap_[1]->pEnd(0))
                   *(tarTrajP.position_W(0) - curvePtrMap_[1]->pEnd(0))
                   +(tarTrajP.position_W(1) - curvePtrMap_[1]->pEnd(1))
                   *(tarTrajP.position_W(1) - curvePtrMap_[1]->pEnd(1)));
        if (lateral_traj_error > 0.05)
        {
          calculateMaxTiltAngle();
          std::cout<<"Rearranging curve, with latest position error:"<<lateral_traj_error<<std::endl;

          // Rearange the curve.
          Eigen::Vector3d tarPosition;
          tarPosition << tarTrajP.position_W(0), tarTrajP.position_W(1), approaching_height_;
          curveRequests_.push_back(CurveRequest(1, 0.0, true, tarPosition));  // */
        }
        relTrajPointDeque_.pop_front();
      }
    }
  }
  else
  {
    if (!relTrajPointDeque_.empty())
    {
      relTrajPointDeque_.pop_front();
    }
  }
}

void LandingStateMachineNode::tarPredicting()
{
  // fill the tarTrajPointVec_ here,
  // If there is enough data, then send a request for prediction,
  // Wait the prediction,
  // Then track the predicted trajectory.

  if (true)
  {
    // Depend on the predict tracking result, go to mode 1 or 3
    // good -> mode_ = 1;
    // bad -> mode_ = 3;
  }
}

void LandingStateMachineNode::tarApproaching()
{
  double x_pre, y_pre;
  Eigen::VectorXd pre(2);
  if (t_.toSec() - tstart_ < 3.0)
  {
    for (int i=0;i<cx_tar_.size();i++)
    {
      x_pre += cx_tar_[i]*pow(t_.toSec() - t0_, i);
    }
    for (int i=0;i<cy_tar_.size();i++)
    {
      y_pre += cy_tar_[i]*pow(t_.toSec() - t0_, i);
    }
    pre(0) = x_pre; pre(1) = y_pre;
  }
  else
  {
    pre << 0.0, 0.0;
  }
  predictReporter_.reportX(pre);

  // /*
  if (!relTrajPointDeque_.empty())
  {
    calculateMaxTiltAngle();

    // cmd yaw = odom yaw - rel yaw = odom yaw - (odom yaw - cmd yaw).
    mav_msgs::EigenTrajectoryPoint relTrajP = relTrajPointDeque_.front();
    mav_msgs::EigenTrajectoryPoint tarTrajP = tarTrajPoint_;

    cmdTrajPoint_.position_W(0) = tarTrajP.position_W(0);
    cmdTrajPoint_.position_W(1) = tarTrajP.position_W(1);
    cmdTrajPoint_.setFromYaw(tarTrajP.getYaw());
    // std::cout<<"cmd_yaw calcu:"<< relTrajP.getYaw()<<std::endl;

    /*
    std::cout<<"pos error:"<<relTrajP.position_W(0)*relTrajP.position_W(0) + relTrajP.position_W(1)*relTrajP.position_W(1)
                     <<", yaw error:"<<fabs(relTrajP.getYaw())
                     <<", cos(tilt):"<<acos(load_factor_inverse_)
                     <<std::endl; // */
    if (sqrt(relTrajP.position_W(0)*relTrajP.position_W(0) + relTrajP.position_W(1)*relTrajP.position_W(1)) < 0.1
         && fabs(relTrajP.getYaw()) < 0.05 // about 2.86°
         && acos(load_factor_inverse_) < 0.2  // load_factor_inverse = cos(tilt_angle)
        )
    {
      approachedTimer_.start();

      if (landing_permitted_)
      {
        mode_ = 4;

        // Create curve request for mode 1
        Eigen::VectorXd tarPosition(3);
        tarPosition << 0.0, 0.0, 0.2;
        double duration = 0.85;
        curveRequests_.push_back(CurveRequest(4, duration, false, tarPosition));  // */

        landing_permitted_ = false;
        ROS_INFO_NAMED("landing_state_machine","Approaching successful, start landing!, mode set to %d!", mode_);
      }
    }

    relTrajPointDeque_.pop_front();
  } // */
  else
  {

    if (t_.toSec() - tstart_ < 3.0)
    {
      cmdTrajPoint_.position_W(0) = x_pre;
      cmdTrajPoint_.position_W(1) = y_pre;
    }
    else
    {
      cmdTrajPoint_.position_W(0) = gtOdomUGV_.position(0);
      cmdTrajPoint_.position_W(1) = gtOdomUGV_.position(1);
    } //*/
    //cmdTrajPoint_.position_W(0) = gtOdomUGV_.position(0);
    //cmdTrajPoint_.position_W(1) = gtOdomUGV_.position(1);  // */
  }
  cmdTrajPoint_.position_W(2) = approaching_height_;
}

void LandingStateMachineNode::tarLanding()
{
  // Initialize the UAV height for the vision module, we apply the cmdTrajPoint_
  if (!curvePtrMap_.count(4) == 0)
  {
    // If
    double now = t_.toSec();
    if (now > curvePtrMap_[4]->tEnd)
    {
      cmdTrajPoint_.position_W(2) = 0.2;
      curvePtrMap_.erase(4);
    }
    else
    {
      curvePtrMap_[4]->curveZ.evaluate(now, &cmdTrajPoint_.position_W(2));
    }
  }

  // Set command yaw
  // /*
  if (!relTrajPointDeque_.empty())
  {
    calculateMaxTiltAngle();

    // cmd yaw = odom yaw - rel yaw = odom yaw - (odom yaw - cmd yaw).
    mav_msgs::EigenTrajectoryPoint relTrajP = relTrajPointDeque_.front();
    mav_msgs::EigenTrajectoryPoint tarTrajP = tarTrajPoint_;

    cmdTrajPoint_.position_W(0) = tarTrajP.position_W(0);
    cmdTrajPoint_.position_W(1) = tarTrajP.position_W(1);
    cmdTrajPoint_.setFromYaw(tarTrajP.getYaw());
    // std::cout<<"cmd_yaw calcu:"<< relTrajP.getYaw()<<std::endl;

    relTrajPointDeque_.pop_front();
  } // */
  else
  {
    cmdTrajPoint_.position_W(0) = gtOdomUGV_.position(0);
    cmdTrajPoint_.position_W(1) = gtOdomUGV_.position(1);
  } //*/

  // /*
  //std::cout<<"height error:"<<((fabs(0.2 - odometry_.position(2))))<<", norm:"<<(cmdTrajPoint_.position_W - odometry_.position).norm()<<", xy_norm:"<<
  //           sqrt((cmdTrajPoint_.position_W(0) - odometry_.position(0))*(cmdTrajPoint_.position_W(0) - odometry_.position(0))
  //                       + (cmdTrajPoint_.position_W(1) - odometry_.position(1))*(cmdTrajPoint_.position_W(1) - odometry_.position(1)))<<std::endl; //*/

  if ((fabs(0.2 - odometry_.position(2)) < 0.08
       &&  sqrt((cmdTrajPoint_.position_W(0) - odometry_.position(0))*(cmdTrajPoint_.position_W(0) - odometry_.position(0))
             + (cmdTrajPoint_.position_W(1) - odometry_.position(1))*(cmdTrajPoint_.position_W(1) - odometry_.position(1))) < 0.08)
       )//&& !target_in_fov_)
  {
    std_msgs::Bool landed_msg;
    landed_msg.data = true;
    landedPub_.publish(landed_msg);

    landed_ = true;
    ROS_INFO_NAMED("landing_state_machine","Landed!");
  }
  // */
}

void LandingStateMachineNode::clockCallback(const rosgraph_msgs::Clock& clock)
{
  boost::mutex::scoped_lock lock1(command_mutex_);
  boost::mutex::scoped_lock lock2(measurement_mutex_);
  dt_ = clock.clock - t_;
  t_ = clock.clock;

  if (motor_activated_)
  {
    // Switching mode.
    // 0: Landing process initialization mode, 1: Preparation mode,
    // 2: Approaching mode,                                  3: Landing mode.
    // 4: Custom
    (this->*switchMethod_[mode_])();

    /*cmdTrajPoint_.position_W << 2.0*cos(0.943*(t_.toSec() - tTrick_)),
                                                               2.0*sin(0.943*(t_.toSec() - tTrick_)),
                                                               0.2*(t_.toSec() - tTrick_);
    //cmdTrajPoint_.setFromYaw( atan2(1.8*cos(0.943*(t_.toSec() - tTrick_)) , -1.8*sin(0.943*(t_.toSec() - tTrick_)))  ); */
    // Use differential filters filter all command signals we have.
    Eigen::VectorXd cmd_pos_W_f(3), cmd_vel_W_f(3), cmd_vel_W_f_f(3), cmd_accel_W_f(3);
    cmd_pos_W_f = odometry_.position;
    cmdPosTracker_.track(cmdTrajPoint_.position_W, cmd_pos_W_f, cmd_vel_W_f, dt_.toSec());

    cmd_vel_W_f_f = cmd_vel_W_f;
    cmdVelTracker_.track(cmd_vel_W_f, cmd_vel_W_f_f, cmd_accel_W_f, dt_.toSec());
    cmdTrajPoint_.velocity_W = cmd_vel_W_f_f;
    cmdTrajPoint_.acceleration_W = cmd_accel_W_f;

    Eigen::Matrix3d cmd_R(cmdTrajPoint_.orientation_W_B.toRotationMatrix()), cmd_R_f, dot_cmd_R_f;
    Eigen::VectorXd cmd_R_vec(9), cmd_R_f_vec(9), dot_cmd_R_f_vec(9);
    for (int i=0;i<3;i++)
    {
      for (int j=0;j<3;j++)
      {
        cmd_R_vec(3*i+j) = cmd_R(i, j);
      }
    }
    cmd_R_f_vec = cmd_R_vec;
    cmdYawTracker_.track(cmd_R_vec, cmd_R_f_vec, dot_cmd_R_f_vec, dt_.toSec());
    for (int i=0;i<3;i++)
    {
      for (int j=0;j<3;j++)
      {
        cmd_R_f(i, j) = cmd_R_f_vec(3*i+j);
        dot_cmd_R_f(i, j) = dot_cmd_R_f_vec(3*i+j);
      }
    }

    // Set trajectory position and velocity error.
    trajErrorCmdAccelYaw_.position_W = odometry_.position - cmd_pos_W_f;
    trajErrorCmdAccelYaw_.velocity_W = R_*odometry_.velocity - cmdTrajPoint_.velocity_W;
    //trajErrorCmdAccelYaw_.position_W = odometry_.position - cmdTrajPoint_.position_W;
    //trajErrorCmdAccelYaw_.velocity_W = R_*odometry_.velocity - cmd_vel_W_f;
    trajErrorCmdAccelYaw_.acceleration_W = cmd_accel_W_f;
    trajErrorCmdAccelYaw_.setFromYaw(atan2(cmd_R_f(1, 0), cmd_R_f(0, 0)));   // */
    trajErrorCmdAccelYaw_.setFromYawRate(atan2(dot_cmd_R_f(1, 0), dot_cmd_R_f(0, 0)));   // */

    // Publish the results.
    std_msgs::Float32 max_tilt_angle_msg;
    max_tilt_angle_msg.data = max_tilt_angle_;
    maxTiltAnglePub_.publish(max_tilt_angle_msg);

    trajectory_msgs::MultiDOFJointTrajectoryPoint trajErrorCmdAccelYawMsg;
    mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajErrorCmdAccelYaw_, &trajErrorCmdAccelYawMsg);

    // Bug found, if trajectoryPoint
    std_msgs::Float32 yaw_msg;

    yaw_msg.data = trajErrorCmdAccelYaw_.getYaw();
    // std::cout<<"cmd_yaw:"<<trajErrorCmdAccelYaw_.getYaw()<<", cmd_msg_yaw:"<<yaw_msg.data<<std::endl;

    /* mav_msgs::EigenTrajectoryPoint trajErrorCmdAccelYaw;
    mav_msgs::eigenTrajectoryPointFromMsg(trajErrorCmdAccelYawMsg, &trajErrorCmdAccelYaw);
    Eigen::Quaterniond a( yaw_quat_msg.data[0], yaw_quat_msg.data[1], yaw_quat_msg.data[2], yaw_quat_msg.data[3]);
    geometry_msgs::Quaternion q_msg;
    mav_msgs::quaternionEigenToMsg(trajErrorCmdAccelYaw_.orientation_W_B, &q_msg);
    std::cout<<"trajCmdYaw:"<<trajErrorCmdAccelYaw_.getYaw()<<", floatarray:"<<mav_msgs::yawFromQuaternion(a)<<", downloaded:"<<trajErrorCmdAccelYaw.getYaw()<<std::endl;
    std::cout<<"cyaw:"<<yaw_msg.data<<", getYaw:"<<trajErrorCmdAccelYaw_.getYaw()<<std::endl; //*/
    cmdYawPub_.publish(yaw_msg);

    // std::cout<<"trajError:"<<trajErrorCmdAccelYawMsg.transforms.data()->translation.z<<std::endl;
    trajErrorCmdAccelYawPointPub_.publish(trajErrorCmdAccelYawMsg); // */
  }
  else
  {
    reset();
  }
}

void LandingStateMachineNode::landPrepCurve(const CurveRequest& cReq)
{
  if (!auto_navigation_)
  {
    double h0 = odometry_.position(2),
                   he = cReq.endState_(2) - h0,
                   T = cReq.duration_;

    ACADO::Function f;
    ACADO::TIME t;
    double t0 = t_.toSec();
    f << h0 + 3.0*he*pow((t - t0)/T, 2) - 2.0*he*pow((t - t0)/T, 3);

    CmdCurve* cLanperep = new CmdCurve;
    double te = t0 + T;
    cLanperep->tEnd = te;
    cLanperep->curveZ.add(t0, te + 0.1, f);

    // Clean the old curvePtrMap;
    if (!curvePtrMap_.count(cReq.type_) == 0)
    {curvePtrMap_.erase(cReq.type_);}
    // Insert the new curve to the curvePtrMap.
    curvePtrMap_.insert(std::pair<int, CmdCurve*>(cReq.type_, cLanperep));

    // /* for test
    ACADO::GnuplotWindow window;
    window.addSubplot(cLanperep->curveZ, t0, t0 + T, "test curve");
    window.plot(); // */
  }
  else
  {
    double t0;
    double T, Tz, Tzrest;
    int orderPlusOne = 4; // 3+1
    int type =  cReq.type_;
    Eigen::Vector3d p0, De;
    Eigen::VectorXd Cx(orderPlusOne), Cy(orderPlusOne), Cz(orderPlusOne);

    t0 = t_.toSec();
    T = cReq.duration_;
    p0 = odometry_.position;
    // we want the curve time is at least 1.5 sec.
    if (T <= 2.5)
    {
      T = 2.5;
      Tz = T;
      Tzrest = 0.0;
    }
    else
    {
      Tz = 2.5;
      Tzrest = T - 2.5;
    }
    De = cReq.endState_ - p0;

    ACADO::Function fx, fy, fz, fzrest;
    ACADO::Expression ex(0.0), ey(0.0), ez(0.0), ezrest(0.0);
    ACADO::TIME t;

    Cx << p0(0), 0.0, 3.0*De(0), - 2.0*De(0);
    Cy << p0(1), 0.0, 3.0*De(1), - 2.0*De(1);
    Cz << p0(2), 0.0, 3.0*De(2), - 2.0*De(2);

    for (int i=0;i<orderPlusOne;i++)
    {
      ex += Cx(i)*pow((t - t0)/T, i);
      ey += Cy(i)*pow((t - t0)/T, i);
      ez += Cz(i)*pow((t - t0)/Tz, i);
    }
    ezrest += cReq.endState_(2);

    std::cout<<"zend:"<<cReq.endState_(2)<<std::endl;

    fx << ex;
    fy << ey;
    fz << ez;
    fzrest << ezrest;

    CmdCurve* cTrack = new CmdCurve;
    double te = t0 + T;
    double tez = t0 + Tz;
    double tezrest = te + Tzrest;
    cTrack ->tStart = t0;
    cTrack ->tEnd = te;
    cTrack ->pStart = p0;
    cTrack ->pEnd = cReq.endState_;
    cTrack ->curveX.add(t0, te + 0.1, fx);
    cTrack ->curveY.add(t0, te + 0.1, fy);
    cTrack ->curveZ.add(t0, tez, fz);
    cTrack ->curveZ.add(tez, tezrest+0.1, fzrest);

    // Clean the old curvePtrMap;
    if (!curvePtrMap_.count(type) == 0)
    {curvePtrMap_.erase(type);}

    // Insert the new curve to the curvePtrMap.
    curvePtrMap_.insert(std::pair<int, CmdCurve*>(type, cTrack));

    // /* for test
    ACADO::GnuplotWindow window;
    window.addSubplot(cTrack->curveX, t0, t0 + T, "test curve x");
    window.addSubplot(cTrack->curveY, t0, t0 + T, "test curve y");
    window.addSubplot(cTrack->curveZ, t0, t0 + T, "test curve z");
    window.plot(); // */
  }
}

void LandingStateMachineNode::trackingCurve(const CurveRequest& cReq)
{
  // /*
  double t0;
  double T;
  int orderPlusOne = 4; // 3+1
  int type =  cReq.type_;
  Eigen::Vector3d p0, De;
  Eigen::VectorXd Cx(orderPlusOne), Cy(orderPlusOne), Cz(orderPlusOne);

  bool rearrange;
  rearrange = false;
  if (cReq.rearrange_)
  {
    rearrange = true;
    if (!curvePtrMap_.count(type)==0)
    {
      t0 = curvePtrMap_[type]->tStart;
      T = sqrt((cReq.endState_(0) - curvePtrMap_[type]->pStart(0))
                     *(cReq.endState_(0) - curvePtrMap_[type]->pStart(0))
                    + (cReq.endState_(1) - curvePtrMap_[type]->pStart(1))
                     *(cReq.endState_(1) - curvePtrMap_[type]->pStart(1)));
      p0 = curvePtrMap_[type]->pStart;
    }
    else
    {
      ROS_WARN_NAMED("landing_state_machine","Curve rearrange requared, "
                                             "but no curve for mode %d exist, ignore!", type);
      return ;
    }
  }
  else
  {
    t0 = t_.toSec();
    T = cReq.duration_;
    p0 = odometry_.position;
  }
  // we want the curve time is at least 1.5 sec.
  if (T < 1.5) {T = 1.5;}
  De = cReq.endState_ - p0;

  ACADO::Function fx, fy, fz;
  ACADO::Expression ex(0.0), ey(0.0), ez(0.0);
  ACADO::TIME t;

  Cx << p0(0), 0.0, 3.0*De(0), - 2.0*De(0);
  Cy << p0(1), 0.0, 3.0*De(1), - 2.0*De(1);
  Cz << p0(2), 0.0, 3.0*De(2), - 2.0*De(2);

  for (int i=0;i<orderPlusOne;i++)
  {
    ex += Cx(i)*pow((t - t0)/T, i);
    ey += Cy(i)*pow((t - t0)/T, i);
    ez += Cz(i)*pow((t - t0)/T, i);
  }

  fx << ex;
  fy << ey;
  fz << ez;

  CmdCurve* cTrack = new CmdCurve;
  double te = t0 + T;
  cTrack ->tStart = t0;
  cTrack ->tEnd = te;
  cTrack ->pStart = p0;
  cTrack ->pEnd = cReq.endState_;
  cTrack ->curveX.add(t0, te + 0.1, fx);
  cTrack ->curveY.add(t0, te + 0.1, fy);
  cTrack ->curveZ.add(t0, te + 0.1, fz);

  // Clean the old curvePtrMap;
  if (!curvePtrMap_.count(type) == 0)
  {curvePtrMap_.erase(type);}
  // Insert the new curve to the curvePtrMap.
  curvePtrMap_.insert(std::pair<int, CmdCurve*>(type, cTrack));
 /* for test
  if (!rearrange)
  {
    ACADO::GnuplotWindow window;
    window.addSubplot(cTrack->curveX, t0, t0 + T, "test curve x");
    window.addSubplot(cTrack->curveY, t0, t0 + T, "test curve y");
    window.addSubplot(cTrack->curveZ, t0, t0 + T, "test curve z");
    window.plot();
  } //*/
}

void LandingStateMachineNode::predictCurve(const CurveRequest& cReq)
{
  //  /*
  ACADO::OCP* ocpPtr;
  ACADO::OptimizationAlgorithm* algorithmPtr;
  ACADO::DifferentialState Xn, Yn, Zn,
                                                       Vx, Vy, Vz,
                                                       r11, r12, r13,
                                                       r21, r22, r23,
                                                       r31, r32, r33,
                                                       p, q, r;
  ACADO::Control w0w0, w1w1, w2w2, w3w3, w4w4, w5w5;
  ACADO::Parameter T;
  ACADO::DifferentialEquation f; // differential equations
  // ACADO::Function y; // outputs

  Eigen::Matrix3d J;
  J.setZero();
  J(0,0) = 0.0347563;
  J(1,1) = 0.0458929;
  J(2,2) = 0.0977;

  f << dot(Xn) == Vx;
  f << dot(Yn) == Vy;
  f << dot(Zn) == Vz;
  f << dot(Vx) == r31*(5.45263e-06*w0w0+5.45263e-06*w1w1+5.45263e-06*w2w2+5.45263e-06*w3w3+5.45263e-06*w4w4+5.45263e-06*w5w5);
  f << dot(Vy) == r32*(5.45263e-06*w0w0+5.45263e-06*w1w1+5.45263e-06*w2w2+5.45263e-06*w3w3+5.45263e-06*w4w4+5.45263e-06*w5w5);
  f << dot(Vz) == r33*(5.45263e-06*w0w0+5.45263e-06*w1w1+5.45263e-06*w2w2+5.45263e-06*w3w3+5.45263e-06*w4w4+5.45263e-06*w5w5) - 9.81; // g = - 9.81 //
  f << dot(r11) == r12*r - r13*q;      f << dot(r12) == r13*p - r11*r;      f << dot(r13) == r11*q - r12*p;
  f << dot(r21) == r22*r - r23*q;      f << dot(r22) == r23*p - r21*r;      f << dot(r23) == r21*q - r22*p;
  f << dot(r31) == r32*r - r33*q;      f << dot(r32) == r33*p - r31*r;      f << dot(r33) == r31*q - r32*p;
  f << dot(p) == (J(1,1) - J(2,2))*q*r/J(0,0)+2.64405e-05*w0w0+5.28809e-05*w1w1+2.64405e-05*w2w2-2.64405e-05*w3w3-5.28809e-05*w4w4-2.64405e-05*w5w5;
  f << dot(q) == (J(2,2) - J(0,0))*r*p/J(1,1)-3.46831e-05*w0w0-1.96101e-16*w1w1+3.46831e-05*w2w2+3.46831e-05*w3w3-1.96101e-16*w4w4-3.46831e-05*w5w5;
  f << dot(r) == (J(0,0) - J(1,1))*p*q/J(2,2)-1.39997e-06*w0w0+1.39997e-06*w1w1-1.39997e-06*w2w2+1.39997e-06*w3w3-1.39997e-06*w4w4-1.39997e-06*w5w5;

  ocpPtr = new ACADO::OCP(0.0, T);
  ocpPtr->minimizeLagrangeTerm(80*(pow(Xn - 0.0, 2) + pow(Yn - 0.0, 2) + pow(Zn - 0.3, 2))
                                                           + 80*(pow(Vx - 0.0, 2) + pow(Vy - 0.0, 2) + pow(Vz - 0.0, 2)));
  // ocpPtr->minimizeLagrangeTerm(5*(w0w0*w0w0));
  // ocpPtr->minimizeLagrangeTerm(5*(w1w1*w1w1));
  // ocpPtr->minimizeLagrangeTerm(5*(w2w2*w2w2));
  // ocpPtr->minimizeLagrangeTerm(5*(w3w3*w3w3));
  // ocpPtr->minimizeLagrangeTerm(5*(w4w4*w4w4));
  // ocpPtr->minimizeLagrangeTerm(5*(w5w5*w5w5));
  ocpPtr->minimizeMayerTerm(80*(pow(Xn - 0.0, 2) + pow(Yn - 0.0, 2) + pow(Zn - 0.3, 2))
                                                           + 80*(pow(Vx - 0.0, 2) + pow(Vy - 0.0, 2) + pow(Vz - 0.0, 2)));
  ocpPtr->subjectTo(f);
  ocpPtr->subjectTo(0.0 <= w0w0 <= 702244.0);
  ocpPtr->subjectTo(0.0 <= w1w1 <= 702244.0);
  ocpPtr->subjectTo(0.0 <= w2w2 <= 702244.0);
  ocpPtr->subjectTo(0.0 <= w3w3 <= 702244.0);
  ocpPtr->subjectTo(0.0 <= w4w4 <= 702244.0);
  ocpPtr->subjectTo(0.0 <= w5w5 <= 702244.0);
  ocpPtr->subjectTo(-5.0 <= p <= 5.0);
  ocpPtr->subjectTo(-5.0 <= q <= 5.0);
  ocpPtr->subjectTo(-5.0 <= r <= 5.0);

  Eigen::Vector3d odom_velocity_W;
  odom_velocity_W = R_*odometry_.velocity;

  ocpPtr->subjectTo(ACADO::AT_START, Xn ==  odometry_.position(0));
  ocpPtr->subjectTo(ACADO::AT_START, Yn ==  odometry_.position(1));
  ocpPtr->subjectTo(ACADO::AT_START, Zn ==  odometry_.position(2));
  ocpPtr->subjectTo(ACADO::AT_START, Vx ==  odom_velocity_W(0));
  ocpPtr->subjectTo(ACADO::AT_START, Vy ==  odom_velocity_W(1));
  ocpPtr->subjectTo(ACADO::AT_START, Vz ==  odom_velocity_W(2));
  ocpPtr->subjectTo(ACADO::AT_START, r11 == R_(0,0));
  ocpPtr->subjectTo(ACADO::AT_START, r12 == R_(0,1));
  ocpPtr->subjectTo(ACADO::AT_START, r13 == R_(0,2));
  ocpPtr->subjectTo(ACADO::AT_START, r21 == R_(1,0));
  ocpPtr->subjectTo(ACADO::AT_START, r22 == R_(1,1));
  ocpPtr->subjectTo(ACADO::AT_START, r23 == R_(1,2));
  ocpPtr->subjectTo(ACADO::AT_START, r31 == R_(2,0));
  ocpPtr->subjectTo(ACADO::AT_START, r32 == R_(2,1));
  ocpPtr->subjectTo(ACADO::AT_START, r33 == R_(2,2));
  ocpPtr->subjectTo(ACADO::AT_START, p == odometry_.angular_velocity(0));
  ocpPtr->subjectTo(ACADO::AT_START, q == odometry_.angular_velocity(1));
  ocpPtr->subjectTo(ACADO::AT_START, r ==  odometry_.angular_velocity(2));
  ocpPtr->subjectTo(ACADO::AT_END, Xn ==  0.0);
  ocpPtr->subjectTo(ACADO::AT_END, Yn ==  0.0);
  ocpPtr->subjectTo(ACADO::AT_END, Zn ==  0.3);
  ocpPtr->subjectTo(ACADO::AT_END, Vx ==  0.0);
  ocpPtr->subjectTo(ACADO::AT_END, Vy ==  0.0);
  ocpPtr->subjectTo(ACADO::AT_END, Vz ==  0.0);
  //ocpPtr->subjectTo(ACADO::AT_END, p ==  0.0);
  //ocpPtr->subjectTo(ACADO::AT_END, q ==  0.0);
  ocpPtr->subjectTo(ACADO::AT_END, r ==  0.0); //*///*/
  ocpPtr->subjectTo(ACADO::AT_END, 0.85 <= r33 <= 1.0);

  algorithmPtr = new ACADO::OptimizationAlgorithm(*ocpPtr);
  algorithmPtr->set( ACADO::DISCRETIZATION_TYPE , ACADO::MULTIPLE_SHOOTING);
  algorithmPtr->set( ACADO::INTEGRATOR_TYPE , ACADO::INT_BDF);
  algorithmPtr->set( ACADO::HESSIAN_APPROXIMATION   , ACADO::BLOCK_BFGS_UPDATE);
  algorithmPtr->set( ACADO::KKT_TOLERANCE   , 1e-4);
  algorithmPtr->set( ACADO::ABSOLUTE_TOLERANCE, 1e-4);
  algorithmPtr->set( ACADO::INTEGRATOR_TOLERANCE, 1e-4);
  algorithmPtr->set( ACADO::MAX_NUM_ITERATIONS, 10000);
  algorithmPtr->set( ACADO::MAX_NUM_INTEGRATOR_STEPS, 10000);

  /* // If 'true', print the ACADO Copyright banner:
  //alg.set(PRINT_COPYRIGHT, BT_TRUE); // default
  algorithmPtr->set(ACADO::PRINT_COPYRIGHT, BT_FALSE);

  // If 'MEDIUM', this will print table with " sqp it | qp its | kkt tol "
  //alg.set(PRINTLEVEL, MEDIUM); // default
  algorithmPtr->set(ACADO::PRINTLEVEL,  ACADO::LOW);

  // If 'true', print the number of, and time, for RHS evaluations, jacobian evaluations, etc.
  algorithmPtr->set(ACADO::PRINT_INTEGRATOR_PROFILE, BT_FALSE); // default

  // If 'true', print the time for SQP iteration, condensing, solving QP, etc.
  algorithmPtr->set(ACADO::PRINT_SCP_METHOD_PROFILE, BT_FALSE); // default */

  algorithmPtr->solve();

  ACADO::VariablesGrid U, X;
  algorithmPtr->getControls(U);
  algorithmPtr->getDifferentialStates(X);
  double te = algorithmPtr->getEndTime();

  ACADO::GnuplotWindow window;
  window.addSubplot( X(0), "x");
  window.addSubplot( X(1), "y");
  window.addSubplot( X(2), "z");
  window.plot();

  ACADO::GnuplotWindow window2;
  window2.addSubplot( X(3), "Vx");
  window2.addSubplot( X(4), "Vy");
  window2.addSubplot( X(5), "Vz");
  window2.plot();

  ACADO::GnuplotWindow window1;
  window1.addSubplot( X(15), "p");
  window1.addSubplot( X(16), "q");
  window1.addSubplot( X(17), "r");
  window1.plot();
  ACADO::GnuplotWindow window3;
  window3.addSubplot( U(0), "w1");
  window3.addSubplot( U(1), "w2");
  window3.addSubplot( U(2), "w3");
  window3.plot();

  ACADO::GnuplotWindow window4;
  window4.addSubplot( U(3), "w3");
  window4.addSubplot( U(4), "w4");
  window4.addSubplot( U(5), "w5");
  window4.plot(); // */

  Xn.clearStaticCounters();
  Yn.clearStaticCounters();
  Zn.clearStaticCounters();
  Vx.clearStaticCounters();
  Vy.clearStaticCounters();
  Vz.clearStaticCounters();
  p.clearStaticCounters();
  q.clearStaticCounters();
  r.clearStaticCounters();
  r11.clearStaticCounters(); r12.clearStaticCounters(); r13.clearStaticCounters();
  r21.clearStaticCounters(); r22.clearStaticCounters(); r23.clearStaticCounters();
  r31.clearStaticCounters(); r32.clearStaticCounters(); r33.clearStaticCounters();
  T.clearStaticCounters();
  w0w0.clearStaticCounters();
  w1w1.clearStaticCounters();
  w2w2.clearStaticCounters();
  w3w3.clearStaticCounters();
  w4w4.clearStaticCounters();
  w5w5.clearStaticCounters();
}

// No curve
void LandingStateMachineNode::noCurve(const CurveRequest& cReq)
{
}

void LandingStateMachineNode::landingCurve(const CurveRequest& cReq)
{
  double h0 = odometry_.position(2),
                 he = cReq.endState_(2) - h0,
                 T = cReq.duration_;

  ACADO::Function f;
  ACADO::TIME t;
  double t0 = t_.toSec();
  f << h0 + 3.0*he*pow((t - t0)/T, 2) - 2.0*he*pow((t - t0)/T, 3);

  CmdCurve* cLanperep = new CmdCurve;
  double te = t0 + T;
  cLanperep->tEnd = te;
  cLanperep->curveZ.add(t0, te + 0.1, f);

  // Clean the old curvePtrMap;
  if (!curvePtrMap_.count(cReq.type_) == 0)
  {curvePtrMap_.erase(cReq.type_);}
  // Insert the new curve to the curvePtrMap.
  curvePtrMap_.insert(std::pair<int, CmdCurve*>(cReq.type_, cLanperep));

  // /* for test
  ACADO::GnuplotWindow window;
  window.addSubplot(cLanperep->curveZ, t0, t0 + T, "test curve");
  window.plot(); // */
}

void LandingStateMachineNode::planThread()
{
  while (nhPlan_.ok())
  {
    if (!curveRequests_.empty())
    {
      CurveRequest cReq = curveRequests_.front();
      // Switching mode.
      // 0: Create curve for landing preparation, 1: Create curve for tracking,
      // 2: Create curve for time optimized tracking,  3: No curve,   4: create curve for landing.
      (this->*createCurve_[cReq.type_])(cReq);

      curveRequests_.pop_front();
    }
  }
}

void LandingStateMachineNode::run()
{
  boost::function0<void> fcn_plan =
      boost::bind(&LandingStateMachineNode::planThread, this);
  boost::thread plan_thread(fcn_plan);

  while (ros::ok())
  {
    ros::spinOnce();
  }

  plan_thread.join();
}
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "landing_state_machine");

  landing_control::LandingStateMachineNode landing_state_machine_node;

  landing_state_machine_node.run();

  return 0;
}
