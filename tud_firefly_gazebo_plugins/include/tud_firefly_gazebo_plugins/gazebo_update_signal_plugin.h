#ifndef ROTORS_GAZEBO_ZPDATE_SINGAL_H
#define ROTORS_GAZEBO_ZPDATE_SINGAL_H

#include <cmath>
#include <deque>
#include <random>
#include <stdio.h>

#include <boost/bind.hpp>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <rotors_gazebo_plugins/common.h>
#include <rotors_gazebo_plugins/sdf_api_wrapper.hpp>

namespace gazebo {

class GazeboUpdateEventPlugin : public ModelPlugin {
 public:
  GazeboUpdateEventPlugin ()
    : ModelPlugin(),
      node_handle_(NULL),
      update_pub_topic_("firefly/sim_update_event") {}

  ~GazeboUpdateEventPlugin();

 protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo& /*_info*/);

private:
  std::string namespace_;
  std::string update_pub_topic_;

  ros::NodeHandle* node_handle_;
  ros::Publisher update_pub_;

  double update_time;
  double step_size; 


  physics::WorldPtr world_;
  physics::ModelPtr model_;

  /// \brief Pointer to the update event connection.
  event::ConnectionPtr updateConnection_;

  boost::thread callback_queue_thread_;
};

}

#endif
