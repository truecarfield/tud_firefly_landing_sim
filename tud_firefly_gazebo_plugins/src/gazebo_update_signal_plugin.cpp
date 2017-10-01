#include "tud_firefly_gazebo_plugins/gazebo_update_signal_plugin.h"

namespace gazebo {

GazeboUpdateEventPlugin::~GazeboUpdateEventPlugin() {
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}

void GazeboUpdateEventPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
 model_=_model;
 world_=model_->GetWorld();
 step_size=world_->GetPhysicsEngine()->GetMaxStepSize();

 node_handle_ = new ros::NodeHandle(namespace_);
 updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboUpdateEventPlugin::OnUpdate, this, _1));

 update_pub_ = node_handle_->advertise<std_msgs::Float64MultiArray>(update_pub_topic_, 1);

 // ros::param::get("gazebo/time_step", step_size);
}

void GazeboUpdateEventPlugin::OnUpdate(const common::UpdateInfo& _info) {
 update_time= world_->GetSimTime().Float();

 std_msgs::Float64MultiArrayPtr update_msg(new std_msgs::Float64MultiArray);
 update_msg->data.push_back(update_time);
 update_msg->data.push_back(step_size);

 // std::cout<<"update_event_publisher update time is "<<_info.simTime.Double()<<std::endl;
 update_pub_.publish(update_msg);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboUpdateEventPlugin);
}
