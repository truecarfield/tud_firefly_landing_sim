#include "tud_firefly_control/differential_tracker.h"

namespace rotors_control {

void DifferentialTracker::report(Eigen::VectorXd& tau, std::vector<int>& config)
{
  tau = tau_;
  config = tauConfig_;
}

void DifferentialTracker::init(const ros::NodeHandle& pnh, const std::string& trackerName)
{
  int dim;
  bool pubResult;

  firstValueReceived_ = false;
  dim_ = dim;
  trackerName_ = trackerName;
  pnh.param("dim_" + trackerName, dim, 3);
  pnh.param("pub_" + trackerName, pubResult, false);

  // Load up the tau for differential filter of the tracker (from the launch file/parameter server) 
  std::vector<double> tauStdVector(dim, 0);
  Eigen::VectorXd tau(dim);
  tau.setZero();
  if (pnh.getParam("tau_" + trackerName, tauStdVector))
  {
    if(tauStdVector.size() != dim)
    {
      ROS_ERROR_STREAM("Tau of "+ trackerName +" differential filter must be of size " << dim << ". Provided config was of "
      "size " << tauStdVector.size() << ". No time constants will be initialized for differential filter");
    }
    else
    {
      /* std::cout<<"tauStdVector0: "
                       <<tauStdVector[0]<<std::endl
                       <<"tauStdVector1: "
                       <<tauStdVector[1]<<std::endl
                       <<"tauStdVector2: "
                       <<tauStdVector[2]<<std::endl; */
      for (int i=0;i<dim;i++)
      {
        tau(i) = tauStdVector[i];
      }
    }
  }
  else
  {
    ROS_ERROR_STREAM("Tau of the " + trackerName + " differential filter initialization failed. No time constants will be initialized for differential filter");
  }

  // Load up the tau config for differential filter of the gyroscope
  std::vector<int> tauConfig(dim, 0);
  if(pnh.getParam("tau_" + trackerName + "_config", tauConfig))
  {
    if(tauConfig.size() != dim)
    {
      ROS_ERROR_STREAM("Tau configuration of the " + trackerName + " differential filter must be of size " << dim << ". Provided config was of "
        "size " << tauConfig.size() << ". ");
    }
  }
  init(dim, tau, tauConfig);

  pubResult_ = pubResult;
  if (pubResult_)
  {
    resultMsg_.data.resize(dim*3);
    resultPub_ = nh_.advertise<std_msgs::Float64MultiArray>(trackerName, 1);
  }
}

void DifferentialTracker::publish(const Eigen::VectorXd& x)
{
  if (pubResult_)
  {
    // std::cout<<"x.size:"<<x.size()<<std::endl;
    for (int i=0;i<x.size();i++)
    {
      resultMsg_.data[i] = x(i);
      resultMsg_.data[x.size()+i] = x_f(i);
      resultMsg_.data[x.size()*2+i] = dx_f(i);
    }
    resultPub_.publish(resultMsg_);
  }
}

void DifferentialTracker::init(const int& dim, const Eigen::VectorXd& tau, const std::vector<int>& tauConfig)
 {
   x_f.resize(dim);
   dx_f.resize(dim);
   ddx_f.resize(dim);
   tau_.resize(dim);
   tauConfig_.resize(dim);

   x_f.setZero();
   dx_f.setZero();
   ddx_f.setZero();
   tau_.setZero();
   tau_ = tau;
   tauConfig_ = tauConfig;
   if (tauConfig_.size() != dim)
   {
     ROS_ERROR_STREAM("Dimension of tauConfig is "<< tauConfig_.size() <<" does not match to dimension "<< dim <<" of the filter, which will lead to serious failure!");
   };   
 }

 DifferentialTracker::DifferentialTracker()
 {
   pubResult_ = false;

   switchMethod[0] = &DifferentialTracker::PT1;
   switchMethod[1] = &DifferentialTracker::PT2;
   switchMethod[2] = &DifferentialTracker::pass;
   switchMethod[3] = &DifferentialTracker::prevent;
   switchMethod[4] = &DifferentialTracker::adaptPT2;
 }

 DifferentialTracker::~DifferentialTracker() {}

void DifferentialTracker::PT1(const double& x, const int& i, const double& dt)
{
  dx_f(i) = (x - x_f(i))/tau_(i);
  x_f(i) += dx_f(i)*dt;
 // std::cout<<"dx_f("<<i<<") is "<<dx_f(i)<<std::endl;
}

void DifferentialTracker::PT2(const double& x, const int& i, const double& dt)
{
  ddx_f(i) = -2.0*dx_f(i)/tau_(i) - (x_f(i) - x)/(tau_(i)*tau_(i));
  dx_f(i) += ddx_f(i)*dt;
  x_f(i) += dx_f(i)*dt;
  // std::cout<<"The "<<i<<" th state is x:"<<x<<", x_f("<<i<<"):"<<x_f(i)<<", dx_f("<<i<<"):"<<dx_f(i)<<", ddx_f("<<i<<"):"<<ddx_f(i)<<", tau_f("<<i<<"):"<<tau_(i)<<std::endl;
}

void DifferentialTracker::pass(const double& x, const int& i, const double& dt)
{
  // if (i==2 && !std::isnan(x) && std::isnan(x_f(i)))
  //  {std::cout<<"passing x("<<i<<"):"<<x<<" to x_f("<<i<<"):"<<x_f(i)<<std::endl;}
  x_f(i) = x;
}

void DifferentialTracker::prevent(const double& x, const int& i, const double& dt)
{
}

void DifferentialTracker::adaptPT2(const double &x, const int &i, const double &dt)
{
  double tauAdapt = std::min(fabs(adaptVar_), tau_(i) + 0.2);
  tauAdapt = std::max(tauAdapt, tau_(i));
  //std::cout<<"adaptVar:"<<tauAdapt<<std::endl;

  ddx_f(i) = -2.0*dx_f(i)/tauAdapt - (x_f(i) - x)/(tauAdapt*tauAdapt);
  // std::cout<<"ddx_f(i):"<<ddx_f(i)<<std::endl;
  // ddx_f(i) = boost::math::sign(ddx_f(i))*std::min(fabs(ddx_f(i)), 0.05);
  dx_f(i) += ddx_f(i)*dt;
  x_f(i) += dx_f(i)*dt;
}

void DifferentialTracker::reportX(const Eigen::VectorXd& x)
{
  if (pubResult_)
  {
    std_msgs::Float64MultiArray x_msg;
    for (int i=0;i<x.size();i++)
      x_msg.data.push_back(x(i));
    resultPub_.publish(x_msg);
  }
}

void DifferentialTracker::trackAdapt(const Eigen::VectorXd &x, Eigen::VectorXd &x_f_out, const double &adaptVar, const double &dt)
{
  adaptVar_ = adaptVar;
  if (!firstValueReceived_)
  {
    x_f = x_f_out;
    firstValueReceived_ = true;
  }

  for (int i = 0;i < x.size(); i++)
  {
    /* Switching method */
    (this->*switchMethod[tauConfig_[i]])(x(i), i, dt);
  }
  x_f_out = x_f;

  publish(x);
}

void DifferentialTracker::track(const Eigen::VectorXd& x, Eigen::VectorXd& x_f_out, const double &dt)
{
  if (!firstValueReceived_)
  {
    x_f = x_f_out;
    firstValueReceived_ = true;
  }

  for (int i = 0;i < x.size(); i++)
  {
    /* Switching method */
    (this->*switchMethod[tauConfig_[i]])(x(i), i, dt);
  }
  x_f_out = x_f;

  publish(x);
}

void DifferentialTracker::diff(const Eigen::VectorXd& x, Eigen::VectorXd& dx_f_out, const double &dt)
{
  if (!firstValueReceived_)
  {
    x_f = x;
    firstValueReceived_ = true;
  }

  for (int i = 0;i < x.size(); i++)
  {
    /* Switching method */
    (this->*switchMethod[tauConfig_[i]])(x(i), i, dt);
  }
  dx_f_out = dx_f;

  publish(x);
}

void DifferentialTracker::track(const Eigen::VectorXd& x, Eigen::VectorXd& x_f_out, Eigen::VectorXd& dx_f_out, const double &dt)
{
  if (!firstValueReceived_)
  {
    x_f = x_f_out;
    firstValueReceived_ = true;
  }

  for (int i = 0;i < x.size(); i++)
  {
    /* Switching method */
    (this->*switchMethod[tauConfig_[i]])(x(i), i, dt);
  }
  x_f_out = x_f;
  dx_f_out = dx_f;

  publish(x);
}

void DifferentialTracker::track(const Eigen::VectorXd& x, Eigen::VectorXd& x_f_out, Eigen::VectorXd& dx_f_out, Eigen::VectorXd& ddx_f_out, const double &dt)
{
  if (!firstValueReceived_)
  {
    x_f = x_f_out;
    firstValueReceived_ = true;
  }

  for (int i = 0;i < x.size(); i++)
  {
    /* Switching method */
    (this->*switchMethod[tauConfig_[i]])(x(i), i, dt);
  }
  x_f_out = x_f;
  dx_f_out = dx_f;
  ddx_f_out = ddx_f;

  publish(x);
}

void DifferentialTracker::reset()
{
  x_f.setZero();
  dx_f.setZero();
  ddx_f.setZero();

  adaptVar_ = 0.0;

  firstValueReceived_ = false;
}

}
