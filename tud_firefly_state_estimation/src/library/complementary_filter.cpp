#include "tud_firefly_state_estimation/complementary_filter.h"

namespace RobotLocalization {

CompFilter::CompFilter():
  gravity_(9.80665)
{
  testbad = false;
   beta_ << 0.95,  0.0,     0.0,
                     0.0,     0.95,  0.0,
                     0.0,     0,0,    1.0;
   Identity_<< 1.0,   0.0,   0.0,
                           0.0,   1.0,   0.0,
                           0.0,   0.0,   1.0;
}

CompFilter::~CompFilter() {}

void CompFilter::init(const Eigen::Matrix3d& beta,
                                          const Eigen::Vector3d& tauGyr,
                                          const Eigen::Vector3d& tauAccel,
                                          const std::vector<int>& tauGyrConfig,
                                          const std::vector<int>& tauAccelConfig)
{
  beta_ = beta;
  gyrTracker_.init(tauGyr.size(), tauGyr, tauGyrConfig);
  accelTracker_.init(tauAccel.size(), tauAccel, tauAccelConfig);
  filteredData_.resize(3);
  filteredData_.setZero();
}

void CompFilter::update(const Eigen::Vector3d& accData,
                                                  const Eigen::Vector3d& gyrData,
                                                  const double& dt,
                                                  Eigen::Vector3d& filteredData)
{

  // Integrate the gyroscope data
  filteredData_ += gyrData*dt; filteredDataTest_ += gyrData*dt;
  gyrRtest += gyrData(2)*dt;

  Eigen::VectorXd dataGyrOut(3), dataGyrOutTest(3);;
  gyrTracker_.track(filteredData_, dataGyrOut, dt);
  accelTracker_.track(filteredDataTest_, dataGyrOutTest, dt);

 // Compensate for drift with accelerometer data
  Eigen::VectorXd dataAccelTemp(3), dataAccelOut(3);

  double sinPitchHat = -accData(1)/gravity_;
  double sinRollHat = accData(0)/gravity_;
  if (sinPitchHat > 1.0)
  {
    sinPitchHat = 1.0;
  }
  else if (sinPitchHat < -1.0)
  {
    sinPitchHat = -1.0;
  }
  if (sinRollHat > 1.0)
  {
    sinRollHat = 1.0;
  }
  else if (sinRollHat < -1.0)
  {
    sinRollHat = -1.0;
  }

  dataAccelTemp << ::asin(sinPitchHat),
                                        ::asin(sinRollHat),
                                       dataGyrOut(2);

    // accelTracker_.track(dataAccelTemp, dataAccelOut, dt);


  double testvor = filteredData_(2);
  filteredData_ = beta_*dataGyrOut + (Identity_ - beta_)*dataAccelTemp;
  filteredDataTest_(2) = beta_(2, 2)*dataGyrOutTest(2) + (Identity_(2,2) - beta_(2,2))*dataGyrOutTest(2);
  filteredData = filteredData_;
  double testnach = filteredData_(2);
  //if (!std::isnan(testvor) && std::isnan(testnach) && !testbad)
  //{testbad = true;
  //  std::cout<<"comp accData1:"<<sinPitchHat<<", roll:"<<::asin(sinPitchHat)<<", accData2:"<<sinRollHat<<", pitch:"<< ::asin(sinRollHat)<< ", yaw:"<<dataGyrOut(2)<<", dataGyrOutTest:"<<dataGyrOutTest(2)<<", r:"<<gyrData(2)<<"haha"<<std::endl;}
}

}
