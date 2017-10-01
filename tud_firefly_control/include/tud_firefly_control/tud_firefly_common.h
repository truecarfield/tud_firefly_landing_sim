#ifndef TUD_FIREFLY_COMMON_H
#define TUD_FIREFLY_COMMON_H

#include <rotors_control/common.h>

namespace  rotors_control {

const double PI = 3.141592653589793;
const double TAU = 6.283185307179587;

inline Eigen::Quaterniond toQuaternion(double roll, double pitch, double yaw)
{
  Eigen::Quaterniond q;
  double t0 = std::cos(yaw * 0.5);
  double t1 = std::sin(yaw * 0.5);
  double t2 = std::cos(roll * 0.5);
  double t3 = std::sin(roll * 0.5);
  double t4 = std::cos(pitch * 0.5);
  double t5 = std::sin(pitch * 0.5);

  q.w() = t0 * t2 * t4 + t1 * t3 * t5;
  q.x() = t0 * t3 * t4 - t1 * t2 * t5;
  q.y() = t0 * t2 * t5 + t1 * t3 * t4;
  q.z() = t1 * t2 * t4 - t0 * t3 * t5;
  return q;
}

double clampRotation(double rotation)
{
  while (rotation > PI)
  {
    rotation -= TAU;
  }

  while (rotation < -PI)
  {
    rotation += TAU;
  }

  return rotation;
}

}
#endif // TUD_FIREFLY_COMMON_H
