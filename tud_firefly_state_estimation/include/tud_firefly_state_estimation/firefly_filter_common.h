#ifndef FIREFLY_FILTER_COMMON_H
#define FIREFLY_FILTER_COMMON_H

#include <robot_localization/filter_common.h>

namespace RobotLocalization {

//! @brief Enumeration that defines the control vector which is output from the controller
//!
enum FireflyControlMembers
{
  ControlMemberAp,        // Angular acclerations in {B}
  ControlMemberAq,
  ControlMemberAr,
  ControlMemberAu,         // Linear accelerations in {B}
  ControlMemberAv,
  ControlMemberAw
};

const int FIREFLY_STATE_SIZE = 18;
const int CONTROL_SIZE = 6;
}

#endif
