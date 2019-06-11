/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#ifndef CR_CONTROL_TYPES_HPP_
#define CR_CONTROL_TYPES_HPP_

namespace cr {
namespace control {

/*!
Structure defining a 3rd order kinematic waypoint
\ingroup control
*/
struct KinematicWaypoint {
  double time;
  Eigen::VectorXd position;
  Eigen::VectorXd velocity;
  Eigen::VectorXd acceleration;
  Eigen::VectorXd jerk;
};

} // namepsace control
} // namepsace cr

#endif
