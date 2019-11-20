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
  KinematicWaypoint() = default;
  KinematicWaypoint(std::size_t n)
      : time(0), position(Eigen::VectorXd::Zero(n)),
        velocity(Eigen::VectorXd::Zero(n)),
        acceleration(Eigen::VectorXd::Zero(n)), jerk(Eigen::VectorXd::Zero(n)) {
  }

  double time;
  Eigen::VectorXd position;
  Eigen::VectorXd velocity;
  Eigen::VectorXd acceleration;
  Eigen::VectorXd jerk;
};

} // namespace control
} // namespace cr

#endif
