/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#ifndef CR_TRAJECTORY_GENERATOR_HPP_
#define CR_TRAJECTORY_GENERATOR_HPP_

#include "Eigen/Dense"
#include "aspect/Temporal.hpp"
#include "core/Clock.hpp"
#include "core/Types.hpp"
#include "control/Policy.hpp"

namespace cr {
namespace control {

//! Structure defining a waypoint
struct Waypoint {
  double time;
  Eigen::VectorXd position;
  Eigen::VectorXd velocity;
  Eigen::VectorXd acceleration;
  Eigen::VectorXd jerk;
};

//------------------------------------------------------------------------------
/*!
 \class TrajectoryGenerator
 \ingroup control

 \brief This class provides methods for generating minimum jerk
 trajectories from initial and final conditions.

 \details
 ## Description
 TrajectoryGenerator implements the minimum-jerk trajectory generation
 technique from a set of initial condititions, final conditions, and their
 1st and 2nd derivatives.


 These methods are available with the trajectory generator:
 - TrajectoryGenerator::solve computes the minimum jerk trajectory for
 the specified initial and final conditions and stores the representation
 of the trajectory internally.
 - TrajectoryGenerator::step computes the values of the trajectory for
 a specified time (if specified) or for the time elapsed since the solve
 method was called (if time is not specified).

 ## Example
 This example demonstrates use of the TrajectoryGenerator class.
 \include example_TrajectoryGenerator.cpp

 ## References
 [1] N. Hogan, "Adaptive control of mechanical impedance by coactivation of
     antagonist muscles," IEEE Trans. on Automatic Control AC-29: 681-690,
     1984. \n\n

 */
//------------------------------------------------------------------------------
template <typename ActionType>
class TrajectoryGenerator : public Policy<ActionType> {

public:
  //! Class constructor
  TrajectoryGenerator(const ActionType& i_action)
    : Policy<ActionType>(i_action) {}

  //! Destructor
  virtual ~TrajectoryGenerator() = default;

  //! Factory
  static core::StepPtr create(const ActionType& i_action) {
    return core::StepPtr(new TrajectoryGenerator(i_action));
  }

  CR_ASPECT_TEMPORAL_RUNTIME

public:
  //! The prototype policyCallback function must be implemented.
  virtual ActionType policyCallback(double i_t) = 0;

  //! This function starts the trajectory generator
  void onStart() { m_timer.startTimer(); }

  //! This function steps the callback and updates the action.
  virtual void step() { this->m_action = (m_policy_fcn)(getTime()); }

protected:
  //! bound callback function
  std::function<ActionType(double)> m_policy_fcn =
      std::bind(&TrajectoryGenerator::policyCallback, this, ph::_1);
  
};

} // namepsace control
} // namepsace cr

#endif
