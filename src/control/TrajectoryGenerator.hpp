/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#ifndef CR_CONTROL_TRAJECTORY_GENERATOR_HPP_
#define CR_CONTROL_TRAJECTORY_GENERATOR_HPP_

#include "Eigen/Dense"
#include "aspect/Temporal.hpp"
#include "core/Clock.hpp"
#include "core/Types.hpp"
#include "control/Policy.hpp"

namespace cr {
namespace control {

//------------------------------------------------------------------------------
/*!
 \class TrajectoryGenerator
 \ingroup control

 \brief This class provides methods for generating minimum jerk
 trajectories from initial and final conditions.

 \details
 The TrajectoryGenerator uses an internal clock to produce a time-based action

 \f[
 u_{k} = \pi(t_k)
 \f]

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
