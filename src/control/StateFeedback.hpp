/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#ifndef CR_CONTROL_STATE_FEEDBACK_HPP_
#define CR_CONTROL_STATE_FEEDBACK_HPP_

#include "Eigen/Dense"
#include "aspect/State.hpp"
#include "control/Policy.hpp"

namespace cr {
namespace control {

//------------------------------------------------------------------------------
/*!
 \class StateFeedback
 \ingroup control

 \brief This class implements a state feedback control policy.

 \details
 Policy implements a control law, which computes an action \f$u\f$ from state
 \f$x\f$, i.e.

 \f[
 u_{k} = \pi(t_k, x_k)
 \f]

 ## References
 [1] J. Crassidis and J. Junkins, "Optimal Estimation of Dynamic Systems",
 Ed. 2, CRC Press, 2012. \n\n

 [2] S. Thrun, W. Burgard, and D. Fox, "Probabilistic Robotics", MIT Press,
 2006.
 \n\n
 */
//------------------------------------------------------------------------------
template <typename StateType, typename ActionType>
class StateFeedback : public control::Policy<ActionType> {

public:
  //! Class constructor
  StateFeedback(const StateType &i_state, const ActionType &i_action)
      : Policy<ActionType>(i_action), m_state(i_state){};

  //! Class destructor
  virtual ~StateFeedback() = default;

  CR_ASPECT_STATE_WRITE(StateType);

public:
  //! The prototype policyCallback function must be implemented.
  virtual ActionType policyCallback(StateType /** i_x **/) {
    return this->m_action;
  }

  //! This function steps the callback and updates the action.
  void step() override { this->m_action = (m_policy_fcn)(m_state); }

protected:
  //! bound callback function
  std::function<ActionType(double, StateType)> m_policy_fcn =
      std::bind(&StateFeedback::policyCallback, this, ph::_1, ph::_2);
};

} // namespace control
} // namespace cr

#endif
