/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#ifndef CR_CONTROL_POLICY_HPP_
#define CR_CONTROL_POLICY_HPP_

#include "Eigen/Dense"
#include "aspect/Action.hpp"
#include "core/Item.hpp"
#include "core/Step.hpp"
#include "core/Clock.hpp"

namespace cr {
namespace control {

namespace ph = std::placeholders;

//------------------------------------------------------------------------------
/*!
 \class Policy
 \ingroup control

 \brief This class implements a control policy.

 \details
 A control policy in general computes an action \f$u\f$, and is represented
 by \f$\pi : \rightarrow u\f$.

 ## References
 [1] J. Crassidis and J. Junkins, "Optimal Estimation of Dynamic Systems",
 Ed. 2, CRC Press, 2012. \n\n

 [2] S. Thrun, W. Burgard, and D. Fox, "Probabilistic Robotics", MIT Press,
 2006.
 \n\n
 */
//------------------------------------------------------------------------------
template <typename ActionType>
class Policy : public core::Step, public core::Item {

public:
  //! Class constructor
  Policy(const ActionType &i_action)
    : m_action(i_action) {};

  //! Class destructor
  virtual ~Policy() = default;

  //! Factory
  static core::StepPtr create(const ActionType &i_action) { 
    return core::StepPtr(new Policy<ActionType>(i_action));
  }

  CR_ASPECT_ACTION_READ(ActionType)
};

} // namepsace model
} // namepsace cr

#endif
