/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#ifndef CR_MODEL_MOTION_HPP_
#define CR_MODEL_MOTION_HPP_

#include "Eigen/Dense"
#include "aspect/Action.hpp"
#include "aspect/Parameter.hpp"
#include "aspect/State.hpp"
#include "aspect/Temporal.hpp"
#include "core/Item.hpp"
#include "core/Step.hpp"
#include "math/Integration.hpp"

namespace cr {
namespace model {

namespace ph = std::placeholders;

//------------------------------------------------------------------------------
/*!
 \class Motion
 \ingroup model

 \brief This class implements a motion model from a supplied dynamics callback 
 function. Specifically, MotionModel sets up a container for the discrete time
 set of equations

 \f[
 x_{k+1} = f(t_k, x_k, u_k)
 \f]

 where \f$x\f$ is state, \f$u\f$ is input action, \f$t\f$ is time, and \f$k\f$
 is a discrete sampling index.

 To use this class, users must override Motion::motionCallback

 ## References
 [1] J. Crassidis and J. Junkins, "Optimal Estimation of Dynamic Systems",
 Ed. 2, CRC Press, 2012. \n\n

 [2] S. Thrun, W. Burgard, and D. Fox, "Probabilistic Robotics", MIT Press,
 2006.
 \n\n
 */
//------------------------------------------------------------------------------
template <typename StateType, typename ActionType>
class Motion : public core::Step, public core::Item {

public:
  //! Class constructor
  Motion(const StateType &i_state, 
         const ActionType &i_action, 
         const double i_dt = 0.01)
    : m_state(i_state), m_action(i_action), m_dt(i_dt) {};

  //! Class destructor
  virtual ~Motion() = default;

  CR_ASPECT_STATE_WRITE(StateType);

  CR_ASPECT_ACTION_WRITE(ActionType);

  CR_ASPECT_TEMPORAL_DISCRETIZED

public:
  //! The prototype motionCallback function must be implemented.
  virtual StateType motionCallback(double i_t, StateType i_x,
                                   ActionType i_u) = 0;

  //! This function steps the callback and updates the state.
  virtual void step() { m_state = (m_motion_fcn)(m_time, m_state, m_action); }

protected:
  //! bound callback function
  std::function<StateType(double, StateType, ActionType)> m_motion_fcn =
      std::bind(&Motion::motionCallback, this, ph::_1, ph::_2, ph::_3);
};

} // namepsace model
} // namepsace cr

#endif
