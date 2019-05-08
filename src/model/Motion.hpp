/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#ifndef CR_MOTION_HPP_
#define CR_MOTION_HPP_

#include "Eigen/Dense"
#include "core/Item.hpp"
#include "core/Step.hpp"

namespace cr {
namespace model {

namespace ph = std::placeholders;

//------------------------------------------------------------------------------
/*!
 \class Motion
 \ingroup model

 \brief This class implements a motion model.

 \details
 ## Description
 Motion implements a motion model from a supplied dynamics callback function.
 Specifically, MotionModel sets up a container for the discrete-time model

 \f[
 x_{k+1} = f(t_k, x_k, u_k)
 \f]

 where \f$x\f$ is the state, \f$u\f$ is the input, \f$t\f$ is time, and \f$k\f$
 is a discrete sampling index, specified by \f$dt\$.

 To use this class, users must derive
 - `StateType callback(double i_t, StateType i_x, ActionType i_u)`

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
  Motion(const StateType &i_x0,
         const ActionType &i_u0,
         const double i_dt = 0.01)
      : m_state(i_x0), m_action(i_u0), m_dt(i_dt) {};

  //! Class destructor
  virtual ~Motion() = default;

public:
  //! The prototype motionCallback function must be implemented.
  virtual StateType motionCallback(double i_t, StateType i_x,
                                   ActionType i_u) = 0;

  ///! This function steps the callback and updates the state.
  virtual void step() {
    m_state = this->m_motion_fcn(m_time, m_state, m_action);
    m_time += m_dt;
  }

public:
  //! Set the state vector (x)
  void setState(const StateType &i_x) { m_state = i_x; }

  //! Get the state vector (x)
  StateType getState() { return m_state; }

  //! Set the action vector (u)
  void setAction(const ActionType &i_u) { m_action = i_u; }

  //! Get the action vector (u)
  ActionType setAction() { return m_action; }

  //! Set the time step (s)
  void setTimeStep(const double i_timeStep) { m_dt = i_timeStep; }

  //! Get the time step (s)
  double getTimeStep() { return m_dt; }

  //! Get the model time (s)
  double getTime() { return m_time; }

protected:
  //! bound callback function
  std::function<StateType(double, StateType, ActionType)> m_motion_fcn =
      std::bind(&Motion::motionCallback, this, ph::_1, ph::_2, ph::_3);

  //! Sample rate (s)
  double m_dt;

  //! Current time (s)
  double m_time = 0.0;

  //! Dynamic state of the system (x)
  StateType m_state;

  //! Dynamic state of the system (u)
  ActionType m_action;
};

} // namepsace model
} // namepsace cr

#endif
