/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#include "Motion.hpp"
#include "math/Integration.hpp"

namespace cr {
namespace model {

//------------------------------------------------------------------------------
/*!
 The constructor creates a motion model.\n

 \param[in] i_x0 - the initial state.
 \param[in] i_u0 - the initial action.
 \param[in] i_timeStep - the time step of the system
 */
//------------------------------------------------------------------------------
Motion::Motion(Eigen::VectorXd i_x0, Eigen::VectorXd i_u0, double i_dt) {
  m_time = 0;
  m_state = i_x0;
  m_action = i_u0;
  m_dt = i_dt;
}

//------------------------------------------------------------------------------
/*!
 This method steps the motion model, updating the internal state.
 */
//------------------------------------------------------------------------------
void Motion::step() {

  // use the bound member function pointer
  // https://stackoverflow.com/questions/7582546/using-generic-stdfunction-objects-with-member-functions-in-one-class
  //
  // (lambdas create a memory leak with shared pointers!
  // http://floating.io/2019/07/lambda-shared_ptr-memory-leak/)
  m_state =
      math::Integration::rungeKuttaStep(m_fcn, m_time, m_state, m_action, m_dt);

  // update the time
  m_time = m_time + m_dt;
}

} // namepsace model
} // namepsace cr
