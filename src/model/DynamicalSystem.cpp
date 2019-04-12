/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#include "DynamicalSystem.hpp"
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
DynamicalSystem::DynamicalSystem(const Eigen::VectorXd& i_x0,
    const Eigen::VectorXd& i_u0, const double i_dt, ModelType i_type)
    : Motion<Eigen::VectorXd, Eigen::VectorXd>(i_x0, i_u0, i_dt)  {
  m_time = 0;
  m_state = i_x0;
  m_action = i_u0;
  m_dt = i_dt;
  m_type = i_type;
}

//------------------------------------------------------------------------------
/*!
 This method steps the motion model, updating the internal state.
 */
//------------------------------------------------------------------------------
void DynamicalSystem::step() {
  if (m_type == DISCRETE_TIME) {
    m_state = (m_motion_fcn)(m_time, m_state, m_action);

  } else if (m_type == CONTINUOUS_TIME) {
    m_state = math::Integration::rungeKuttaStep(
      m_motion_fcn, m_time, m_state, m_action, m_dt);
  }
  m_time = m_time + m_dt;
}

} // namepsace model
} // namepsace cr
