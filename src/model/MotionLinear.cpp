/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#include "MotionLinear.hpp"

namespace cr {
namespace model {

//------------------------------------------------------------------------------
/*!
 The constructor creates a motion model.  The i_A, i_B matrices specify
 one of the dynamics equation forms below:\n

 Case 1: (Continuous)

 If i_type is set to CR_MOTION_CONTINUOUS, then the callback sets

 \f$ \dot{x} = A x + B u \f$

 where \f$x\f$ is the system state, \f$u\f$ is the input (forcing)
 vector, and \f$t\f$ is time.


 Case 2: (Discrete)

 If i_type is set to CR_MOTION_DISCRETE, then the callback sets

 \f$ x_{k+1} = A x_k + B u_k \f$

 where \f$x_k\f$ is the current system state, \f$u_k\f$ is the
 input (forcing) vector, and \f$t_k\f$ is time at interval \f$k\f$.


 \param[in] i_A - the dynamics matrix
 \param[in] i_B - the input matrix
 \param[in] i_type - indicates whether the callback is continuous or
                      discrete, see CoreRobotics::CRMotionModelType.
 \param[in] i_x0 - the initial state.
 \param[in] i_timeStep - the time step of the system
 */
//------------------------------------------------------------------------------
MotionLinear::MotionLinear(const Eigen::MatrixXd &i_A,
                           const Eigen::MatrixXd &i_B,
                           const Eigen::VectorXd &i_x0,
                           const Eigen::VectorXd &i_u0, const double i_dt,
                           DynamicalSystem::ModelType i_type)
    : DynamicalSystem(i_x0, i_u0, i_dt, i_type) {
  this->m_time = 0;
  this->m_A = i_A;
  this->m_B = i_B;
}

//------------------------------------------------------------------------------
/*!
 This method performs the linear model.\n

 \param[in] i_t - time t(k)
 \param[in] i_x - state x(k)
 \param[in] i_u - input u(k)
 \return - the state derivative \dot{x} or next state x_{k+1}
 */
//------------------------------------------------------------------------------
Eigen::VectorXd MotionLinear::motionCallback(double i_t, Eigen::VectorXd i_x,
                                             Eigen::VectorXd i_u) {
  return m_A * i_x + m_B * i_u;
}

} // namespace model
} // namepsace cr
