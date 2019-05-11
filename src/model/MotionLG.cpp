/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#include "MotionLG.hpp"

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
MotionLG::MotionLG(const MotionLGParameters &i_parameters,
                   const noise::Gaussian &i_state, 
                   const Eigen::VectorXd &i_action,
                   const double i_dt)
  : Motion<noise::Gaussian, Eigen::VectorXd, MotionLGParameters>(
    i_parameters, i_state, i_action, i_dt) {}

//------------------------------------------------------------------------------
/*!
 This method performs the linear Gaussian model.\n

 \param[in] i_t - time t(k)
 \param[in] i_x - state x(k)
 \param[in] i_u - input u(k)
 \return - the next state x_{k+1}
 */
//------------------------------------------------------------------------------
noise::Gaussian MotionLG::motionCallback(double i_t,
                                             noise::Gaussian i_x,
                                             Eigen::VectorXd i_u) {
  Eigen::VectorXd x = i_x.getParameters().mean;
  Eigen::MatrixXd P = i_x.getParameters().cov;
  noise::Gaussian g;
  auto gp = g.getParameters();
  gp.mean = m_parameters.m_A * x + m_parameters.m_B * i_u;
  gp.cov = m_parameters.m_A * P * m_parameters.m_A.transpose()
    + m_parameters.m_C * m_parameters.m_Q * m_parameters.m_C.transpose();
  g.setParameters(gp);
  return g;
}

} // namespace model
} // namepsace cr
