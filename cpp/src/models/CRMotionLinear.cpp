//=====================================================================
/*
 Software License Agreement (BSD-3-Clause License)
 Copyright (c) 2017, CoreRobotics.
 All rights reserved.
 
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:
 
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 
 * Neither the name of CoreRobotics nor the names of its contributors
 may be used to endorse or promote products derived from this
 software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
 
 \project CoreRobotics Project
 \url     www.corerobotics.org
 \author  Parker Owan
 \version 0.0
 
 */
//=====================================================================

#include "CRMotionLinear.hpp"


//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics {
    
    
//=====================================================================
/*!
 The constructor creates a motion model.  The in_A, in_B matrices specify
 one of the dynamics equation forms below:\n
 
 Case 1: (Continuous)
 
 If in_type is set to CR_MOTION_CONTINUOUS, then the callback sets
 
 \f$ \dot{x} = A x + B u \f$
 
 where \f$x\f$ is the system state, \f$u\f$ is the input (forcing)
 vector, and \f$t\f$ is time.
 
 
 Case 2: (Discrete)
 
 If in_type is set to CR_MOTION_DISCRETE, then the callback sets
 
 \f$ x_{k+1} = A x_k + B u_k \f$
 
 where \f$x_k\f$ is the current system state, \f$u_k\f$ is the 
 input (forcing) vector, and \f$t_k\f$ is time at interval \f$k\f$.
 
 
 \param[in] in_A - the dynamics matrix
 \param[in] in_B - the input matrix
 \param[in] in_type - indicates whether the callback is continuous or 
                      discrete, see CoreRobotics::CRMotionModelType.
 \param[in] in_x0 - the initial state.
 \param[in] in_timeStep - the time step of the system
 */
//---------------------------------------------------------------------
CRMotionLinear::CRMotionLinear(Eigen::MatrixXd in_A,
                               Eigen::MatrixXd in_B,
                               CRMotionModelType in_type,
                               Eigen::VectorXd in_x0,
                               double in_timeStep)
{
    this->m_time = 0;
    this->m_A = in_A;
    this->m_B = in_B;
    this->m_type = in_type;
    this->setTimeStep(in_timeStep);
    this->setState(in_x0);
}
    
    
//=====================================================================
/*!
 This method simulates the motion forward by one step from the current
 state.  If the motion model is discrete, this just evaluates the
 dynamics equation (callback) for the specified input u.  However,
 if the model is continuous, then a Runge-Kutta integration scheme is
 used to simulate the next time step.\n
 
 \param[in] in_u - input (forcing term) vector
 \return - the new state
 */
//---------------------------------------------------------------------
Eigen::VectorXd CRMotionLinear::motion(Eigen::VectorXd in_u)
{
    double t = this->m_time;
    double dt = this->m_dt;
    
    if (this->m_type == CR_MOTION_DISCRETE) {
        // update the state
        this->m_state = this->m_dynPredictFcn(this->m_state,in_u,t);
        
    } else if (this->m_type == CR_MOTION_CONTINUOUS) {
        // update the state
        this->m_state = this->rk4step(this->m_state,in_u,t,dt);
    }
    
    // update the time
    this->m_time = t+dt;
    
    // return the new state
    return this->m_state;
}


//=====================================================================
/*!
 This method performs a Runge Kutta step on the dynFcn member.\n
 
 \param[in] in_x - state x(k)
 \param[in] in_u - input u(k)
 \param[in] in_t - time t(k)
 \param[in] in_dt - sample rate dt
 \return - the next state x(k+1)
 */
//---------------------------------------------------------------------
Eigen::VectorXd CRMotionLinear::rk4step(Eigen::VectorXd in_x,
                                       Eigen::VectorXd in_u,
                                       double in_t,
                                       double in_dt)
{
    // RK4 step
    Eigen::VectorXd f1 = this->m_dynPredictFcn(in_x,in_u,in_t);
    Eigen::VectorXd f2 = this->m_dynPredictFcn(in_x+in_dt*f1/2,in_u,in_t+in_dt/2);
    Eigen::VectorXd f3 = this->m_dynPredictFcn(in_x+in_dt*f2/2,in_u,in_t+in_dt/2);
    Eigen::VectorXd f4 = this->m_dynPredictFcn(in_x+in_dt*f3,in_u,in_t+in_dt);
    return in_x + in_dt/6*(f1 + 2*f2 + 2*f3 + f4);
}


//=====================================================================
// End namespace
}
