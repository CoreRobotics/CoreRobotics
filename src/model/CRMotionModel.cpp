//=====================================================================
/*
Software License Agreement (BSD-3-Clause License)
Copyright (c) 2019, CoreRobotics.
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

*/
//=====================================================================

#include "CRMotionModel.hpp"


//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics  {
    
    
//=====================================================================
/*!
 The constructor creates a motion model.  The i_dynamics specifies
 one of the dynamics equation forms below:\n
 
 Case 1: (Continuous)
 
 If i_type is set to CR_MOTION_CONTINUOUS, then the callback sets
 
 \f$ \dot{x} = f(t,x,u) \f$
 
 where \f$x\f$ is the system state, \f$u\f$ is the input (forcing)
 vector, and \f$t\f$ is time.
 
 
 Case 2: (Discrete)
 
 If i_type is set to CR_MOTION_DISCRETE, then the callback sets
 
 \f$ x_{k+1} = f(t_k,x_k,u_k) \f$
 
 where \f$x_k\f$ is the current system state, \f$u_k\f$ is the 
 input (forcing) vector, and \f$t_k\f$ is time at interval \f$k\f$.
 
 
 \param[in] i_dynamics - callback to the dynamics equation
 \param[in] i_type - indicates whether the callback is continuous or 
                      discrete, see CoreRobotics::CRMotionModelType.
 \param[in] i_x0 - the initial state.
 \param[in] i_timeStep - the time step of the system
 */
//---------------------------------------------------------------------
CRMotionModel::CRMotionModel(Eigen::VectorXd(i_dynamics)(double,
                                                         Eigen::VectorXd,
                                                         Eigen::VectorXd),
                             CRMotionModelType i_type,
                             Eigen::VectorXd i_x0,
                             double i_timeStep)
{
    this->m_time = 0;
    this->m_dynPredictFcn = i_dynamics;
    this->m_type = i_type;
    this->setTimeStep(i_timeStep);
    this->setState(i_x0);
}

// overloaded constructor for initializing derived classes
CRMotionModel::CRMotionModel()
{
    this->m_time = 0;
    this->m_dt = 0.01;
}


//=====================================================================
/*!
 This method simulates the motion forward by one step from the current
 state.  If the motion model is discrete, this just evaluates the
 dynamics equation (callback) for the specified input u.  However,
 if the model is continuous, then a Runge-Kutta integration scheme is
 used to simulate the next time step.\n
 
 \param[in] i_u - input (forcing term) vector
 \return - the new state
 */
//---------------------------------------------------------------------
Eigen::VectorXd CRMotionModel::motion(Eigen::VectorXd i_u)
{
    double t = this->m_time;
    double dt = this->m_dt;
    
    if (this->m_type == CR_MOTION_DISCRETE) {
        // update the state
        m_state = (this->m_dynPredictFcn)(t, m_state, i_u);
        
    } else if (this->m_type == CR_MOTION_CONTINUOUS) {
        // update the state
        m_state = this->rk4step(t, m_state, i_u, dt);
    }
    
    // update the time
    this->m_time = t+dt;
    
    // return the new state
    return this->m_state;
}


//=====================================================================
/*!
 This method performs a Runge Kutta step on the dynFcn member.\n
 
 \param[in] i_t - time t(k)
 \param[in] i_x - state x(k)
 \param[in] i_u - input u(k)
 \param[in] i_dt - sample rate dt
 \return - the next state x(k+1)
 */
//---------------------------------------------------------------------
Eigen::VectorXd CRMotionModel::rk4step(double i_t,
                                       Eigen::VectorXd i_x,
                                       Eigen::VectorXd i_u,
                                       double i_dt)
{
    // RK4 step
    Eigen::VectorXd f1 = (this->m_dynPredictFcn)(i_t, i_x, i_u);
    Eigen::VectorXd f2 = (this->m_dynPredictFcn)(i_t+i_dt/2,i_x+i_dt*f1/2,i_u);
    Eigen::VectorXd f3 = (this->m_dynPredictFcn)(i_t+i_dt/2,i_x+i_dt*f2/2,i_u);
    Eigen::VectorXd f4 = (this->m_dynPredictFcn)(i_t+i_dt,i_x+i_dt*f3,i_u);
    return i_x + i_dt/6*(f1 + 2*f2 + 2*f3 + f4);
}


//=====================================================================
// End namespace
}
