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

#include "CRMotionModel.hpp"
#include "CRMath.hpp"


//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics {
    
    
//=====================================================================
/*!
 The constructor creates a motion model.\n
 
 \param[in] dynamicFcn - a callback function for the dynamics: see
    CRMotionModel::setDynamics() for more information.
 \param[in] x0 - the initial state
 \param[in] dt - the time step (s)
 \param[in] type - the motion model type, see: CoreRobotics::CRMotionModelType
 \param[in] noise - the process noise model, see CoreRobotics::CRNoiseModel
 */
//---------------------------------------------------------------------
CRMotionModel::CRMotionModel(Eigen::VectorXd(in_dynamicFcn)(double,
                                                            Eigen::VectorXd,
                                                            Eigen::VectorXd),
                             Eigen::VectorXd in_x0,
                             double in_dt,
                             CRMotionModelType in_type,
                             CRNoiseModel* in_noise) {
    this->setDynamics(in_dynamicFcn);
    this->setState(in_x0);
    this->setTimeStep(in_dt);
    this->setType(in_type);
    this->setProcessNoise(in_noise);
    this->m_time = 0;
}
CRMotionModel::CRMotionModel(Eigen::VectorXd(in_dynamicFcn)(double,
                                                            Eigen::VectorXd,
                                                            Eigen::VectorXd),
                             Eigen::VectorXd in_x0,
                             double in_dt,
                             CRMotionModelType in_type) {
    this->setDynamics(in_dynamicFcn);
    this->setState(in_x0);
    this->setTimeStep(in_dt);
    this->setType(in_type);
    this->m_time = 0;
}
CRMotionModel::CRMotionModel()
{
    this->m_time = 0;
}
    
    
//=====================================================================
/*!
 This method sets the callback to the dynamic equation function.  If 
 type is specified as CR_MODEL_DISCRETE, then this function must
 describe the difference equation:\n
 
 \f$ x_{k+1} =  f(t_k,x_k,u_k) \f$
 
 If the type is specified as CR_MODEL_CONTINUOUS, then this function
 must describe the differential equation:\n
 
 \f$ \dot{x} =  f(t,x,u) \f$
 
 \param[in] dynamicFcn - a callback function of the above form.
 */
//---------------------------------------------------------------------
void CRMotionModel::setDynamics(Eigen::VectorXd(in_dynamicFcn)(double,
                                                               Eigen::VectorXd,
                                                               Eigen::VectorXd))
{
    this->m_dynFcn = in_dynamicFcn;
}


//=====================================================================
/*!
 This method simulates the motion forward by one step from the current
 state.  If the motion model is discrete, this just evaluates the 
 dynamics equation (callback) for the specified input u.  However,
 if the model is continuous, then a Runge-Kutta integration scheme is 
 used to simulate the next time step.  The sampleNoise flag can be set
 to simulate the motion with additive noise sampled from the supplied 
 noise model.  If the noise model is not defined, the flag does 
 nothing.\n
 
 \param[in] u - input (forcing term) vector
 \param[in] sampleNoise - a boolean flag specifying if the noise model
                should be sampled to add noise to the simulation.
 */
//---------------------------------------------------------------------
void CRMotionModel::simulateMotion(Eigen::VectorXd in_u, bool in_sampleNoise)
{
    double t = this->m_time;
    double dt = this->m_timeStep;
    
    // sample the noise
    Eigen::VectorXd w;
    if (this->m_usingNoise && in_sampleNoise){
        w = this->m_processNoise->sample();
    } else {
        w = this->m_state;
        w.setZero();
    }
    
    if (this->m_type == CR_MODEL_DISCRETE) {
        // update the state
        this->m_state = (this->m_dynFcn)(t,this->m_state,in_u) + w;
        
    } else if (this->m_type == CR_MODEL_CONTINUOUS) {
        // update the state
        this->m_state = this->rk4step(t,this->m_state,in_u) + w;
    }
    
    // update the time
    this->m_time = t+dt;
}
    
    
//=====================================================================
/*!
 This method performs a Runge Kutta step on the dynFcn member.\n
 */
//---------------------------------------------------------------------
Eigen::VectorXd CRMotionModel::rk4step(double in_t,
                                       Eigen::VectorXd in_x,
                                       Eigen::VectorXd in_u)
{
    double dt = this->m_timeStep;
    // RK4 step
    Eigen::VectorXd f1 = (this->m_dynFcn)(in_t,in_x,in_u);
    Eigen::VectorXd f2 = (this->m_dynFcn)(in_t+dt/2,in_x+dt*f1/2,in_u);
    Eigen::VectorXd f3 = (this->m_dynFcn)(in_t+dt/2,in_x+dt*f2/2,in_u);
    Eigen::VectorXd f4 = (this->m_dynFcn)(in_t+dt,in_x+dt*f3,in_u);
    return in_x + dt/6*(f1 + 2*f2 + 2*f3 + f4);
}


//=====================================================================
// End namespace
}
