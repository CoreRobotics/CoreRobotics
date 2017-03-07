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
#include "CRMotionLinear.hpp"
#include "CRMath.hpp"


//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics {
    
    
//=====================================================================
/*!
 The constructor creates a linear motion model.\n
 
 \param[in] A - dynamics matrix
 \param[in] B - input matrix
 \param[in] x0 - the initial state
 \param[in] dt - the time step (s)
 \param[in] type - the motion model type,
                    see: CoreRobotics::CRMotionModelType
 */
//---------------------------------------------------------------------
CRMotionLinear::CRMotionLinear(Eigen::MatrixXd A,
                               Eigen::MatrixXd B,
                               Eigen::VectorXd x0,
                               double dt,
                               CRMotionModelType type)
{
    this->setState(x0);
    this->setTimeStep(dt);
    this->setType(type);
    this->time = 0;
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
void CRMotionLinear::setDynamics(Eigen::MatrixXd A,
                                 Eigen::MatrixXd B)
{
    this->A = A;
    this->B = B;
}

    
//=====================================================================
// Private Members:

//! sets the linear dynamic model
Eigen::VectorXd CRMotionLinear::dynFcn(double t,
                                       Eigen::VectorXd x,
                                       Eigen::VectorXd u){
    return (this->A*x + this->B*u);
}

//=====================================================================
// End namespace
}
