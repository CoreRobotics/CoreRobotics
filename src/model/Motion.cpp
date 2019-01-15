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
//---------------------------------------------------------------------

#include "Motion.hpp"
#include "math/Integration.hpp"

//---------------------------------------------------------------------
// Begin namespace
namespace cr {
namespace model {
    
    
//---------------------------------------------------------------------
/*!
 The constructor creates a motion model.\n
 
 \param[in] i_x0 - the initial state.
 \param[in] i_u0 - the initial action.
 \param[in] i_timeStep - the time step of the system
 */
//---------------------------------------------------------------------
Motion::Motion(Eigen::VectorXd i_x0,
               Eigen::VectorXd i_u0,
               double i_dt)
{
    m_time = 0;
    m_state = i_x0;
    m_action = i_u0;
    m_dt = i_dt;
}


//---------------------------------------------------------------------
/*!
 This method steps the motion model, updating the internal state.
 */
//---------------------------------------------------------------------
void Motion::step()
{
    
    // use the bound member function pointer
    // https://stackoverflow.com/questions/7582546/using-generic-stdfunction-objects-with-member-functions-in-one-class
    //
    // (lambdas create a memory leak with shared pointers!
    // http://floating.io/2019/07/lambda-shared_ptr-memory-leak/)
    m_state = Integration::rungeKuttaStep(m_fcn, m_time, m_state, m_action, m_dt);
    
    // update the time
    m_time = m_time + m_dt;
}


}
}
// end namespace
//---------------------------------------------------------------------