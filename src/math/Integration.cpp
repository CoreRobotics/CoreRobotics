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

#include "Integration.hpp"
#include "Eigen/Dense"
#include "Eigen/SVD"


//---------------------------------------------------------------------
// Begin namespace
namespace cr {
namespace math {

    
//---------------------------------------------------------------------
/*!
 This method performs a forward euler integration step for the dynamical
 system:\n
 
 \f[
 \dot{x} = f(t,x,u)
 \f]
 
 This equation is specified as a callback function to the integration.
 
 \param [in] i_dynamicSystem - the continuous time dynamic system.
 \param [in] i_t - the current time in seconds
 \param [in] i_x - the current state vector
 \param [in] i_u - the current input vector
 \param [in] i_dt - the time step to integrate over
 \return the state vector at t+dt
 
 */
//---------------------------------------------------------------------
Eigen::VectorXd Integration::forwardEulerStep(Eigen::VectorXd(i_dynamicSystem)(double,
                                                                          Eigen::VectorXd,
                                                                          Eigen::VectorXd),
                                         double i_t,
                                         Eigen::VectorXd i_x,
                                         Eigen::VectorXd i_u,
                                         double i_dt){
    // forward integration step
    return i_x + i_dt*i_dynamicSystem(i_t,i_x,i_u);
}
    
Eigen::VectorXd Integration::forwardEulerStep(std::function<Eigen::VectorXd(double,
                                                                            Eigen::VectorXd,
                                                                            Eigen::VectorXd)> i_dynamicSystem,
                                              double i_t,
                                              Eigen::VectorXd i_x,
                                              Eigen::VectorXd i_u,
                                              double i_dt){
    // forward integration step
    return i_x + i_dt*i_dynamicSystem(i_t,i_x,i_u);
}
    
    
//---------------------------------------------------------------------
/*!
 This method performs a Runge-Kutta integration step for the dynamical
 system:\n
 
 \f[
 \dot{x} = f(t,x,u)
 \f]
 
 This equation is specified as a callback function to the integration.
 The Runga-Kutta integration is an accurate integration scheme at the
 expense of computational complexity over the forward Euler method.
 
 \param [in] i_dynamicSystem - the continuous time dynamic system
 \param [in] i_t - the current time in seconds
 \param [in] i_x - the current state vector
 \param [in] i_u - the current input vector
 \param [in] i_dt - the time step to integrate over
 \return the state vector at t+dt
 
 */
//---------------------------------------------------------------------
Eigen::VectorXd Integration::rungeKuttaStep(Eigen::VectorXd(i_dynamicSystem)(double,
                                                                        Eigen::VectorXd,
                                                                        Eigen::VectorXd),
                                       double i_t,
                                       Eigen::VectorXd i_x,
                                       Eigen::VectorXd i_u,
                                       double i_dt){
    // RK4 step
    Eigen::VectorXd f1 = i_dynamicSystem(i_t,i_x,i_u);
    Eigen::VectorXd f2 = i_dynamicSystem(i_t+i_dt/2,i_x+i_dt*f1/2,i_u);
    Eigen::VectorXd f3 = i_dynamicSystem(i_t+i_dt/2,i_x+i_dt*f2/2,i_u);
    Eigen::VectorXd f4 = i_dynamicSystem(i_t+i_dt,i_x+i_dt*f3,i_u);
    return i_x + i_dt/6*(f1 + 2*f2 + 2*f3 + f4);
}
    
Eigen::VectorXd Integration::rungeKuttaStep(std::function<Eigen::VectorXd(double,
                                                                          Eigen::VectorXd,
                                                                          Eigen::VectorXd)> i_dynamicSystem,
                                            double i_t,
                                            Eigen::VectorXd i_x,
                                            Eigen::VectorXd i_u,
                                            double i_dt){
    // RK4 step
    Eigen::VectorXd f1 = i_dynamicSystem(i_t, i_x, i_u);
    Eigen::VectorXd f2 = i_dynamicSystem(i_t+i_dt/2, i_x+i_dt*f1/2, i_u);
    Eigen::VectorXd f3 = i_dynamicSystem(i_t+i_dt/2, i_x+i_dt*f2/2, i_u);
    Eigen::VectorXd f4 = i_dynamicSystem(i_t+i_dt, i_x+i_dt*f3, i_u);
    return i_x + i_dt/6*(f1 + 2*f2 + 2*f3 + f4);
}


}
}
// end namespace
//---------------------------------------------------------------------


