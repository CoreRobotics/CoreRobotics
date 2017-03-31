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

#ifndef CRMotionProbabilistic_hpp
#define CRMotionProbabilistic_hpp

//=====================================================================
// Includes
#include "Eigen/Dense"
#include "CRMotionModel.hpp"

//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics {
    
//=====================================================================
/*!
 \file CRMotionProbabilistic.hpp
 \brief Implements a class that handles probabilistic motion models.
 */
//---------------------------------------------------------------------
/*!
 \class CRMotionProbabilistic
 \ingroup models
 
 \brief This class implements a probabilistic motion model.
 
 \details
 \section Description
 CRMotionModel implements a motion model from a supplied dynamics
 callback function.  Specifically, CRMotionModel sets up a container
 for the continuous model
 
 \f$ \dot{x} = f(x,u,t) \f$
 
 or
 
 \f$ x_{k+1} = f(x_k,u_k,t_k) \f$
 
 where \f$x\f$ is the state vector, \f$u\f$ is the input vector,
 \f$t\f$ is time, and \f$k\f$ is a discrete sampling index.
 
 These methods are available for interfacing with the Motion Model:
 - CRMotionModel::setState sets the underlying state vector.
 - CRMotionModel::getState returns the state vector.
 - CRMotionModel::setTimeStep sets the time step (s).
 - CRMotionModel::getTimeStep returns the time step (s).
 - CRMotionModel::gettime returns the simulation time (s).=
 - CRMotionModel::motion computes a new state and updates the
 internal value for an input (u).
 
 \section Example
 This example demonstrates use of the CRMotionModel class.
 \code
 
 #include <iostream>
 #include "CoreRobotics.hpp"
 
 // Use the CoreRobotics namespace
 using namespace CoreRobotics;
 
 
 // -------------------------------------------------------------
 // Declare a continuous motion model - xdot = fcn(x,u,t)
 Eigen::VectorXd dynFcn(Eigen::VectorXd x, Eigen::VectorXd u, double t){
     return -x + u;  // motion
 }
 
 
 // -------------------------------------------------------------
 void main(void){
 
     std::cout << "*************************************\n";
     std::cout << "Demonstration of CRMotionModel.\n";
     
     
     // initialize a state vector
     Eigen::VectorXd x(1);
     x << 10;
     
     
     // initialize a deterministic sensor model
     CRMotionModel model = CRMotionModel(*dynFcn,CR_MOTION_CONTINUOUS,x,0.2);
     
     
     // initialize an input and set it to zero
     Eigen::VectorXd u(1);
     u << 0;
     
     // Initialize a time t
     double t = 0;
     
     // loop
     printf("Time (s) | State\n");
     while(t <= 5) {
     
         // output the time and state
         printf("%5.1f    | %5.2f\n",t,x(0));
         
         // step at t = 2.5
         if (t >= 2.5){
             u << 10;
         }
         
         // get next state & time
         x = model.motion(u);
         t = model.getTime();
     }
     printf("%5.1f    | %5.2f\n",t,x(0));
 
 }
 // -------------------------------------------------------------
 
 \endcode
 
 \section References
 [1] J. Crassidis and J. Junkins, "Optimal Estimation of Dynamic Systems",
 Ed. 2, CRC Press, 2012. \n\n
 
 [2] S. Thrun, W. Burgard, and D. Fox, "Probabilistic Robotics", MIT Press,
 2006. \n\n
 */
//=====================================================================
class CRMotionProbabilistic : public CRMotionModel {
    
//---------------------------------------------------------------------
// Constructor and Destructor
public:
    
    //! Class constructor
    CRMotionProbabilistic(Eigen::VectorXd(in_dynamics)(Eigen::VectorXd,
                                                       Eigen::VectorXd,
                                                       double,
                                                       bool),
                          CRMotionModelType in_type,
                          Eigen::VectorXd in_x0,
                          double in_timeStep);
    CRMotionProbabilistic();
    
//---------------------------------------------------------------------
// Public Methods
public:
    
    //! Simulate the motion
    Eigen::VectorXd motion(Eigen::VectorXd in_u, bool sampleNoise);
    
    Eigen::VectorXd motion(Eigen::VectorXd in_u);
    
//---------------------------------------------------------------------
// Protected Methods
protected:
    
    //! A Runge-Kutta solver on the dynFcn
    Eigen::VectorXd rk4step(Eigen::VectorXd in_x,
                            Eigen::VectorXd in_u,
                            double in_t,
                            double in_dt,
                            bool in_sample);
    
//---------------------------------------------------------------------
// Protected Members
protected:
    
    //! Callback to the dynamic model function \dot{x} = f(x,u,t,w) or
    //! x_kp1 = f(x_k,u_k,t_k,w_k) depending on what the type is set to.
    Eigen::VectorXd(*m_dynPredictFcn)(Eigen::VectorXd,
                                      Eigen::VectorXd,
                                      double,
                                      bool);
    
};

//=====================================================================
// End namespace
}


#endif
