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

#ifndef CRMotionLinear_hpp
#define CRMotionLinear_hpp

//=====================================================================
// Includes
#include "Eigen/Dense"
#include "CRMotionModel.hpp"

//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics {
    
//=====================================================================
/*!
 \file CRMotionLinear.hpp
 \brief Implements a class that handles linear motion models.
 */
//---------------------------------------------------------------------
/*!
 \class CRMotionLinear
 \ingroup models
 
 \brief This class implements a linear motion model.
 
 \details
 \section Description
 CRMotionLinear implements a motion model from a supplied dynamics
 callback function.  Specifically, CRMotionLinear sets up a container
 for the continuous model
 
 \f$ \dot{x} = Ax + Bu \f$
 
 or
 
 \f$ x_{k+1} = A x_k + B u_k \f$
 
 where \f$x\f$ is the state vector, \f$u\f$ is the input vector,
 and \f$k\f$ is a discrete sampling index.
 
 These methods are available for interfacing with the Motion Model:
 - CRMotionLinear::setState sets the underlying state vector.
 - CRMotionLinear::getState returns the state vector.
 - CRMotionLinear::setTimeStep sets the time step (s).
 - CRMotionLinear::getTimeStep returns the time step (s).
 - CRMotionLinear::gettime returns the simulation time (s).=
 - CRMotionLinear::motion computes a new state and updates the
 internal value for an input (u).
 
 \section Example
 This example demonstrates use of the CRMotionLinear class.
 \code
 
 #include <iostream>
 #include "CoreRobotics.hpp"
 
 // Use the CoreRobotics namespace
 using namespace CoreRobotics;
 
 
 // -------------------------------------------------------------
 void main(void){
 
     std::cout << "*************************************\n";
     std::cout << "Demonstration of CRMotionLinear.\n";
     
     
     // initialize a state vector
     Eigen::VectorXd x(1);
     x << 10;
     
     // Dynamics Matrix
     Eigen::Matrix<double,1,1> A;
     A << -1;
     
     // Input matrix
     Eigen::Matrix<double,1,1> B;
     B << 1;
     
     // initialize a linear dynamics model
     CRMotionLinear model = CRMotionLinear(A,B,CR_MOTION_CONTINUOUS,x,0.2);
     
     
     // initialize an input and set it to zero
     Eigen::VectorXd u(1);
     u << 0;
     
     // Initialize a time t
     double t = 0;
     
     // loop
     printf("Time (s) | State\n");
     while(t <= 5) {
     
         // output the time and state
         printf("%5.1f    | %5.1f | %5.2f\n",t,u(0),x(0));
         
         // step at t = 2.5
         if (t >= 2.5){
             u << 10;
         }
         
         // get next state & time
         x = model.motion(u);
         t = model.getTime();
     }
     printf("%5.1f    | %5.1f | %5.2f\n",t,u(0),x(0));
     
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
class CRMotionLinear : public CRMotionModel {
    
//---------------------------------------------------------------------
// Constructor and Destructor
public:
    
    //! Class constructor
    CRMotionLinear(Eigen::MatrixXd i_A,
                   Eigen::MatrixXd i_B,
                   CRMotionModelType i_type,
                   Eigen::VectorXd i_x0,
                   double i_timeStep);
    
//---------------------------------------------------------------------
// Get/Set Methods
public:
    
    //! Set the dynamics and input matrices
    void setDynamics(Eigen::MatrixXd i_A, Eigen::MatrixXd i_B){
        this->m_A = i_A;
        this->m_B = i_B;
    }
    
    
//---------------------------------------------------------------------
// Public Methods
public:
    
    //! Simulate the motion
    Eigen::VectorXd motion(Eigen::VectorXd i_u);
    
//---------------------------------------------------------------------
// Protected Methods
protected:
    
    //! A Runge-Kutta solver on the dynFcn
    Eigen::VectorXd rk4step(Eigen::VectorXd i_x,
                            Eigen::VectorXd i_u,
                            double i_t,
                            double i_dt);
    
//---------------------------------------------------------------------
// Protected Members
protected:
    
    //! Dynamics Matrix
    Eigen::MatrixXd m_A;
    
    //! Input Matrix
    Eigen::MatrixXd m_B;
    
    //! Callback to the dynamic model function \dot{x} = f(x,u,t) or
    //! x_kp1 = f(x_k,u_k,t_k) depending on what the type is set to.
    Eigen::VectorXd m_dynPredictFcn(Eigen::VectorXd x,
                                    Eigen::VectorXd u,
                                    double t)
    {
        return this->m_A*x + this->m_B*u;
    };
};

//=====================================================================
// End namespace
}


#endif
