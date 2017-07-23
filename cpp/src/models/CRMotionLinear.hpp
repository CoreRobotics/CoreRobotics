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
 ## Description
 CRMotionLinear implements a motion model from a supplied dynamics
 callback function.  Specifically, CRMotionLinear sets up a container
 for the continuous model
 
 \f[
 \dot{x} = Ax + Bu
 \f]
 
 or
 
 \f[
 x_{k+1} = A x_k + B u_k
 \f]
 
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
 
 ## Example
 This example demonstrates use of the CRMotionLinear class.
 \include test_CRMotionLinear.cpp
 
 ## References
 [1] J. Crassidis and J. Junkins, "Optimal Estimation of Dynamic Systems",
 Ed. 2, CRC Press, 2012. \n\n
 
 [2] S. Thrun, W. Burgard, and D. Fox, "Probabilistic Robotics", MIT Press, 2006.
 \n\n
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
    
    //! Callback to the dynamic model function \f$\dot{x} = f(x,u,t)\f$
    //  or \f$x_kp1 = f(x_k,u_k,t_k)\f$ depending on what the type is set to.
    Eigen::VectorXd m_dynPredictFcn(Eigen::VectorXd x,
                                    Eigen::VectorXd u,
                                    double t){
        return this->m_A*x + this->m_B*u;
    }
};

//=====================================================================
// End namespace
};


#endif
