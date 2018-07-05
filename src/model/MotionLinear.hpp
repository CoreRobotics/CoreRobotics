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

*/
//=====================================================================

#ifndef MotionLinear_hpp
#define MotionLinear_hpp

//=====================================================================
// Includes
#include "Eigen/Dense"
#include "MotionModel.hpp"

//=====================================================================
// CoreRobotics namespace
namespace cr {
    
//=====================================================================
/*!
 \file MotionLinear.hpp
 \brief Implements a class that handles linear motion models.
 */
//---------------------------------------------------------------------
/*!
 \class MotionLinear
 \ingroup models
 
 \brief This class implements a linear motion model.
 
 \details
 ## Description
 MotionLinear implements a motion model from a supplied dynamics
 callback function.  Specifically, MotionLinear sets up a container
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
 
 These methods are available for interfacing with the linear motion model:
 - MotionLinear::setState sets the underlying state vector.
 - MotionLinear::getState returns the state vector.
 - MotionLinear::setTimeStep sets the time step (s).
 - MotionLinear::getTimeStep returns the time step (s).
 - MotionLinear::getTime returns the simulation time (s).
 - MotionLinear::setDynamics sets the dynamics matrices (A, B).
 - MotionLinear::motion computes a new state and updates the
 internal value for an input (u).
 
 ## Example
 This example demonstrates use of the MotionLinear class.
 \include example_MotionLinear.cpp
 
 ## References
 [1] J. Crassidis and J. Junkins, "Optimal Estimation of Dynamic Systems",
 Ed. 2, CRC Press, 2012. \n\n
 
 [2] S. Thrun, W. Burgard, and D. Fox, "Probabilistic Robotics", MIT Press, 2006.
 \n\n
 */
//=====================================================================
class MotionLinear : public MotionModel {
    
//---------------------------------------------------------------------
// Constructor and Destructor
public:
    
    //! Class constructor
    MotionLinear(Eigen::MatrixXd i_A,
                   Eigen::MatrixXd i_B,
                   MotionModelType i_type,
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
    Eigen::VectorXd rk4step(double i_t,
                            Eigen::VectorXd i_x,
                            Eigen::VectorXd i_u,
                            double i_dt);
    
    
//---------------------------------------------------------------------
// Protected Members
protected:
    
    //! Dynamics Matrix
    Eigen::MatrixXd m_A;
    
    //! Input Matrix
    Eigen::MatrixXd m_B;
    
    //! Callback to the dynamic model function
    Eigen::VectorXd m_dynPredictFcn(double t,
                                    Eigen::VectorXd x,
                                    Eigen::VectorXd u){
        return this->m_A*x + this->m_B*u;
    }
};

//=====================================================================
// End namespace
};


#endif