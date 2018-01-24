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

#ifndef CRMotionModel_hpp
#define CRMotionModel_hpp

//=====================================================================
// Includes
#include "Eigen/Dense"

//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics {
    
//=====================================================================
/*!
 \file CRMotionModel.hpp
 \brief Implements a class that handles motion models.
 */
//---------------------------------------------------------------------
/*!
 \class CRMotionModel
 \ingroup models
 
 \brief This class implements a motion model.
 
 \details
 ## Description
 CRMotionModel implements a motion model from a supplied dynamics
 callback function.  Specifically, CRMotionModel sets up a container
 for the continuous model
 
 \f[
 \dot{x} = f(t,x,u)
 \f]
 
 or
 
 \f[
 x_{k+1} = f(t_k,x_k,u_k)
 \f]
 
 where \f$x\f$ is the state vector, \f$u\f$ is the input vector,
 \f$t\f$ is time, and \f$k\f$ is a discrete sampling index.
 
 These methods are available for interfacing with the Motion Model:
 - CRMotionModel::setState sets the underlying state vector.
 - CRMotionModel::getState returns the state vector.
 - CRMotionModel::setTimeStep sets the time step (s).
 - CRMotionModel::getTimeStep returns the time step (s).
 - CRMotionModel::getTime returns the simulation time (s).
 - CRMotionModel::motion computes a new state and updates the
 internal value for an input (u).
 
 ## Example
 This example demonstrates use of the CRMotionModel class.
 \include example_CRMotionModel.cpp
 
 ## References
 [1] J. Crassidis and J. Junkins, "Optimal Estimation of Dynamic Systems",
 Ed. 2, CRC Press, 2012. \n\n
 
 [2] S. Thrun, W. Burgard, and D. Fox, "Probabilistic Robotics", MIT Press, 2006.
 \n\n
 */
//=====================================================================
//! Enumerator for specifying whether the specified dynamic model is
//  either continuous or discrete.
enum CRMotionModelType {
    CR_MOTION_CONTINUOUS,
    CR_MOTION_DISCRETE
};

//=====================================================================
class CRMotionModel {
    
//---------------------------------------------------------------------
// Constructor and Destructor
public:
    
    //! Class constructor
    CRMotionModel(Eigen::VectorXd(i_dynamics)(double,
                                              Eigen::VectorXd,
                                              Eigen::VectorXd),
                  CRMotionModelType i_type,
                  Eigen::VectorXd i_x0,
                  double i_timeStep);
    CRMotionModel();
    
//---------------------------------------------------------------------
// Get/Set Methods
public:
    
    //! Set the state vector (x)
    void setState(Eigen::VectorXd i_x) {this->m_state = i_x;}
    
    //! Get the state vector (x)
    Eigen::VectorXd getState(void) {return this->m_state;}
    
    //! Set the time step (s)
    void setTimeStep(double i_timeStep) {this->m_dt = i_timeStep;}
    
    //! Get the time step (s)
    double getTimeStep(void) {return this->m_dt;}
    
    //! Get the model time (s)
    double getTime(void) {return this->m_time;}
    
    
    
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
    
    //! Dynamic model function callback type
    CRMotionModelType m_type;
    
    //! Sample rate (s)
    double m_dt;
    
    //! Current time (s)
    double m_time;
    
    //! Dynamic state of the system
    Eigen::VectorXd m_state;
    
    //! Callback to the dynamic model function
    Eigen::VectorXd(*m_dynPredictFcn)(double,
                                      Eigen::VectorXd,
                                      Eigen::VectorXd);
    
};

//=====================================================================
// End namespace
};


#endif
