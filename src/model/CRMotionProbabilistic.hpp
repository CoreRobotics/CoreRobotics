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

#ifndef CRMotionProbabilistic_hpp
#define CRMotionProbabilistic_hpp

//=====================================================================
// Includes
#include "core/CRTypes.hpp"
#include "Eigen/Dense"
#include "CRMotionModel.hpp"

//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics  {
    
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
 ## Description
 CRMotionModel implements a probabilistic motion model from a supplied
 dynamics callback function.  Specifically, CRMotionModel sets up a 
 container for the continuous model
 
 \f[
 \dot{x} = f(t,x,u,w)
 \f]
 
 or
 
 \f[
 x_{k+1} = f(t_k,x_k,u_k,w_k)
 \f]
 
 where \f$x\f$ is the state vector, \f$u\f$ is the input vector,
 \f$t\f$ is time, \f$w\f$ is process noise, and \f$k\f$ is a discrete
 sampling index.
 
 These methods are available for interfacing with the Motion Model:
 - CRMotionProbabilistic::setState sets the underlying state vector.
 - CRMotionProbabilistic::getState returns the state vector.
 - CRMotionProbabilistic::setTimeStep sets the time step (s).
 - CRMotionProbabilistic::getTimeStep returns the time step (s).
 - CRMotionProbabilistic::getTime returns the simulation time (s).
 - CRMotionProbabilistic::motion computes a new state and updates the
 internal value for an input (u) and a flag that indicates whether noise
 should be sampled.
 
 ## Example
 This example demonstrates use of the CRMotionModel class.
 \include example_CRMotionProbabilistic.cpp
 
 ## References
 [1] J. Crassidis and J. Junkins, "Optimal Estimation of Dynamic Systems",
 Ed. 2, CRC Press, 2012. \n\n
 
 [2] S. Thrun, W. Burgard, and D. Fox, "Probabilistic Robotics", MIT Press,
 2006. \n\n
 */
//=====================================================================
class [[deprecated(CR_DEPRECATED)]] CRMotionProbabilistic : public CRMotionModel {
    
//---------------------------------------------------------------------
// Constructor and Destructor
public:
    
    //! Class constructor
    CRMotionProbabilistic(Eigen::VectorXd(i_dynamics)(double,
                                                      Eigen::VectorXd,
                                                      Eigen::VectorXd,
                                                      bool),
                          CRMotionModelType i_type,
                          Eigen::VectorXd i_x0,
                          double i_timeStep);
    CRMotionProbabilistic();
    
//---------------------------------------------------------------------
// Public Methods
public:
    
    //! Simulate the motion
    Eigen::VectorXd motion(Eigen::VectorXd i_u, bool sampleNoise);
    
    Eigen::VectorXd motion(Eigen::VectorXd i_u);
    
//---------------------------------------------------------------------
// Protected Methods
protected:
    
    //! A Runge-Kutta solver on the dynFcn
    Eigen::VectorXd rk4step(double i_t,
                            Eigen::VectorXd i_x,
                            Eigen::VectorXd i_u,
                            double i_dt,
                            bool i_sample);
    
//---------------------------------------------------------------------
// Protected Members
protected:
    
    //! Callback to the dynamic model function
    Eigen::VectorXd(*m_dynPredictFcn)(double,
                                      Eigen::VectorXd,
                                      Eigen::VectorXd,
                                      bool);
    
};

//=====================================================================
// End namespace
};


#endif
