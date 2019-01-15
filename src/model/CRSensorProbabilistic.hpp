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

#ifndef CRSensorProbabilistic_hpp
#define CRSensorProbabilistic_hpp

//=====================================================================
// Includes
#include "core/CRTypes.hpp"
#include "Eigen/Dense"
#include "CRSensorModel.hpp"

//=====================================================================
// CoreRobotics namespace
namespace [[deprecated(CR_DEPRECATED)]] CoreRobotics {
    
//=====================================================================
/*!
 \file CRSensorProbabilistic.hpp
 \brief Implements a class that handles probabilistic sensor models.
 */
//---------------------------------------------------------------------
/*!
 \class CRSensorProbabilistic
 \ingroup models
 
 \brief This class implements a probabilistic sensor model.
 
 \details
 ## Description
 CRSensorProbabilistic implements a probabilistic sensor model from 
 a supplied observation callback function and likelihood callback
 functions.  Specifically, CRSensorProbabilistic sets up a container
 for the model
 
 \f$ z = h(x,w) \f$,
 
 where \f$x\f$ is the state vector, and \f$z\f$ is the sensor
 measurement vector. and \f$w\f$ is noise.  Additionally, a probabilistic
 model
 
 \f$ p(z \mid x) \f$
 
 must be specified to return the probability of observing a measurement
 \f$z\f$ given the current state \f$x\f$.
 
 These methods are used to interface with the Probabilistic Sensor Model:
 - CRSensorProbabilistic::setState sets the underlying state vector.
 - CRSensorProbabilistic::getState outputs the state vector.
 - CRSensorProbabilistic::measurement computes a simulated measurement
 vector (z) from the underlying state (x).
 - CRSensorProbabilistic::likelihood computes the probability of 
 observing measurement (z) for the given state (x).
 
 ## Example
 This example demonstrates use of the CRSensorProbabilistic class.
 \include example_CRSensorProbabilistic.cpp
 
 ## References
 [1] J. Crassidis and J. Junkins, "Optimal Estimation of Dynamic Systems",
 Ed. 2, CRC Press, 2012. \n\n
 
 [2] S. Thrun, W. Burgard, and D. Fox, "Probabilistic Robotics", MIT Press, 2006.
 \n\n
 */
//=====================================================================
class [[deprecated(CR_DEPRECATED)]] CRSensorProbabilistic : public CRSensorModel {
    
//---------------------------------------------------------------------
// Constructor and Destructor
public:
    
    //! Class constructor
    CRSensorProbabilistic(Eigen::VectorXd(i_predictor)(Eigen::VectorXd,
                                                        bool),
                          double(i_likelihood)(Eigen::VectorXd,
                                                Eigen::VectorXd),
                          Eigen::VectorXd i_x0);
    
//---------------------------------------------------------------------
// Public Methods
public:
    
    //! Simulate the measurement
    Eigen::VectorXd measurement(bool i_sampleNoise);
    
    Eigen::VectorXd measurement(void);
    
    //! Get the likelihood of a measurement
    double likelihood(Eigen::VectorXd i_z);
    
//---------------------------------------------------------------------
// Protected Members
protected:
    
    //! Callback to the probabilistic predictor function z = h(x,v)
    Eigen::VectorXd(*m_measPredictFcn)(Eigen::VectorXd,
                                       bool);
    
    //! Callback to the probabilistic likelihood function p(zObserved|h(x))
    double(*m_measLikelihoodFcn)(Eigen::VectorXd,
                                 Eigen::VectorXd);
    
};

//=====================================================================
// End namespace
}


#endif
