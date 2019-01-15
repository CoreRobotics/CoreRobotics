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

#ifndef CRSensorModel_hpp
#define CRSensorModel_hpp

//=====================================================================
// Includes
#include "core/CRTypes.hpp"
#include "Eigen/Dense"

//=====================================================================
// CoreRobotics namespace
namespace [[deprecated(CR_DEPRECATED)]] CoreRobotics {
    
//=====================================================================
/*!
 \file CRSensorModel.hpp
 \brief Implements a class that handles sensor models.
 */
//---------------------------------------------------------------------
/*!
 \class CRSensorModel
 \ingroup models
 
 \brief This class implements a sensor model.
 
 \details
 ## Description
 CRSensorModel implements a sensor model from a supplied observation
 callback function.  Specifically, CRSensorModel sets up a container
 for the model
 
 \f$ z = h(x) \f$,
 
 where \f$x\f$ is the state vector, and \f$z\f$ is the sensor 
 measurement vector.
 
 These methods are used to interface with the Sensor Model:
 - CRSensorModel::setState sets the underlying state vector.
 - CRSensorModel::getState outputs the state vector.
 - CRSensorModel::measurement computes a simulated measurement 
 vector (z) from the underlying state (x).
 
 ## Example
 This example demonstrates use of the CRSensorModel class.
 \include example_CRSensorModel.cpp
 
 ## References
 [1] J. Crassidis and J. Junkins, "Optimal Estimation of Dynamic Systems",
 Ed. 2, CRC Press, 2012. \n\n
 
 [2] S. Thrun, W. Burgard, and D. Fox, "Probabilistic Robotics", MIT Press,
 2006. \n\n
 */
//=====================================================================
class [[deprecated(CR_DEPRECATED)]] CRSensorModel {
    
//---------------------------------------------------------------------
// Constructor and Destructor
public:
    
    //! Class constructor
    CRSensorModel(Eigen::VectorXd(i_predictor)(Eigen::VectorXd),
                  Eigen::VectorXd i_x0);
    CRSensorModel();
    
//---------------------------------------------------------------------
// Get/Set Methods
public:
    
    //! Set the state vector (x)
    void setState(Eigen::VectorXd i_x) {this->m_state = i_x;}
    
    //! Get the state vector (x)
    Eigen::VectorXd getState(void) {return this->m_state;}
    
//---------------------------------------------------------------------
// Public Methods
public:
    
    //! Simulate the measurement
    Eigen::VectorXd measurement(void);
    
//---------------------------------------------------------------------
// Protected Members
protected:
    
    //! Underlying state of the system
    Eigen::VectorXd m_state;
    
    //! Callback to the deterministic predictor function z = h(x)
    Eigen::VectorXd(*m_measPredictFcn)(Eigen::VectorXd);
    
};

//=====================================================================
// End namespace
}


#endif
