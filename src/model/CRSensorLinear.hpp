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
//=====================================================================

#ifndef CRSensorLinear_hpp
#define CRSensorLinear_hpp

//=====================================================================
// Includes
#include "CRSensorModel.hpp"
#include "Eigen/Dense"
#include "core/CRTypes.hpp"

//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics {

//=====================================================================
/*!
 \file CRSensorLinear.hpp
 \brief Implements a class that handles sensor models.
 */
//---------------------------------------------------------------------
/*!
 \class CRSensorLinear
 \ingroup model

 \brief This class implements a sensor model.

 \details
 ## Description
 CRSensorLinear implements a sensor model from a supplied observation
 callback function.  Specifically, CRSensorLinear sets up a container
 for the linear observation model

 \f$ z = H x \f$,

 where \f$x\f$ is the state vector, and \f$z\f$ is the sensor
 measurement vector.

 These methods are used to interface with the Sensor Model:
 - CRSensorLinear::setState sets the underlying state vector.
 - CRSensorLinear::getState outputs the state vector.
 - CRSensorLinear::measurement computes a simulated measurement
 vector (z) from the underlying state (x).
 - CRSensorLinear::setObservation sets the observation matrix (H).

 ## Example
 This example demonstrates use of the CRSensorLinear class.
 \include example_CRSensorLinear.cpp

 ## References
 [1] J. Crassidis and J. Junkins, "Optimal Estimation of Dynamic Systems",
 Ed. 2, CRC Press, 2012. \n\n

 [2] S. Thrun, W. Burgard, and D. Fox, "Probabilistic Robotics", MIT Press,
 2006. \n\n
 */
//=====================================================================
#ifndef SWIG
class[[deprecated(CR_DEPRECATED)]] CRSensorLinear : public CRSensorModel {
#else
class CRSensorLinear : public CRSensorModel {
#endif

  //---------------------------------------------------------------------
  // Constructor and Destructor
public:
  //! Class constructor
  CRSensorLinear(Eigen::MatrixXd i_H, Eigen::VectorXd i_x0);

  //---------------------------------------------------------------------
  // Get/Set Methods
public:
  //! Set the dynamics and input matrices
  void setObservation(Eigen::MatrixXd i_H) { this->m_H = i_H; }

  //---------------------------------------------------------------------
  // Public Methods
public:
  //! Simulate the measurement
  Eigen::VectorXd measurement(void);

  //---------------------------------------------------------------------
  // Protected Members
protected:
  //! Observation matrix
  Eigen::MatrixXd m_H;
};

//=====================================================================
// End namespace
}

#endif
