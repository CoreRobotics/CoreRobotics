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

#ifndef CRNoiseModel_hpp
#define CRNoiseModel_hpp

//=====================================================================
// Includes
#include "Eigen/Dense"
#include <random>

//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics {
    
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
 \section Description
 CRSensorModel implements a seensor model from a supplied observation
 callback function.  Specifically, CRSensorModel sets up a container
 for the model
 
 \f$ z = h(x,u) \f$,
 
 where \f$x\f$ is the state vector, \f$u\f$ is the input vector,
 and \f$z\f$ is the sensor measurement vector.
 
 These methods are used to set up the sensor model:
 - CRSensorModel::setCallbackObsv sets the observation callback function.
 - CRSensorModel::setState sets the underlying state vector.
 
 These methods return states of the model:
 - CRSensorModel::getState outputs the state vector.
 - CRSensorModel::getMeasurement outputs the measurement vector.
 
 These methods simulate sensor measurements:
 - CRSensorModel::simulateMeasurement computes the measurement vector (z)
 from the underlying state (x) for a given input (u).
 
 \section Example
 This example demonstrates use of the CRSensorModel class.
 \code
 
 #include "CoreRobotics.hpp"
 #include #include <iostream>
 
 using namespace CoreRobotics;
 
 main() {
 
 }
 
 \endcode
 
 \section References
 [1] J. Crassidis and J. Junkins, "Optimal Estimation of Dynamic Systems",
 Ed. 2, CRC Press, 2012. \n\n
 
 [2] S. Thrun, W. Burgard, and D. Fox, "Probabilistic Robotics", MIT Press,
 2006. \n\n
 */
//=====================================================================
//! Enumerator for specifying whether the specified dynamic model is
//  either continuous or discrete.
enum CRNoiseType {
    CR_NOISE_CONTINUOUS,
    CR_NOISE_DISCRETE
};
    
//=====================================================================
// ! Paramter structure declaration
struct icdParam{
    CRNoiseType type;
    double(*icdFunction)(double);
};
    
//=====================================================================
class CRNoiseModel {
    
//---------------------------------------------------------------------
// Constructor and Destructor
public:
    
    //! Class constructor
    CRNoiseModel();
    CRNoiseModel(unsigned seed);
    
//---------------------------------------------------------------------
// Get/Set Methods
public:
    
    //! Set the parameters that describe the distribution
    virtual void setParameters(CRNoiseType type,
                               double(*icdFunction)(double));
    
//---------------------------------------------------------------------
// Public Methods
public:
    
    //! Sample a noise vector from the density
    virtual void sample(Eigen::Matrix<double,1,1> &x);
    
//---------------------------------------------------------------------
// Protected Members
protected:
    
    //! Noise model type
    icdParam parameters;
    
    //! Seed value
    unsigned seed;
    
    //! Random number generator
    std::default_random_engine generator;
    
};

//=====================================================================
// End namespace
}


#endif
