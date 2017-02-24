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

#ifndef CRSensorModel_hpp
#define CRSensorModel_hpp

//=====================================================================
// Includes
#include "Eigen/Dense"

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
 
 // declare the observation system callback (z = h(x,u))
    Eigen::VectorXd obsEqn(Eigen::VectorXd x, Eigen::VectorXd u){
    Eigen::Matrix<double, 1, 2> C;
    C << 1, 0;
    return C*x;
 }
 
 main() {
     double dt = 0.1; // time step (s)
     double t = 0;    // define the time (s)
     Eigen::VectorXd x(2); // Define a state vector
     x << 0, 0; // state IC
     Eigen::VectorXd u(1); // Define an input vector
     u << 1; // make the input constant for the demonstration
     Eigen::VectorXd z(1); // Define a measurement vector
     z << 0; // init value in memory
     
     // initialize a sensor model
     CRSensorModel sensor = CRSensorModel(x);
     sensor.setCallbackObsv(obsEqn);
     
     std::cout << "\nSensor simulation:\n";
     printf("t = %3.1f, x = (%+6.4f, %+6.4f), z= (%6.4f)\n",t,x(0),x(1),z(0));
     
     // Now perform a simulation of the system reponse over 2 seconds
     while(t<2){
     
         // simulate a simple dynamic system
         double x0 = x(0)+dt*x(1);
         double x1 = -dt*10*x(0)+(1-dt*6)*x(1)+dt*u(0);
         x << x0, x1;
         
         sensor.setState(x);
         sensor.simulateMeasurement(u, false);
         sensor.getMeasurement(z);
         printf("t = %3.1f, x = (%+6.4f, %+6.4f), z= (%6.4f)\n",t,x(0),x(1),z(0));
         
         t = t+dt;
     }
 }
 
 \endcode
 
 \section References
 [1] J. Crassidis and J. Junkins, "Optimal Estimation of Dynamic Systems",
 Ed. 2, CRC Press, 2012. \n\n
 
 [2] S. Thrun, W. Burgard, and D. Fox, "Probabilistic Robotics", MIT Press,
 2006. \n\n
 */
//=====================================================================
class CRSensorModel {
    
//---------------------------------------------------------------------
// Constructor and Destructor
public:
    
    //! Class constructor
    CRSensorModel(Eigen::VectorXd x0);
    
//---------------------------------------------------------------------
// Get/Set Methods
public:
    
    //! Set the observation callback function.
    virtual void setCallbackObsv(Eigen::VectorXd(observationFcn)(Eigen::VectorXd,
                                                                 Eigen::VectorXd));
    
    //! Set the process noise model.
    // virtual void setMeasurementNoise(CRNoiseModel noise);
    
    //! Set the system state vector (x)
    void setState(Eigen::VectorXd x) {this->state = x;}
    
    //! Get the state vector (x)
    void getState(Eigen::VectorXd &x) {x = this->state;}
    
    //! Get the measurement vector (z)
    void getMeasurement(Eigen::VectorXd &z) {z = this->measurement;}
    
//---------------------------------------------------------------------
// Public Methods
public:
    
    //! Simulate the measurement
    void simulateMeasurement(Eigen::VectorXd u, bool sampleNoise);
    
//---------------------------------------------------------------------
// Protected Members
protected:
    
    //! Underlying state of the system
    Eigen::VectorXd state;
    
    //! Measurement
    Eigen::VectorXd measurement;
    
    //! Pointer to the measurement noise model
    // CRNoiseModel *obsNoise;
    
    //! Callback to the measurement model function z = h(x,u)
    Eigen::VectorXd(*obsFcn)(Eigen::VectorXd,
                             Eigen::VectorXd);
    
};

//=====================================================================
// End namespace
}


#endif
