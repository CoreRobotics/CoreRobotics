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
 \brief Implements a class that handles dynamic models.
 */
//---------------------------------------------------------------------
/*!
 \class CRMotionModel
 \ingroup models
 
 \brief This class implements a motion model.
 
 \details
 \section Description
 CRMotionModel implements a motion model from a supplied dynamics 
 callback function.  Specifically, CRMotionModel sets up a container
 and methods for simulating either a continuous-time set of differential
 equations as in
 
 \f$ \dot{x} = f (t,x,u) \f$,
 
 or a discrete-time set of difference equations as in
 
 \f$ x_{k+1} = f(t_k,x_k,u_k) \f$,
 
 where \f$x\f$ is the state vector, \f$u\f$ is the input vector,
 \f$t\f$ is time, and \f$k\f$ is the discrete time index.
 
 These methods are used to set up the motion model:
 - CRMotionModel::setCallbackDyn sets the dynamics callback function 
 (one of the two types specified above).
 - CRMotionModel::setType sets the type of the callback function.
 - CRMotionModel::setTimeStep sets the time step (s).
 - CRMotionModel::setState sets the underlying state vector.
 
 These methods return states of the model:
 - CRMotionModel::getState outputs the state vector.
 - CRManipulator::getTime outputs the associated time (s);
 
 These methods simulate dynamic motion:
 - CRMotionModel::simulateMotion updates the underlying state and time
 by solving the dynamic equation for the input vector and current 
 underlying state and time.
 
 \section Example
 This example demonstrates use of the CRMotionModel class.
 \code
 
 #include "CoreRobotics.hpp"
 #include #include <iostream>
 
 using namespace CoreRobotics;
 
 // declare the dynamic system callback (xdot = f(t,x,u))
 Eigen::VectorXd dynEqn(double t, Eigen::VectorXd x, Eigen::VectorXd u){
    Eigen::Matrix2d A;
    Eigen::Vector2d B;
    A << 0.0, 1.0, -10.0, -6.0;
    B << 0.0, 1.0;
    return A*x + B*u;
 }
 
 main() {
    double dt = 0.1; // time step (s)
    double t = 0; // define the time (s)
    Eigen::VectorXd x(2); // Define a state vector
    x << 0, 0; // state IC
    Eigen::VectorXd u(1); // Define an input vector
    u << 1; // make the input constant for the demonstration
 
    // initialize a continuous dynamic model
    CRMotionModel cModel = CRMotionModel(dynEqn,x,dt,CR_MODEL_CONTINUOUS);
 
    std::cout << "\nContinuous simulation:\n";
    printf("t = %3.1f, x = (%+6.4f, %+6.4f)\n",t,x(0),x(1));
 
    // Now perform a simulation of the system reponse over 2 seconds
    while(t<2){
        cModel.simulateMotion(u, false);
        cModel.getState(x);
        cModel.getTime(t);
        printf("t = %3.1f, x = (%+6.4f, %+6.4f)\n",t,x(0),x(1));
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
//! Enumerator for specifying whether the specified dynamic model is
//  either continuous or discrete.
enum CRMotionModelType {
    CR_MODEL_CONTINUOUS,
    CR_MODEL_DISCRETE
};
    
//=====================================================================
class CRMotionModel {
    
//---------------------------------------------------------------------
// Constructor and Destructor
public:
    
    //! Class constructor
    CRMotionModel(Eigen::VectorXd(dynamicFcn)(double,
                                              Eigen::VectorXd,
                                              Eigen::VectorXd),
                  Eigen::VectorXd x0,
                  double dt,
                  CRMotionModelType type);
    
//---------------------------------------------------------------------
// Get/Set Methods
public:
    
    //! Set the dynamics callback function.
    virtual void setCallbackDyn(Eigen::VectorXd(dynamicFcn)(double,
                                                            Eigen::VectorXd,
                                                            Eigen::VectorXd));
    
    //! Set the process noise model.
    // virtual void setProcessNoise(CRNoiseModel noise);
    
    //! Set the supplied dynamic function type.
    void setType(CRMotionModelType type) {this->type = type;}
    
    //! Set the time step (s)
    void setTimeStep(double dt) {this->timeStep = dt;}
    
    //! Set the system state vector (x)
    void setState(Eigen::VectorXd x) {this->state = x;}
    
    //! Get the state vector (x)
    void getState(Eigen::VectorXd &x) {x = this->state;}
    
    //! Get the model time (s)
    void getTime(double &t) {t = this->time;}
    
//---------------------------------------------------------------------
// Public Methods
public:
    
    //! Simulate the model forward a single step
    void simulateMotion(Eigen::VectorXd u, bool sampleNoise);
    
//---------------------------------------------------------------------
// Protected Members
protected:
    
    //! Dynamic model function callback type
    CRMotionModelType type;
    
    //! Sample rate (s)
    double timeStep;
    
    //! Current time (s)
    double time;
    
    //! Dynamic state of the system
    Eigen::VectorXd state;
    
    //! Pointer to the process noise model
    // CRNoiseModel *dynNoise;
    
    //! Callback to the dynamic model function \dot{x} = f(t,x,u) or
    //! x_kp1 = f(t_k,x_k,u_k) depending on what the type is set to.
    Eigen::VectorXd(*dynFcn)(double,
                             Eigen::VectorXd,
                             Eigen::VectorXd);
    
};

//=====================================================================
// End namespace
}


#endif
