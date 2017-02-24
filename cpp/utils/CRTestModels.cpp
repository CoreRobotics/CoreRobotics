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

#include <iostream>
#include "CoreRobotics.hpp"


// Use the CoreRobotics namespace
using namespace CoreRobotics;


// declare the callback functions
Eigen::VectorXd cdynamics(double t, Eigen::VectorXd x, Eigen::VectorXd u);
Eigen::VectorXd ddynamics(double t, Eigen::VectorXd x, Eigen::VectorXd u);
Eigen::VectorXd observation(double t, Eigen::VectorXd x, Eigen::VectorXd u);

// define a time step
double dt = 0.1;
 
// main
void CRTestModels(void){
    
    std::cout << "**********************\n";
    std::cout << "Running the CRTestModels\n";
    

    // define the time (s)
    double t = 0;
    
    // Define a state vector
    Eigen::VectorXd x(2);
    x << 0, 0; // IC
    
    // Define an input vector
    Eigen::VectorXd u(1);
    u << 1;
    
    
    // initialize a discrete dynamic model
    CRMotionModel dModel = CRMotionModel(x,dt,CR_MODEL_DISCRETE);
    dModel.setDynamics(ddynamics);
    
    std::cout << "\nDiscrete simulation:\n";
    printf("t = %3.1f, x = (%+6.4f, %+6.4f)\n",t,x(0),x(1));
    
    while(t<2){
        dModel.simulateMotion(u, false);
        dModel.getState(x);
        dModel.getTime(t);
        printf("t = %3.1f, x = (%+6.4f, %+6.4f)\n",t,x(0),x(1));
    }
    
    // Reset IC
    t = 0;
    x << 0, 0;
    
    // initialize a continuous dynamic model
    CRMotionModel cModel = CRMotionModel(x,dt,CR_MODEL_CONTINUOUS);
    cModel.setDynamics(cdynamics);
    
    std::cout << "\nContinuous simulation:\n";
    printf("t = %3.1f, x = (%+6.4f, %+6.4f)\n",t,x(0),x(1));
    
    while(t<2){
        cModel.simulateMotion(u, false);
        cModel.getState(x);
        cModel.getTime(t);
        printf("t = %3.1f, x = (%+6.4f, %+6.4f)\n",t,x(0),x(1));
    }
    
    
}



// Callback for the continuous motion model
Eigen::VectorXd cdynamics(double t, Eigen::VectorXd x, Eigen::VectorXd u){
    
    Eigen::Matrix2d A;
    Eigen::Vector2d B;
    
    A << 0.0, 1.0, -10.0, -6.0;
    B << 0.0, 1.0;
    
    return A*x + B*u;
}


// Callback for the doscrete motion model
Eigen::VectorXd ddynamics(double t, Eigen::VectorXd x, Eigen::VectorXd u){
    
    Eigen::Matrix2d A;
    Eigen::Vector2d B;
    Eigen::Matrix2d I = Eigen::Matrix2d::Identity();
    
    A << 0.0, 1.0, -10.0, -6.0;
    B << 0.0, 1.0;
    
    return (I+dt*A)*x + (dt*B)*u;
}


// Callback for the sensor model
Eigen::VectorXd observation(double t, Eigen::VectorXd x, Eigen::VectorXd u){
    
    Eigen::Matrix<double, 1, 2> C;
    Eigen::Matrix<double, 1, 1> D;
    
    C << 1, 0;
    D << 0;
    
    return C*x + D*u;
}





