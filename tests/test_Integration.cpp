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
#include <iostream>
#include "CoreRobotics.hpp"
#include "gtest/gtest.h"

// Use the CoreRobotics namespace
using namespace cr::math;

// Declare a continuous dynamical system - xdot = fcn(t,x,u)
// The exact solution of this system is y = exp(-t) for u = 0
Eigen::VectorXd myDynamicalSystem(double t, Eigen::VectorXd x, Eigen::VectorXd u){
    return -x + u;  // motion
}

//
// Test the runge kutta method
//
TEST(Integration, RungeKutta){
    double t = 0;
    double dt = 0.01;   // sample rate (seconds)
    Eigen::VectorXd x(1);   // state vector
    Eigen::VectorXd u(1);   // input vector
    x << 1;   // initial condition
    u << 0;     // input value
    
    // perform constant sample rate integration
    while(t <= 5){
        x = Integration::rungeKuttaStep(*myDynamicalSystem, t, x, u, dt);
        t += dt;
    }
    
    // check that we approach the final values
    EXPECT_NEAR(5, t, dt); // check that the simulation integrated properly
    EXPECT_NEAR(exp(-t), x(0), 1e-6); // check the final state to exact solution
}

//
// Test the forward euler method
//
TEST(Integration, ForwardEuler){
    double t = 0;
    double dt = 0.01;   // sample rate (seconds)
    Eigen::VectorXd x(1);   // state vector
    Eigen::VectorXd u(1);   // input vector
    x << 1;   // initial condition
    u << 0;     // input value
    
    // perform constant sample rate integration
    while(t <= 5){
        x = Integration::forwardEulerStep(*myDynamicalSystem, t, x, u, dt);
        t += dt;
    }
    
    // check that we approach the final values
    EXPECT_NEAR(5, t, dt); // check that the simulation integrated properly
    EXPECT_NEAR(exp(-t), x(0), 1e-3); // check the final state to exact solution
}


