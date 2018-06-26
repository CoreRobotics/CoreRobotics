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
#include <iostream>
#include "CoreRobotics.hpp"
#include "gtest/gtest.h"

// Use the CoreRobotics namespace
using namespace CoreRobotics;

// Declare a continuous motion model - xdot = fcn(t,x,u)
Eigen::VectorXd continuousDynFcn(double t, Eigen::VectorXd x, Eigen::VectorXd u){
    return -x + u;  // motion
}

// Declare a discrete motion model - xNext = fcn(t,x,u)
Eigen::VectorXd discreteDynFcn(double t, Eigen::VectorXd x, Eigen::VectorXd u){
    x(0) = exp(-1 * 0.01) * x(0) -1 * (exp(-1 * 0.01) - 1) * 1 * u(0); // motion (dt = 0.01)
    return x;
}

//
// Test the continuous model.  Here we specify a continuous differential model,
// which is solved using the internal integration scheme of the CRMotionLinear model
//
TEST(CRMotionModel, Continuous){
    double dt = 0.01;   // sample rate (seconds)
    Eigen::VectorXd x(1);   // state vector
    Eigen::VectorXd u(1);   // input vector
    x << 1;     // initial condition
    u << 0;     // input value
    CRMotionModel model = CRMotionModel(*continuousDynFcn, CRBX_MOTION_CONTINUOUS, x, dt);
    double t = 0; // Initialize a time t
    while(t < 5) { // simulate for 5 seconds
        x = model.motion(u); // simulate the motion
        t = model.getTime(); // get the simulation time
    }
    EXPECT_NEAR(5, t, dt); // check that the simulation integrated properly
    EXPECT_NEAR(0, x(0), 0.01); // check the final state (should be within 1% of F.V. = 0)
}


//
// Test the discrete model.  Here we discretize the same system using exact discretization:
// https://en.wikipedia.org/wiki/Discretization#Discretization_of_linear_state_space_models
// this approach gives us full control over how the system is discretized
//
TEST(CRMotionModel, Discrete){
    double dt = 0.01;   // sample rate (seconds)
    Eigen::VectorXd x(1);   // state vector
    Eigen::VectorXd u(1);   // input vector
    x << 1;     // initial condition
    u << 0;     // input value
    CRMotionModel model = CRMotionModel(*discreteDynFcn, CRBX_MOTION_DISCRETE, x, dt);
    double t = 0; // Initialize a time t
    while(t < 5) { // simulate for 5 seconds
        x = model.motion(u); // simulate the motion
        t = model.getTime(); // get the simulation time
    }
    EXPECT_NEAR(5, t, dt); // check that the simulation integrated properly
    EXPECT_NEAR(0, x(0), 0.01); // check the final state (should be within 1% of F.V. = 0)
}
