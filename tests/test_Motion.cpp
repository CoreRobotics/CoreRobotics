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
using namespace cr::model;


// derive a dynamic model class
class myMotionModel : public Motion {
    
public:
    myMotionModel(Eigen::VectorXd x,
                  Eigen::VectorXd u,
                  double dt) : Motion(x, u, dt)
    {}
    
    virtual Eigen::VectorXd callback(double t,
                                     Eigen::VectorXd x,
                                     Eigen::VectorXd u) {
        return -x + u;  // motion
    }
};



//
// Test the model with explicit time stepping.
//
TEST(MotionModel, ExplicitTime){
    
    // set up the states
    double dt = 0.01;   // sample rate (seconds)
    
    Eigen::VectorXd x(1);   // state vector
    Eigen::VectorXd u(1);   // input vector
    
    x << 1;     // initial condition
    u << 0;     // input value
    
    // initialize the model
    // std::shared_ptr<myMotionModel> mdl = std::make_shared<myMotionModel>(x, u, dt);
    myMotionModel mdl(x, u, dt);
    
    // loop the model
    double t = 0; // Initialize a time t
    while(t < 5) { // simulate for 5 seconds
        mdl.step();
        t = mdl.getTime();
        x = mdl.getState();
    }
    
    EXPECT_NEAR(5, t, dt); // check that the simulation integrated properly
    EXPECT_NEAR(0, x(0), 0.01); // check the final state (should be within 1% of F.V. = 0)
}
