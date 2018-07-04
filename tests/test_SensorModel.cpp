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

// Declare a deterministic model - fcn(x,zHat)
Eigen::VectorXd detPredFcn(Eigen::VectorXd x){
    return x;  // observation (z = x)
}

//
// Test the model prediction
//
TEST(SensorModel, Predict){
    Eigen::VectorXd x(1);   // state vector
    Eigen::VectorXd z(1);   // observation
    x << 1;     // initial condition
    SensorModel sensor = SensorModel(*detPredFcn, x);
    z = sensor.measurement();
    EXPECT_DOUBLE_EQ(1, z(0));
    
    x = sensor.getState();
    EXPECT_DOUBLE_EQ(1, x(0));
    
    x << 10;
    sensor.setState(x);
    x = sensor.getState();
    EXPECT_DOUBLE_EQ(10, x(0));
}

