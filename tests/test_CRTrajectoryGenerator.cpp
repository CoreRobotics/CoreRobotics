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

//
// Test solve method
//
TEST(CRTrajectoryGenerator, Solve){
    CRTrajectoryGenerator trajGen;
    CRWaypoint wp;
    
    // Initial conditions
    Eigen::Vector2d x0, v0, a0;
    x0 << 0, 0;
    v0 << 0, -2;
    a0 << 0, 0;
    
    // Final conditions
    double tf = 1.2;
    Eigen::Vector2d xf, vf, af;
    xf << -0.4, 0.5;
    vf << 1.2, 1;
    af << 0, 0;
    
    // Check the solve function
    CRResult result = trajGen.solve(x0, v0, a0, xf, vf, af, tf);
    EXPECT_EQ(result, CR_RESULT_SUCCESS);
}



//
// Test step methods
//
TEST(CRTrajectoryGenerator, Step){
    CRTrajectoryGenerator trajGen;
    CRWaypoint wp;
    
    // Initial conditions
    Eigen::Vector2d x0, v0, a0;
    x0 << 0, 0;
    v0 << 0, -2;
    a0 << 0, 0;
    
    // Final conditions
    double tf = 1.2;
    Eigen::Vector2d xf, vf, af;
    xf << -0.4, 0.5;
    vf << 1.2, 1;
    af << 0, 0;
    
    // add a timer for running the internal clock
    CRClock timer;
    
    // Check the internal timer
    trajGen.solve(x0, v0, a0, xf, vf, af, tf);
    timer.startTimer();
    timer.sleep(tf);
    wp = trajGen.step();
    EXPECT_NEAR(tf, wp.time, 0.01);
    EXPECT_NEAR(xf(0), wp.position(0), 0.001);
    EXPECT_NEAR(xf(1), wp.position(1), 0.001);
    EXPECT_NEAR(vf(0), wp.velocity(0), 0.001);
    EXPECT_NEAR(vf(1), wp.velocity(1), 0.001);
    EXPECT_NEAR(af(0), wp.acceleration(0), 0.001);
    EXPECT_NEAR(af(1), wp.acceleration(1), 0.001);
    
    // Check the value before the initial time (i.e.: t < 0)
    wp = trajGen.step(-1);
    EXPECT_DOUBLE_EQ(0, wp.time);
    EXPECT_DOUBLE_EQ(x0(0), wp.position(0));
    EXPECT_DOUBLE_EQ(x0(1), wp.position(1));
    EXPECT_DOUBLE_EQ(v0(0), wp.velocity(0));
    EXPECT_DOUBLE_EQ(v0(1), wp.velocity(1));
    EXPECT_DOUBLE_EQ(a0(0), wp.acceleration(0));
    EXPECT_DOUBLE_EQ(a0(1), wp.acceleration(1));
    
    // Check the value at the initial time (i.e.: t = 0)
    wp = trajGen.step(0);
    EXPECT_DOUBLE_EQ(0, wp.time);
    EXPECT_DOUBLE_EQ(x0(0), wp.position(0));
    EXPECT_DOUBLE_EQ(x0(1), wp.position(1));
    EXPECT_DOUBLE_EQ(v0(0), wp.velocity(0));
    EXPECT_DOUBLE_EQ(v0(1), wp.velocity(1));
    EXPECT_DOUBLE_EQ(a0(0), wp.acceleration(0));
    EXPECT_DOUBLE_EQ(a0(1), wp.acceleration(1));
    
    // Check the value at the end time (i.e.: t = tf)
    wp = trajGen.step(tf);
    EXPECT_DOUBLE_EQ(tf, wp.time);
    EXPECT_NEAR(xf(0), wp.position(0), 1e-12);
    EXPECT_NEAR(xf(1), wp.position(1), 1e-12);
    EXPECT_NEAR(vf(0), wp.velocity(0), 1e-12);
    EXPECT_NEAR(vf(1), wp.velocity(1), 1e-12);
    EXPECT_NEAR(af(0), wp.acceleration(0), 1e-12);
    EXPECT_NEAR(af(1), wp.acceleration(1), 1e-12);
    
    // Check the value well past the end time (i.e.: t > tf)
    wp = trajGen.step(tf + 10);
    EXPECT_DOUBLE_EQ(tf, wp.time);
    EXPECT_NEAR(xf(0), wp.position(0), 1e-12);
    EXPECT_NEAR(xf(1), wp.position(1), 1e-12);
    EXPECT_NEAR(vf(0), wp.velocity(0), 1e-12);
    EXPECT_NEAR(vf(1), wp.velocity(1), 1e-12);
    EXPECT_NEAR(af(0), wp.acceleration(0), 1e-12);
    EXPECT_NEAR(af(1), wp.acceleration(1), 1e-12);
    
}
