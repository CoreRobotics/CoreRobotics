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
\author  Parker Owan, Tony Piaskowy

*/
//=====================================================================

#include <iostream>
#include "CoreRobotics.hpp"
#include "gtest/gtest.h"


// Use the CoreRobotics namespace
using namespace cr;


//
// constructor
//
TEST(FrameDh, Construct){
    FrameDh frame;
    Eigen::VectorXd p = frame.getPose(CR_EULER_MODE_XYZ);
    EXPECT_DOUBLE_EQ(0, p(0));
    EXPECT_DOUBLE_EQ(0, p(1));
    EXPECT_DOUBLE_EQ(0, p(2));
    EXPECT_DOUBLE_EQ(0, p(3));
    EXPECT_DOUBLE_EQ(0, p(4));
    EXPECT_DOUBLE_EQ(0, p(5));
}


//
// check the regular dh approach (based on the exact values expected
// https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters
//
TEST(FrameDh, ClassicTransformation){
    
    // init classic frame
    double r = 0.1;
    double alpha = M_PI / 3;
    double d = 0.2;
    double theta = M_PI / 6;
    FrameDh frame(r, alpha, d, theta, CR_DH_MODE_CLASSIC, CR_DH_FREE_NONE);
    
    Eigen::Matrix3d R;
    Eigen::Vector3d T;
    frame.getRotationAndTranslation(R, T);
    
    // check to the matrix layout
    EXPECT_DOUBLE_EQ(cos(theta), R(0,0));
    EXPECT_DOUBLE_EQ(-sin(theta)*cos(alpha), R(0,1));
    EXPECT_DOUBLE_EQ(sin(theta)*sin(alpha), R(0,2));
    
    EXPECT_DOUBLE_EQ(sin(theta), R(1,0));
    EXPECT_DOUBLE_EQ(cos(theta)*cos(alpha), R(1,1));
    EXPECT_DOUBLE_EQ(-cos(theta)*sin(alpha), R(1,2));
    
    EXPECT_DOUBLE_EQ(0, R(2,0));
    EXPECT_DOUBLE_EQ(sin(alpha), R(2,1));
    EXPECT_DOUBLE_EQ(cos(alpha), R(2,2));
    
    EXPECT_DOUBLE_EQ(r * cos(theta), T(0));
    EXPECT_DOUBLE_EQ(r * sin(theta), T(1));
    EXPECT_DOUBLE_EQ(d, T(2));
}


//
// check the modified dh approach (based on the exact values expected
// https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters
//
TEST(FrameDh, ModifiedTransformation){
    
    // init classic frame
    double r = 0.1;
    double alpha = M_PI / 3;
    double d = 0.2;
    double theta = M_PI / 6;
    FrameDh frame(r, alpha, d, theta, CR_DH_MODE_MODIFIED, CR_DH_FREE_NONE);
    
    Eigen::Matrix3d R;
    Eigen::Vector3d T;
    frame.getRotationAndTranslation(R, T);
    
    // check to the matrix layout
    EXPECT_DOUBLE_EQ(cos(theta), R(0,0));
    EXPECT_DOUBLE_EQ(-sin(theta), R(0,1));
    EXPECT_DOUBLE_EQ(0, R(0,2));
    
    EXPECT_DOUBLE_EQ(sin(theta)*cos(alpha), R(1,0));
    EXPECT_DOUBLE_EQ(cos(theta)*cos(alpha), R(1,1));
    EXPECT_DOUBLE_EQ(-sin(alpha), R(1,2));
    
    EXPECT_DOUBLE_EQ(sin(theta)*sin(alpha), R(2,0));
    EXPECT_DOUBLE_EQ(cos(theta)*sin(alpha), R(2,1));
    EXPECT_DOUBLE_EQ(cos(alpha), R(2,2));
    
    EXPECT_DOUBLE_EQ(r, T(0));
    EXPECT_DOUBLE_EQ(-d * sin(alpha), T(1));
    EXPECT_DOUBLE_EQ(d * cos(alpha), T(2));
}


//
// SetFreeValue/GetFreeValue
//
TEST(FrameDh, GetFreeValue){
    FrameDh frame(0, 0, 0, M_PI / 2, CR_DH_MODE_MODIFIED, CR_DH_FREE_NONE);
    EXPECT_EQ(NULL, frame.getFreeValue());
    
    frame.setFreeVariable(CR_DH_FREE_THETA);
    EXPECT_DOUBLE_EQ(M_PI / 2, frame.getFreeValue());
    
    frame.setFreeValue(0);
    EXPECT_DOUBLE_EQ(0, frame.getFreeValue());
}


//
// SetFreeVariable/GetFreeVariable
//
TEST(FrameDh, GetFreeVariable){
    FrameDh frame(0, 0, 0, M_PI / 2, CR_DH_MODE_MODIFIED, CR_DH_FREE_NONE);
    EXPECT_EQ(CR_DH_FREE_NONE, frame.getFreeVariable());
    EXPECT_FALSE(frame.isDriven());
    
    frame.setFreeVariable(CR_DH_FREE_THETA);
    EXPECT_EQ(CR_DH_FREE_THETA, frame.getFreeVariable());
    EXPECT_TRUE(frame.isDriven());
}


//
// SetMode/GetMode
//
TEST(FrameDh, GetMode){
    FrameDh frame(0, 0, 0, M_PI / 2, CR_DH_MODE_CLASSIC, CR_DH_FREE_NONE);
    EXPECT_EQ(CR_DH_MODE_CLASSIC, frame.getMode());
    
    frame.setFreeVariable(CR_DH_FREE_THETA);
    frame.setMode(CR_DH_MODE_MODIFIED);
    EXPECT_EQ(CR_DH_MODE_MODIFIED, frame.getMode());
}


//
// GetParameters
//
TEST(FrameDh, GetParameters){
    FrameDh frame(0.1, 0.2, 0.3, M_PI / 2, CR_DH_MODE_MODIFIED, CR_DH_FREE_NONE);
    double r, alpha, d, theta;
    frame.getParameters(r, alpha, d, theta);
    
    EXPECT_DOUBLE_EQ(0.1, r);
    EXPECT_DOUBLE_EQ(0.2, alpha);
    EXPECT_DOUBLE_EQ(0.3, d);
    EXPECT_DOUBLE_EQ(M_PI / 2, theta);
}