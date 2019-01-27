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
\author  Parker Owan, Tony Piaskowy

*/
//=====================================================================

#include <iostream>
#include "CoreRobotics.hpp"
#include "gtest/gtest.h"


// Use the CoreRobotics namespace
using namespace cr;


//
// Test the rotation and translation on construct
//
TEST(Frame, Construct){
    physics::Frame frame;
    Eigen::Matrix3d rot;
    Eigen::Vector3d trans;
    frame.getRotationAndTranslation(rot, trans);
    EXPECT_DOUBLE_EQ(1, rot(0,0));
    EXPECT_DOUBLE_EQ(1, rot(1,1));
    EXPECT_DOUBLE_EQ(1, rot(2,2));
    EXPECT_DOUBLE_EQ(0, rot(0,1));
    EXPECT_DOUBLE_EQ(0, rot(0,2));
    EXPECT_DOUBLE_EQ(0, rot(1,0));
    EXPECT_DOUBLE_EQ(0, rot(1,2));
    EXPECT_DOUBLE_EQ(0, rot(2,0));
    EXPECT_DOUBLE_EQ(0, rot(2,1));
    EXPECT_DOUBLE_EQ(0, trans(0));
    EXPECT_DOUBLE_EQ(0, trans(1));
    EXPECT_DOUBLE_EQ(0, trans(2));
}


//
// Test the rotation and translation on construct
//
TEST(Frame, GetFreeValue){
    physics::Frame frame;
    double value = frame.getFreeValue();
    EXPECT_EQ(NULL, value);
}


//
// Get the vector of orientation angles
//
TEST(Frame, GetOrientation){
    physics::Frame frame;
    Eigen::Matrix3d rot;
    rot << 0, 1, 0, 0, 0, 1, 1, 0, 0;
    Eigen::Vector3d trans;
    trans << 0.1, 0.2, 0.3;
    frame.setRotationAndTranslation(rot, trans);
    
    Eigen::Vector3d o;
    o = frame.getOrientation(physics::CR_EULER_MODE_ZYX);
    EXPECT_DOUBLE_EQ(-M_PI / 2, o(0));
    EXPECT_DOUBLE_EQ(-M_PI / 2, o(1));
    EXPECT_DOUBLE_EQ(0, o(2));
}


//
// Get the pose
//
TEST(Frame, GetPose){
    physics::Frame frame;
    Eigen::Matrix3d rot;
    rot << 0, 1, 0, 0, 0, 1, 1, 0, 0;
    Eigen::Vector3d trans;
    trans << 0.1, 0.2, 0.3;
    frame.setRotationAndTranslation(rot, trans);
    
    Eigen::VectorXd p;
    p = frame.getPose(physics::CR_EULER_MODE_ZYX);
    EXPECT_EQ(6, p.size());
    EXPECT_DOUBLE_EQ(0.1, p(0));
    EXPECT_DOUBLE_EQ(0.2, p(1));
    EXPECT_DOUBLE_EQ(0.3, p(2));
    EXPECT_DOUBLE_EQ(-M_PI / 2, p(3));
    EXPECT_DOUBLE_EQ(-M_PI / 2, p(4));
    EXPECT_DOUBLE_EQ(0, p(5));
    
    // now try a reduced pose
    Eigen::Matrix<bool, 6, 1> pe;
    pe << true, true, true, true, false, false;
    p = frame.getPose(physics::CR_EULER_MODE_XYZ, pe);
    EXPECT_EQ(4, p.size());
    EXPECT_DOUBLE_EQ(0.1, p(0));
    EXPECT_DOUBLE_EQ(0.2, p(1));
    EXPECT_DOUBLE_EQ(0.3, p(2));
    EXPECT_DOUBLE_EQ(-M_PI / 2, p(3));
}


//
// Get the transform to child
//
TEST(Frame, GetTransformToChild){
    physics::Frame frame;
    Eigen::Matrix3d rot;
    rot << 0, 1, 0, 0, 0, 1, 1, 0, 0;
    Eigen::Vector3d trans;
    trans << 0.1, 0.2, 0.3;
    frame.setRotationAndTranslation(rot, trans);
    
    Eigen::Matrix4d T;
    
    // This is T = [R' | -R't ; 0 | 1]
    T = frame.getTransformToChild();
    EXPECT_DOUBLE_EQ(0, T(0,0));
    EXPECT_DOUBLE_EQ(0, T(0,1));
    EXPECT_DOUBLE_EQ(1, T(0,2));
    EXPECT_DOUBLE_EQ(-0.3, T(0,3));
    EXPECT_DOUBLE_EQ(1, T(1,0));
    EXPECT_DOUBLE_EQ(0, T(1,1));
    EXPECT_DOUBLE_EQ(0, T(1,2));
    EXPECT_DOUBLE_EQ(-0.1, T(1,3));
    EXPECT_DOUBLE_EQ(0, T(2,0));
    EXPECT_DOUBLE_EQ(1, T(2,1));
    EXPECT_DOUBLE_EQ(0, T(2,2));
    EXPECT_DOUBLE_EQ(-0.2, T(2,3));
    EXPECT_DOUBLE_EQ(0, T(3,0));
    EXPECT_DOUBLE_EQ(0, T(3,1));
    EXPECT_DOUBLE_EQ(0, T(3,2));
    EXPECT_DOUBLE_EQ(1, T(3,3));
}



//
// Get the transform to parent
//
TEST(Frame, GetTransformToParent){
    physics::Frame frame;
    Eigen::Matrix3d rot;
    rot << 0, 1, 0, 0, 0, 1, 1, 0, 0;
    Eigen::Vector3d trans;
    trans << 0.1, 0.2, 0.3;
    frame.setRotationAndTranslation(rot, trans);
    
    Eigen::Matrix4d T;
    T = frame.getTransformToParent();
    EXPECT_DOUBLE_EQ(0, T(0,0));
    EXPECT_DOUBLE_EQ(1, T(0,1));
    EXPECT_DOUBLE_EQ(0, T(0,2));
    EXPECT_DOUBLE_EQ(0.1, T(0,3));
    EXPECT_DOUBLE_EQ(0, T(1,0));
    EXPECT_DOUBLE_EQ(0, T(1,1));
    EXPECT_DOUBLE_EQ(1, T(1,2));
    EXPECT_DOUBLE_EQ(0.2, T(1,3));
    EXPECT_DOUBLE_EQ(1, T(2,0));
    EXPECT_DOUBLE_EQ(0, T(2,1));
    EXPECT_DOUBLE_EQ(0, T(2,2));
    EXPECT_DOUBLE_EQ(0.3, T(2,3));
    EXPECT_DOUBLE_EQ(0, T(3,0));
    EXPECT_DOUBLE_EQ(0, T(3,1));
    EXPECT_DOUBLE_EQ(0, T(3,2));
    EXPECT_DOUBLE_EQ(1, T(3,3));
}



//
// Get if the frame is driven
//
TEST(Frame, IsDriven){
    physics::Frame frame;
    EXPECT_FALSE(frame.isDriven());
}




//
// transform point to child
//
TEST(Frame, TransformToChild){
    physics::Frame frame;
    Eigen::Matrix3d rot;
    rot << 0, 1, 0, 0, 0, 1, 1, 0, 0;
    Eigen::Vector3d trans;
    trans << 0.1, 0.2, 0.3;
    frame.setRotationAndTranslation(rot, trans);
    
    Eigen::Vector3d p;
    p << 0, 0, 0;
    
    // This is T = [R' | -R't ; 0 | 1]
    Eigen::VectorXd p2 = frame.transformToChild(p);
    EXPECT_DOUBLE_EQ(-0.3, p2(0));
    EXPECT_DOUBLE_EQ(-0.1, p2(1));
    EXPECT_DOUBLE_EQ(-0.2, p2(2));
}



//
// transform point to parent
//
TEST(Frame, TransformToParent){
    physics::Frame frame;
    Eigen::Matrix3d rot;
    rot << 0, 1, 0, 0, 0, 1, 1, 0, 0;
    Eigen::Vector3d trans;
    trans << 0.1, 0.2, 0.3;
    frame.setRotationAndTranslation(rot, trans);
    
    Eigen::Vector3d p;
    p << 0, 0, 0;
    
    Eigen::VectorXd p2 = frame.transformToParent(p);
    EXPECT_DOUBLE_EQ(0.1, p2(0));
    EXPECT_DOUBLE_EQ(0.2, p2(1));
    EXPECT_DOUBLE_EQ(0.3, p2(2));
}
