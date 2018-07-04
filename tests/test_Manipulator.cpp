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

// Setup the Manipulator robot and return the toolIndex
int setupRobot(Manipulator& MyRobot){
    FrameEuler* F0 = new FrameEuler();
    FrameEuler* F1 = new FrameEuler();
    FrameEuler* F2 = new FrameEuler();
    RigidBody* Link0 = new RigidBody();
    RigidBody* Link1 = new RigidBody();
    RigidBody* Link2 = new RigidBody();
    
    // Set info for Link 0 and add to MyRobot
    F0->setFreeVariable(CR_EULER_FREE_ANG_G);
    F0->setMode(CR_EULER_MODE_XYZ);
    F0->setPositionAndOrientation(0, 0, 0.5, 0, 0, 0);
    Link0->setFrame(F0);
    MyRobot.addLink(Link0);
    
    // Set info for Link 1 and add to MyRobot
    F1->setFreeVariable(CR_EULER_FREE_ANG_G);
    F1->setMode(CR_EULER_MODE_XYZ);
    F1->setPositionAndOrientation(1, 0, 0, 0, 0, 0);
    Link1->setFrame(F1);
    MyRobot.addLink(Link1);
    
    // Set info for Link 2 and add to MyRobot
    F2->setFreeVariable(CR_EULER_FREE_NONE);
    F2->setMode(CR_EULER_MODE_XYZ);
    F2->setPositionAndOrientation(2, 0, 0, 0, 0, 0);
    Link2->setFrame(F2);
    MyRobot.addLink(Link2);
    
    // create a tool frame and add to MyRobot
    FrameEuler* Tool = new FrameEuler();
    Tool->setMode(CR_EULER_MODE_XYZ);
    Tool->setPositionAndOrientation(0, 0, 0, 0, 0, 0);
    int toolIndex = MyRobot.addTool(2, Tool);
    return toolIndex;
}



//
// setup the manipulator
//
TEST(Manipulator, Setup){
    Manipulator MyRobot;
    FrameEuler* F0 = new FrameEuler();
    FrameEuler* F1 = new FrameEuler();
    FrameEuler* F2 = new FrameEuler();
    RigidBody* Link0 = new RigidBody();
    RigidBody* Link1 = new RigidBody();
    RigidBody* Link2 = new RigidBody();
    
    // Set info for Link 0 and add to MyRobot
    F0->setFreeVariable(CR_EULER_FREE_ANG_G);
    F0->setMode(CR_EULER_MODE_XYZ);
    F0->setPositionAndOrientation(0, 0, 0.5, 0, 0, 0);
    Link0->setFrame(F0);
    int i = MyRobot.addLink(Link0);
    EXPECT_EQ(0, i);
    
    // Set info for Link 1 and add to MyRobot
    F1->setFreeVariable(CR_EULER_FREE_ANG_G);
    F1->setMode(CR_EULER_MODE_XYZ);
    F1->setPositionAndOrientation(1, 0, 0, 0, 0, 0);
    Link1->setFrame(F1);
    i = MyRobot.addLink(Link1);
    EXPECT_EQ(1, i);
    
    // Set info for Link 2 and add to MyRobot
    F2->setFreeVariable(CR_EULER_FREE_NONE);
    F2->setMode(CR_EULER_MODE_XYZ);
    F2->setPositionAndOrientation(2, 0, 0, 0, 0, 0);
    Link2->setFrame(F2);
    i = MyRobot.addLink(Link2);
    EXPECT_EQ(2, i);
    
    // create a tool frame and add to MyRobot
    FrameEuler* Tool = new FrameEuler();
    Tool->setMode(CR_EULER_MODE_XYZ);
    Tool->setPositionAndOrientation(0, 0, 0, 0, 0, 0);
    int toolIndex = MyRobot.addTool(2, Tool);
    EXPECT_EQ(0, toolIndex);
}



//
// Set/GetConfiguration, GetNLinks, GetDOF
//
TEST(Manipulator, GetConfiguration){
    Manipulator MyRobot;
    int toolIndex = setupRobot(MyRobot);
    
    int n = MyRobot.getNumberOfLinks();
    int dof = MyRobot.getDegreesOfFreedom();
    Eigen::VectorXd q = MyRobot.getConfiguration();
    EXPECT_EQ(3, n);
    EXPECT_EQ(2, q.size());
    EXPECT_DOUBLE_EQ(2, dof);
    EXPECT_DOUBLE_EQ(0, q(0));
    EXPECT_DOUBLE_EQ(0, q(1));
    
    q << -M_PI / 4, M_PI / 4;
    MyRobot.setConfiguration(q);
    q = MyRobot.getConfiguration();
    EXPECT_DOUBLE_EQ(-M_PI / 4, q(0));
    EXPECT_DOUBLE_EQ(M_PI / 4, q(1));
}


//
// Get Forward kin
//
TEST(Manipulator, Kinematics){
    Manipulator MyRobot;
    int toolIndex = setupRobot(MyRobot);
    
    Eigen::MatrixXd fk;
    fk = MyRobot.getForwardKinematics();
    EXPECT_EQ(3, fk.rows());
    EXPECT_EQ(4, fk.cols());
    EXPECT_DOUBLE_EQ(0, fk(0, 0));
    EXPECT_DOUBLE_EQ(0, fk(1, 0));
    EXPECT_DOUBLE_EQ(0, fk(2, 0));
    EXPECT_DOUBLE_EQ(0, fk(0, 1));
    EXPECT_DOUBLE_EQ(0, fk(1, 1));
    EXPECT_DOUBLE_EQ(0.5, fk(2, 1));
    EXPECT_DOUBLE_EQ(1, fk(0, 2));
    EXPECT_DOUBLE_EQ(0, fk(1, 2));
    EXPECT_DOUBLE_EQ(0.5, fk(2, 2));
    EXPECT_DOUBLE_EQ(3, fk(0, 3));
    EXPECT_DOUBLE_EQ(0, fk(1, 3));
    EXPECT_DOUBLE_EQ(0.5, fk(2, 3));
}


//
// Get Jacobian
//
TEST(Manipulator, Jacobian){
    Manipulator MyRobot;
    int toolIndex = setupRobot(MyRobot);
    
    Eigen::MatrixXd J;
    J = MyRobot.jacobian(toolIndex, CR_EULER_MODE_XYZ);
    EXPECT_EQ(6, J.rows());
    EXPECT_EQ(2, J.cols());
    EXPECT_DOUBLE_EQ(0, J(0, 0));
    EXPECT_DOUBLE_EQ(3, J(1, 0));
    EXPECT_DOUBLE_EQ(0, J(2, 0));
    EXPECT_DOUBLE_EQ(0, J(3, 0));
    EXPECT_DOUBLE_EQ(0, J(4, 0));
    EXPECT_DOUBLE_EQ(1, J(5, 0));
    EXPECT_DOUBLE_EQ(0, J(0, 1));
    EXPECT_DOUBLE_EQ(2, J(1, 1));
    EXPECT_DOUBLE_EQ(0, J(2, 1));
    EXPECT_DOUBLE_EQ(0, J(3, 1));
    EXPECT_DOUBLE_EQ(0, J(4, 1));
    EXPECT_DOUBLE_EQ(1, J(5, 1));
    
    Eigen::Matrix<bool, 6, 1> pe;
    pe << true, true, false, false, false, true;
    J = MyRobot.jacobian(toolIndex, CR_EULER_MODE_XYZ, pe);
    EXPECT_EQ(3, J.rows());
    EXPECT_EQ(2, J.cols());
    EXPECT_DOUBLE_EQ(0, J(0, 0));
    EXPECT_DOUBLE_EQ(3, J(1, 0));
    EXPECT_DOUBLE_EQ(1, J(2, 0));
    EXPECT_DOUBLE_EQ(0, J(0, 1));
    EXPECT_DOUBLE_EQ(2, J(1, 1));
    EXPECT_DOUBLE_EQ(1, J(2, 1));
}


//
// Get Hessian
//
TEST(Manipulator, Hessian){
    Manipulator MyRobot;
    int toolIndex = setupRobot(MyRobot);
    
    // NOTE: the numerical hessian is noisy!
    Eigen::MatrixXd H;
    H = MyRobot.hessian(toolIndex, CR_EULER_MODE_XYZ);
    EXPECT_EQ(6, H.rows());
    EXPECT_EQ(2, H.cols());
    EXPECT_NEAR(-3, H(0, 0), 1e-3);
    EXPECT_DOUBLE_EQ(0, H(1, 0));
    EXPECT_DOUBLE_EQ(0, H(2, 0));
    EXPECT_DOUBLE_EQ(0, H(3, 0));
    EXPECT_DOUBLE_EQ(0, H(4, 0));
    EXPECT_DOUBLE_EQ(0, H(5, 0));
    EXPECT_NEAR(-2, H(0, 1), 1e-3);
    EXPECT_DOUBLE_EQ(0, H(1, 1));
    EXPECT_DOUBLE_EQ(0, H(2, 1));
    EXPECT_DOUBLE_EQ(0, H(3, 1));
    EXPECT_DOUBLE_EQ(0, H(4, 1));
    EXPECT_DOUBLE_EQ(0, H(5, 1));
    
    Eigen::Matrix<bool, 6, 1> pe;
    pe << true, true, false, false, false, true;
    H = MyRobot.hessian(toolIndex, CR_EULER_MODE_XYZ, pe);
    EXPECT_EQ(3, H.rows());
    EXPECT_EQ(2, H.cols());
    EXPECT_NEAR(-3, H(0, 0), 1e-3);
    EXPECT_DOUBLE_EQ(0, H(1, 0));
    EXPECT_DOUBLE_EQ(0, H(2, 0));
    EXPECT_NEAR(-2, H(0, 1), 1e-3);
    EXPECT_DOUBLE_EQ(0, H(1, 1));
    EXPECT_DOUBLE_EQ(0, H(2, 1));
}



//
// Get tool/link frames
//
TEST(Manipulator, GetFrames){
    Manipulator MyRobot;
    int toolIndex = setupRobot(MyRobot);
    
    // now get the transformation to the tool for the current configuration
    Eigen::Matrix4d T;
    Frame toolFrame;
    MyRobot.getToolFrame(toolIndex, toolFrame);
    T = toolFrame.getTransformToParent();
    EXPECT_DOUBLE_EQ(1, T(0, 0));
    EXPECT_DOUBLE_EQ(0, T(0, 1));
    EXPECT_DOUBLE_EQ(0, T(0, 2));
    EXPECT_DOUBLE_EQ(3, T(0, 3));
    EXPECT_DOUBLE_EQ(0, T(1, 0));
    EXPECT_DOUBLE_EQ(1, T(1, 1));
    EXPECT_DOUBLE_EQ(0, T(1, 2));
    EXPECT_DOUBLE_EQ(0, T(1, 3));
    EXPECT_DOUBLE_EQ(0, T(2, 0));
    EXPECT_DOUBLE_EQ(0, T(2, 1));
    EXPECT_DOUBLE_EQ(1, T(2, 2));
    EXPECT_DOUBLE_EQ(0.5, T(2, 3));
    
    Eigen::VectorXd p = MyRobot.getToolPose(toolIndex, CR_EULER_MODE_XYZ);
    EXPECT_EQ(6, p.size());
    EXPECT_DOUBLE_EQ(3, p(0));
    EXPECT_DOUBLE_EQ(0, p(1));
    EXPECT_DOUBLE_EQ(0.5, p(2));
    EXPECT_DOUBLE_EQ(0, p(3));
    EXPECT_DOUBLE_EQ(0, p(4));
    EXPECT_DOUBLE_EQ(0, p(5));
    
    // get link frame
    Frame linkFrame;
    MyRobot.getLinkFrame(0, linkFrame);
    T = linkFrame.getTransformToParent();
    EXPECT_DOUBLE_EQ(1, T(0, 0));
    EXPECT_DOUBLE_EQ(0, T(0, 1));
    EXPECT_DOUBLE_EQ(0, T(0, 2));
    EXPECT_DOUBLE_EQ(0, T(0, 3));
    EXPECT_DOUBLE_EQ(0, T(1, 0));
    EXPECT_DOUBLE_EQ(1, T(1, 1));
    EXPECT_DOUBLE_EQ(0, T(1, 2));
    EXPECT_DOUBLE_EQ(0, T(1, 3));
    EXPECT_DOUBLE_EQ(0, T(2, 0));
    EXPECT_DOUBLE_EQ(0, T(2, 1));
    EXPECT_DOUBLE_EQ(1, T(2, 2));
    EXPECT_DOUBLE_EQ(0.5, T(2, 3));
}
