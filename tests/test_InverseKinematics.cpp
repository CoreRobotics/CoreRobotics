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
using namespace cr;
using namespace cr::core;


// Setup the Manipulator robot and return the toolIndex
int setup3dof(world::Manipulator& MyRobot){
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
    F2->setFreeVariable(CR_EULER_FREE_ANG_G);
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
// test the set/get methods
//
TEST(InverseKinematics, SetGet){
    world::Manipulator MyRobot;
    int toolIndex = setup3dof(MyRobot);
    InverseKinematics ikSolver = InverseKinematics(MyRobot,
                                                       toolIndex,
                                                       CR_EULER_MODE_XYZ);
    
    ikSolver.setRobot(MyRobot);
    ikSolver.setMaxIter(100);
    ikSolver.setStepSize(0.1);
    ikSolver.setEulerMode(CR_EULER_MODE_ZYX);
    ikSolver.setTolerance(1e-4);
    ikSolver.setToolIndex(toolIndex);
    ikSolver.setDampingFactor(1.0);
    
    
    EXPECT_EQ(100, ikSolver.getMaxIter());
    EXPECT_DOUBLE_EQ(0.1, ikSolver.getStepSize());
    EXPECT_EQ(CR_EULER_MODE_ZYX, ikSolver.getEulerMode());
    EXPECT_DOUBLE_EQ(1e-4, ikSolver.getTolerance());
    EXPECT_EQ(toolIndex, ikSolver.getToolIndex());
    EXPECT_DOUBLE_EQ(1.0, ikSolver.getDampingFactor());
}


//
// test the set/get methods
//
TEST(InverseKinematics, Solver){
    world::Manipulator* MyRobot = new world::Manipulator();
    int toolIndex = setup3dof(*MyRobot);
    InverseKinematics ikSolver = InverseKinematics(*MyRobot,
                                                       toolIndex,
                                                       CR_EULER_MODE_XYZ);
    
    // variables for testing the results
    Eigen::VectorXd q0(3);          // initial configuration
    Eigen::VectorXd qSolved(3);     // configuration that solves p = fk(qSolved)
    Eigen::Matrix<double, 6, 1> p;  // tool set point pose
    Eigen::MatrixXd fk;             // forward kinematics (for testing the result)
    Result result;
    
    // Solve a non-singular solution
    p << 2.5, 0, 0, 0, 0, 0;
    q0 << 0.1, -0.2, 0.0;
    MyRobot->setConfiguration(q0);
    result = ikSolver.solve(p, q0, qSolved);
    EXPECT_EQ(CR_RESULT_SUCCESS, result);
    
    // Now check the solution results
    ikSolver.setDampingFactor(0);
    ikSolver.setMaxIter(100);
    ikSolver.setTolerance(1e-4);
    MyRobot->setConfiguration(q0);
    result = ikSolver.solve(p, q0, qSolved);
    MyRobot->setConfiguration(qSolved);
    fk = MyRobot->getForwardKinematics();
    
    EXPECT_EQ(3, fk.rows());
    EXPECT_EQ(4, fk.cols());
    EXPECT_EQ(CR_RESULT_SUCCESS, result);
    EXPECT_NEAR(2.5, fk(0,3), 1e-4);
    EXPECT_NEAR(0, fk(1,3), 1e-4);
    EXPECT_NEAR(0.5, fk(2,3), 1e-4);
}

