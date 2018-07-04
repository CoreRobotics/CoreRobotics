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
\author  Cameron Devine

*/
//=====================================================================

#include <iostream>
#include "CoreRobotics.hpp"
#include "gtest/gtest.h"

using namespace cr;

CREulerMode convention = CR_EULER_MODE_XYZ;

Eigen::Matrix<bool, 6, 1> poseElements = (Eigen::Matrix<bool, 6, 1>() << 1, 1, 0, 0, 0, 0).finished();

struct robotData {
	HardLimits solver;
	Manipulator* robot;
	int toolIndex; };

robotData setup_solver(void) {
	// Define a robot
	Manipulator* MyRobot = new Manipulator();

	// Define several frames
	FrameEuler* F0 = new FrameEuler(0, 0, 0, 0, 0, 0, convention, CR_EULER_FREE_ANG_G);
	FrameEuler* F1 = new FrameEuler(2, 0, 0, 0, 0, 0, convention, CR_EULER_FREE_ANG_G);
	FrameEuler* F2 = new FrameEuler(-4, 0, 0, 0, 0, 0, convention, CR_EULER_FREE_ANG_G);
	FrameEuler* F3 = new FrameEuler(1, 0, 0, 0, 0, 0, convention, CR_EULER_FREE_ANG_G);

	// Define the robot links
	RigidBody* Link0 = new RigidBody(F0);
	RigidBody* Link1 = new RigidBody(F1);
	RigidBody* Link2 = new RigidBody(F2);
	RigidBody* Link3 = new RigidBody(F3);

	// Add the links to the robot
	MyRobot->addLink(Link0);
	MyRobot->addLink(Link1);
	MyRobot->addLink(Link2);
	int linkIndex = MyRobot->addLink(Link3);

	// Create a tool and add it
	FrameEuler* Tool = new FrameEuler(1, 0, 0, 0, 0, 0, convention, CR_EULER_FREE_NONE);
	int toolIndex = MyRobot->addTool(linkIndex, Tool);

	// Initialize a Hard Limits solver
	HardLimits solver = HardLimits(*MyRobot, toolIndex, convention, true);

	// Change IK solver maximum iterations
	solver.getIKSolver()->setMaxIter(20);

	// Set the pose elements
	solver.setPoseElements(poseElements);

	robotData returnStruct = {solver, MyRobot, toolIndex};
	//returnStruct.solver = solver;
	//returnStruct.robot = MyRobot;
	//returnStruct.toolIndex = toolIndex;
	return returnStruct;
}

TEST(HardLimits, NoLimits) {
	robotData data = setup_solver();
	
	HardLimits solver = data.solver;
	Manipulator* MyRobot = data.robot;
	int toolIndex = data.toolIndex;

	// Disable the nullspace solver for now
	solver.useNullSpace(false);

	// Define the initial configuration
	Eigen::VectorXd q(4), q0(4);
	q0 << 0, 0.1, 0.2, 0.3;

	// Define and set the goal location
	Eigen::VectorXd goal(2);
	goal << 0, 4.5;
	solver.setToolPose(goal);

	// Run the solver
	q = q0;
	for (int i = 0; i < 5; i++) {
		solver.setQ0(q);
		solver.solve(q);
	}

	// Get tool pose
	MyRobot->setConfiguration(q);
	Eigen::VectorXd toolPose = MyRobot->getToolPose(toolIndex, convention, poseElements);
	
	// Tests
	for (int i = 0; i < toolPose.size(); i++) {
		EXPECT_NEAR(goal(i), toolPose(i), 1e-3);
	}
}

TEST(HardLimits, Limits) {
	robotData data = setup_solver();
	
	HardLimits solver = data.solver;
	Manipulator* MyRobot = data.robot;
	int toolIndex = data.toolIndex;

	// Set jont limits
	solver.setJointLimits(2, -1, 1);

	// Disable the nullspace solver for now
	solver.useNullSpace(false);

	// Define the initial configuration
	Eigen::VectorXd q(4), q0(4);
	q0 << 0, 0.1, 0.2, 0.3;

	// Define and set the goal location
	Eigen::VectorXd goal(2);
	goal << 0, 4.5;
	solver.setToolPose(goal);

	// Run the solver
	q = q0;
	for (int i = 0; i < 5; i++) {
		solver.setQ0(q);
		solver.solve(q);
	}

	// Get tool pose
	MyRobot->setConfiguration(q);
	Eigen::VectorXd toolPose = MyRobot->getToolPose(toolIndex, convention, poseElements);
	
	// Tests
	EXPECT_GT(1, q(2));
	EXPECT_LT(-1, q(2));
	for (int i = 0; i < toolPose.size(); i++) {
		EXPECT_NEAR(goal(i), toolPose(i), 1e-3);
	}
}

TEST(HardLimits, NullSpace) {
	robotData data = setup_solver();
	
	HardLimits solver = data.solver;
	Manipulator* MyRobot = data.robot;
	int toolIndex = data.toolIndex;

	// Set jont limits
	solver.setJointLimits(2, -1, 1);

	// Define the initial configuration
	Eigen::VectorXd q(4), q0(4);
	q0 << 0, 0.1, 0.2, 0.3;

	// Define and set the goal location
	Eigen::VectorXd goal(2);
	goal << 0, 4.5;
	solver.setToolPose(goal);

	// Set the desired nullspace joint motion
	Eigen::VectorXd desiredJointMotion(4);
	desiredJointMotion << -1, 0, 0, 0;
	solver.setJointMotion(desiredJointMotion);

	// Run the solver
	q = q0;
	for (int i = 0; i < 5; i++) {
		solver.setQ0(q);
		solver.solve(q);
	}

	// Get tool pose
	MyRobot->setConfiguration(q);
	Eigen::VectorXd toolPose = MyRobot->getToolPose(toolIndex, convention, poseElements);
	
	// Tests
	EXPECT_GT(1, q(2));
	EXPECT_LT(-1, q(2));
	for (int i = 0; i < toolPose.size(); i++) {
		EXPECT_NEAR(goal(i), toolPose(i), 1e-3);
	}
}

TEST(HardLimits, BadIC) {
	robotData data = setup_solver();
	
	HardLimits solver = data.solver;
	Manipulator* MyRobot = data.robot;
	int toolIndex = data.toolIndex;

	// Set jont limits
	solver.setJointLimits(2, -1, 1);

	// Disable the nullspace solver for now
	solver.useNullSpace(false);

	// Define the initial configuration
	Eigen::VectorXd q(4), q0(4);
	q0 << 0, 0.1, M_PI, 0.3;

	// Define and set the goal location
	Eigen::VectorXd goal(2);
	goal << 0, 4.5;
	solver.setToolPose(goal);

	// Run the solver
	solver.setQ0(q0);
	Result result = solver.solve(q);
	
	// Tests
	EXPECT_EQ(CR_RESULT_BAD_IC, result);
	for (int i = 0; i < q.size(); i++) {
		EXPECT_DOUBLE_EQ(q0(i), q(i));
	}
}
