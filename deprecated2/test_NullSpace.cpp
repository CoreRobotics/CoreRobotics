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

// Use the CoreRobotics namespace
using namespace cr;
using namespace cr::core;

struct testData {
	Eigen::VectorXd initialToolPose;
	Eigen::VectorXd finalToolPose;
	Eigen::VectorXd initialJointAngles;
	Eigen::VectorXd finalJointAngles;
	Result result; };

testData test_NullSpace(Eigen::VectorXd initJoints,
                          Eigen::VectorXd jointMotion,
                          bool usePoseElements = false,
                          Eigen::Matrix<bool, 6, 1> poseElements = (Eigen::Matrix<bool, 6, 1>() << true, true, true, true, true, true).finished()) {
	testData returnStruct;

	world::Manipulator* MyRobot = new world::Manipulator();

	// Initialize translation frames
	FrameEuler* F0 = new FrameEuler();
	FrameEuler* F1 = new FrameEuler();
	FrameEuler* F2 = new FrameEuler();
	FrameEuler* F3 = new FrameEuler();
	FrameEuler* F4 = new FrameEuler();
	FrameEuler* F5 = new FrameEuler();
	FrameEuler* F6 = new FrameEuler();

	// Initialize links
	RigidBody* Link0 = new RigidBody();
	RigidBody* Link1 = new RigidBody();
	RigidBody* Link2 = new RigidBody();
	RigidBody* Link3 = new RigidBody();
	RigidBody* Link4 = new RigidBody();
	RigidBody* Link5 = new RigidBody();
	RigidBody* Link6 = new RigidBody();


	// Set translation frames and DH Modified
	F0->setMode(CR_EULER_MODE_XYZ);
	F1->setMode(CR_EULER_MODE_XYZ);
	F2->setMode(CR_EULER_MODE_XYZ);
	F3->setMode(CR_EULER_MODE_XYZ);
	F4->setMode(CR_EULER_MODE_XYZ);
	F5->setMode(CR_EULER_MODE_XYZ);
	F6->setMode(CR_EULER_MODE_XYZ);

	// Set tralation frame free variables
	F0->setFreeVariable(CR_EULER_FREE_ANG_G);
	F1->setFreeVariable(CR_EULER_FREE_ANG_G);
	F2->setFreeVariable(CR_EULER_FREE_ANG_A);
	F3->setFreeVariable(CR_EULER_FREE_ANG_G);
	F4->setFreeVariable(CR_EULER_FREE_ANG_B);
	F5->setFreeVariable(CR_EULER_FREE_ANG_A);
	F6->setFreeVariable(CR_EULER_FREE_NONE);


	// lengths
	double d1	= 129.275;
	double d2	= 62.68;
	double d3	= 70.0;
	double d4	= 4.135;
	double d5	= 201.07;
	double d6	= 10.57;
	double d7	= 163.24;
	double d8	= 31.25;
	double d9	= 75.47;
	double d10	= 39.36;

	// Set Euler Angles and Position Offsets
	//							  dx		dy		dz		tx				ty		tz
	F0->setPositionAndOrientation(0.0,		0.0,	d1,		0.0,			0.0,	0.0);
    F1->setPositionAndOrientation(0.0,		0.0,	0.0,	M_PI/2.0,		0.0,	0.0);
	F2->setPositionAndOrientation(d3,		d4,		d2,		0.0,			0.0,	0.0);
	F3->setPositionAndOrientation(d5,		0.0,	-d6,	0.0,			0.0,	0.0);
	F4->setPositionAndOrientation(d7,		0.0,	-d8,	-M_PI/2.0,		0.0,	0.0);
	F5->setPositionAndOrientation(d9,		d10,	0.0,	0.0,			0.0,	0.0);
	F6->setPositionAndOrientation(0.0,		0.0,	0.0,	0.0,			0.0,	0.0);

	// Add frames to links
	Link0->setFrame(F0);
	Link1->setFrame(F1);
	Link2->setFrame(F2);
	Link3->setFrame(F3);
	Link4->setFrame(F4);
	Link5->setFrame(F5);
	Link6->setFrame(F6);

	// Add links to robot
	int linkIndex0 = MyRobot->addLink(Link0);
	int linkIndex1 = MyRobot->addLink(Link1);
	int linkIndex2 = MyRobot->addLink(Link2);
	int linkIndex3 = MyRobot->addLink(Link3);
	int linkIndex4 = MyRobot->addLink(Link4);
	int linkIndex5 = MyRobot->addLink(Link5);
	int linkIndex6 = MyRobot->addLink(Link6);

	// Create a tool frame and add to MyRobot
	FrameEuler* Tool = new FrameEuler();
	Tool->setFreeVariable(CR_EULER_FREE_NONE);
	Tool->setMode(CR_EULER_MODE_XYZ);
	Tool->setPositionAndOrientation(90.21, 0.0, -107.43, -M_PI / 8.0, 0.0, 0.0);
	int toolIndex = MyRobot->addTool(linkIndex6, Tool);

	// Initialize the solver
	NullSpace nullSpaceSolver = NullSpace(*MyRobot, toolIndex, CR_EULER_MODE_XYZ);

	// Set the robot orientation
	MyRobot->setConfiguration(initJoints);
	returnStruct.initialJointAngles = initJoints;

	// Compute the initial tool pose
	returnStruct.initialToolPose = MyRobot->getToolPose(toolIndex, CR_EULER_MODE_XYZ);

	// Find a nullspace movement
	Eigen::VectorXd nullJointMotion(6);
	if (usePoseElements) {
		returnStruct.result = nullSpaceSolver.solve(jointMotion, initJoints, poseElements, nullJointMotion);
	} else {
		returnStruct.result = nullSpaceSolver.solve(jointMotion, initJoints, nullJointMotion);
	}

	// Compute the final tool pose
	MyRobot->setConfiguration(initJoints + nullJointMotion);
	returnStruct.finalJointAngles = initJoints + nullJointMotion;
	returnStruct.finalToolPose = MyRobot->getToolPose(toolIndex, CR_EULER_MODE_XYZ);

	return returnStruct;
}

TEST(NullSpace, Full) {
	Eigen::VectorXd initJoints(6);
	initJoints << 0.0, 2.0, 0.0, -2, 0.0, M_PI / 8.0;
	Eigen::VectorXd jointMotion(6);
	jointMotion << 1, 0, 0, 0, 0, 0;
	testData data = test_NullSpace(initJoints, jointMotion);
	EXPECT_EQ(CR_RESULT_SUCCESS, data.result);
	for (int i = 0; i < data.initialToolPose.size(); i++) {
		EXPECT_DOUBLE_EQ(data.initialToolPose(i), data.finalToolPose(i));
	}
	for (int i = 0; i < data.initialJointAngles.size(); i++) {
		EXPECT_DOUBLE_EQ(data.initialJointAngles(i), data.finalJointAngles(i));
	}
}

TEST(NullSpace, PoseElements) {
	Eigen::VectorXd initJoints(6);
	initJoints << 0.0, 2.0, 0.0, -2, 0.0, M_PI / 8.0;
	Eigen::VectorXd jointMotion(6);
	jointMotion << 1, 0, 0, 0, 0, 0;
	Eigen::Matrix<bool, 6, 1> poseElements;
	poseElements << 1, 1, 1, 1, 1, 0;
	testData data = test_NullSpace(initJoints, jointMotion, true, poseElements);
	EXPECT_EQ(CR_RESULT_SUCCESS, data.result);
	for (int i = 0; i < data.initialToolPose.size(); i++) {
		EXPECT_NEAR(data.initialToolPose(i), data.finalToolPose(i), 0.22);
	}
	for (int i = 0; i < data.initialJointAngles.size(); i++) {
		EXPECT_NE(data.initialJointAngles(i), data.finalJointAngles(i));
	}
}
