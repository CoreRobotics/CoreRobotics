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

using namespace CoreRobotics;

void test_CRNullSpace(void) {

	std::cout << "*************************************\n";
	std::cout << "Demonstration of CRNullSpace.\n";

	CRManipulator* MyRobot = new CRManipulator();

	// Initialize translation frames
	CRFrameEuler* F0 = new CRFrameEuler();
	CRFrameEuler* F1 = new CRFrameEuler();
	CRFrameEuler* F2 = new CRFrameEuler();
	CRFrameEuler* F3 = new CRFrameEuler();
	CRFrameEuler* F4 = new CRFrameEuler();
	CRFrameEuler* F5 = new CRFrameEuler();
	CRFrameEuler* F6 = new CRFrameEuler();

	// Initialize links
	CRRigidBody* Link0 = new CRRigidBody();
	CRRigidBody* Link1 = new CRRigidBody();
	CRRigidBody* Link2 = new CRRigidBody();
	CRRigidBody* Link3 = new CRRigidBody();
	CRRigidBody* Link4 = new CRRigidBody();
	CRRigidBody* Link5 = new CRRigidBody();
	CRRigidBody* Link6 = new CRRigidBody();


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
	CRFrameEuler* Tool = new CRFrameEuler();
	Tool->setFreeVariable(CR_EULER_FREE_NONE);
	Tool->setMode(CR_EULER_MODE_XYZ);
	Tool->setPositionAndOrientation(90.21, 0.0, -107.43, -M_PI / 8.0, 0.0, 0.0);
	int toolIndex = MyRobot->addTool(linkIndex6, Tool);

	// Initialize the solver
	CRNullSpace nullSpaceSolver = CRNullSpace(MyRobot, toolIndex, CR_EULER_MODE_XYZ);

	// Set the robot orientation
	Eigen::VectorXd InitJoints(6);
	InitJoints << 0.0, 2.0, 0.0, -2, 0.0, M_PI / 8.0;
	MyRobot->setConfiguration(InitJoints);

	// Compute the initial tool pose
	Eigen::VectorXd toolPose;
	toolPose = MyRobot->getToolPose(toolIndex, CR_EULER_MODE_XYZ);
	std::cout << "Initial Tool Pose\n" << toolPose.transpose() << std::endl;

	// Find a nullspace movement
	Eigen::VectorXd jointMotion(6);
	jointMotion << 1, 0, 0, 0, 0, 0;
	Eigen::VectorXd nullJointMotion(6);
	CRResult result = nullSpaceSolver.solve(jointMotion, InitJoints, nullJointMotion);
	if(result != CR_RESULT_SUCCESS) {std::cout << "Singular Jacobian" << std::endl;}
	std::cout << "NullSpace joint movements\n" << nullJointMotion.transpose() << std::endl;

	// Compute the final tool pose
	MyRobot->setConfiguration(InitJoints + nullJointMotion);
	toolPose = MyRobot->getToolPose(toolIndex, CR_EULER_MODE_XYZ);
	std::cout << "Final Tool Pose\n" << toolPose.transpose() << std::endl;

	// Set pose elements
	Eigen::Matrix<bool, 6, 1> poseElements;
	poseElements << 1, 1, 1, 1, 1, 0;

	// Find a nullspace movement with pose elements
	result = nullSpaceSolver.solve(jointMotion, InitJoints, poseElements, nullJointMotion);
	if(result != CR_RESULT_SUCCESS) {std::cout << "Singular Jacobian" << std::endl;}
	std::cout << "Reduced NullSpace joint movements\n" << nullJointMotion.transpose() << std::endl;

	// Compute the final tool pose
	MyRobot->setConfiguration(InitJoints + nullJointMotion);
	toolPose = MyRobot->getToolPose(toolIndex, CR_EULER_MODE_XYZ);
	std::cout << "Final Tool Pose\n" << toolPose.transpose() << std::endl;
}
