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
\version 0.0

*/
//=====================================================================
#include <iostream>
#include "CoreRobotics.hpp"

// Use the CoreRobotics namespace
using namespace CoreRobotics;


// -------------------------------------------------------------
void test_CRManipulator(void) {

	std::cout << "*************************************\n";
	std::cout << "Demonstration of CRManipulator.\n";

	// ------------------------------------------
	// Create a new robot
	CRManipulator MyRobot;

	// create a couple of rigid body links and add them to the manipulator
	CRFrameEuler* F0 = new CRFrameEuler();
	CRFrameEuler* F1 = new CRFrameEuler();
	CRFrameEuler* F2 = new CRFrameEuler();
	CRRigidBody* Link0 = new CRRigidBody();
	CRRigidBody* Link1 = new CRRigidBody();
	CRRigidBody* Link2 = new CRRigidBody();

	// Set info for Link 0 and add to MyRobot
	F0->freeVar = CR_EULER_FREE_ANG_G;
	F0->setMode(CR_EULER_MODE_XYZ);
	F0->setPositionAndOrientation(0, 0, 0.5, 0, 0, 0);
	Link0->frame = F0;
	MyRobot.addLink(Link0);

	// Set info for Link 1 and add to MyRobot
	F1->freeVar = CR_EULER_FREE_ANG_G;
	F1->setMode(CR_EULER_MODE_XYZ);
	F1->setPositionAndOrientation(1, 0, 0, 0, 0, 0);
	Link1->frame = F1;
	MyRobot.addLink(Link1);

	// Set info for Link 2 and add to MyRobot
	F2->freeVar = CR_EULER_FREE_NONE;
	F2->setMode(CR_EULER_MODE_XYZ);
	F2->setPositionAndOrientation(2, 0, 0, 0, 0, 0);
	Link2->frame = F2;
	MyRobot.addLink(Link2);


	// create a tool frame and add to MyRobot
	CRFrameEuler* Tool = new CRFrameEuler();
	Tool->setMode(CR_EULER_MODE_XYZ);
	Tool->setPositionAndOrientation(0, 0, 0, 0, 0, 0);
	MyRobot.addTool(2, Tool);


	// ------------------------------------------
	// Try out some of the methods available to the manipulator
	
	// Get the configuration values
	int dof;
	Eigen::VectorXd jointAngles;
	MyRobot.getDegreesOfFreedom(dof);
	MyRobot.getConfiguration(jointAngles);
	std::cout << "MyRobot has " << dof << " DOF, with joint angles = ("
		<< jointAngles.transpose() << ") rad" << std::endl;

	// Now get the Forward Kinematics and Jacobian
	Eigen::MatrixXd Jacobian, FwdKin;
	MyRobot.getForwardKinematics(FwdKin);
	MyRobot.getJacobian(0, CR_EULER_MODE_XYZ, Jacobian);
	std::cout << "Forward Kinematics = \n" << FwdKin << std::endl;
	std::cout << "Jacobian = \n" << Jacobian << std::endl;

	// now set a new robot configuration and get the FK and jacobian
	jointAngles << CoreRobotics::PI / 4.0, -CoreRobotics::PI / 2.0;
	std::cout << "Set joint angles = ("
		<< jointAngles.transpose() << ") rad" << std::endl;
	MyRobot.setConfiguration(jointAngles);
	MyRobot.getForwardKinematics(FwdKin);
	MyRobot.getJacobian(0, CR_EULER_MODE_XYZ, Jacobian);
	std::cout << "Forward Kinematics = \n" << FwdKin << std::endl;
	std::cout << "Jacobian = \n" << Jacobian << std::endl;

	// now get the transformation to the tool for the current configuration
	Eigen::Matrix4d T;
	CRFrame toolFrame;
	MyRobot.getToolFrame(0, toolFrame);
	toolFrame.getTransformToParent(T);
	std::cout << "MyRobot tool has a transformation of \n" << T << std::endl;

}
// -------------------------------------------------------------