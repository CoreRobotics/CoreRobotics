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
\author  Cameron Devine

*/
//=====================================================================

#include <iostream>
#include "CoreRobotics.hpp"

using namespace CoreRobotics;

int main(void) {

	std::cout << "*************************************\n";
	std::cout << "Demonstration of CRHardLimits.\n";

	CREulerMode convention = CR_EULER_MODE_XYZ;

	// Define a robot
	CRManipulator* MyRobot = new CRManipulator();

	// Define several frames
	CRFrameEuler* F0 = new CRFrameEuler(0, 0, 0, 0, 0, 0, convention, CR_EULER_FREE_ANG_G);
	CRFrameEuler* F1 = new CRFrameEuler(2, 0, 0, 0, 0, 0, convention, CR_EULER_FREE_ANG_G);
	CRFrameEuler* F2 = new CRFrameEuler(-4, 0, 0, 0, 0, 0, convention, CR_EULER_FREE_ANG_G);
	CRFrameEuler* F3 = new CRFrameEuler(1, 0, 0, 0, 0, 0, convention, CR_EULER_FREE_ANG_G);

	// Define the robot links
	CRRigidBody* Link0 = new CRRigidBody(F0);
	CRRigidBody* Link1 = new CRRigidBody(F1);
	CRRigidBody* Link2 = new CRRigidBody(F2);
	CRRigidBody* Link3 = new CRRigidBody(F3);

	// Add the links to the robot
	MyRobot->addLink(Link0);
	MyRobot->addLink(Link1);
	MyRobot->addLink(Link2);
	int linkIndex = MyRobot->addLink(Link3);

	// Create a tool and add it
	CRFrameEuler* Tool = new CRFrameEuler(1, 0, 0, 0, 0, 0, convention, CR_EULER_FREE_NONE);
	int toolIndex = MyRobot->addTool(linkIndex, Tool);

	// Initialize a Hard Limits solver
	CRHardLimits solver = CRHardLimits(*MyRobot, toolIndex, convention, true);

	// Change IK solver maximum iterations
	solver.getIKSolver()->setMaxIter(20);

	// Disable the nullspace solver for now
	solver.useNullSpace(false);

	// Set the pose elements
	Eigen::Matrix<bool, 6, 1> poseElements;
	poseElements << 1, 1, 0, 0, 0, 0;
	solver.setPoseElements(poseElements);

	// Define the initial configuration
	Eigen::VectorXd q(4), q0(4);
	q0 << 0, 0.1, 0.2, 0.3;

	// Define and set the goal location
	Eigen::VectorXd goal(2);
	goal << 0, 4.5;

	std::cout << "Setting goal tool pose to (" << goal.transpose() << "), (x, y)" << std::endl;

	solver.setToolPose(goal);

	// Run the solver
	q = q0;
	for (int i = 0; i < 5; i++) {
		solver.setQ0(q);
		solver.solve(q);
	}
	
	// Display the results
	std::cout << "Calculated joint configuration without joint limits (" << q.transpose() << ")" << std::endl;

	MyRobot->setConfiguration(q);
	std::cout << "Resulting tool pose (" << MyRobot->getToolPose(toolIndex, convention, poseElements).transpose() << ")" << std::endl;

	// Set jont limits
	solver.setJointLimits(2, -1, 1);

	// Run the solver
	q = q0;
	for (int i = 0; i < 5; i++) {
		solver.setQ0(q);
		solver.solve(q);
	}

	// Display the results
	std::cout << "Calculated joint configuration with joint limits (" << q.transpose() << ")" << std::endl;

	MyRobot->setConfiguration(q);
	std::cout << "Resulting tool pose (" <<  MyRobot->getToolPose(toolIndex, convention, poseElements).transpose() << ")" << std::endl;

	// Enable the nullspace solver
	solver.useNullSpace(true);

	// Set the desired nullspace joint motion
	Eigen::VectorXd desiredJointMotion(4);
	desiredJointMotion << -1, 0, 0, 0;

	std::cout << "Setting nullspace desired joint motion to (" << desiredJointMotion.transpose() << ")" << std::endl;

	solver.setJointMotion(desiredJointMotion);

	// Run the solver
	q = q0;
	for (int i = 0; i < 5; i++) {
		solver.setQ0(q);
		solver.solve(q);
	}

	// Display the results
	std::cout << "Calculated joint configuration with joint limits and nullspace control (" << q.transpose() << ")" << std::endl;

	MyRobot->setConfiguration(q);
	std::cout << "Resulting tool pose (" <<  MyRobot->getToolPose(toolIndex, convention, poseElements).transpose() << ")" << std::endl;

	// Set the initial condition to a point outside the limits
	q << 0, 0.1, M_PI, 0.3;
	std::cout << "Setting initial condition outisde the limits, (" << q.transpose() << ")" << std::endl;
	solver.setQ0(q);

	// Run the solver
	CRResult result = solver.solve(q);

	// Display the results
	std::cout << "Recieved the result ";
	if (result == CR_RESULT_SUCCESS) {
		std::cout << "CR_RESULT_SUCCESS (success)";
	} else if (result == CR_RESULT_SINGULAR) {
		std::cout << "CR_RESULT_SINGULAR (singular jacobian)";
	} else if (result == CR_RESULT_BAD_IC) {
		std::cout << "CR_RESULT_BAD_IC (bad initial conditions)";
	}
	std::cout << " with joint configuration (" << q.transpose() << ")" << std::endl;
}
