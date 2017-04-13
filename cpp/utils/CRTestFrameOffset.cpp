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
 \version 0.0
 
 */
//=====================================================================

#include <iostream>
#include "CoreRobotics.hpp"


// Use the CoreRobotics namespace
using namespace CoreRobotics;

void CRTestFrameOffset(void){
    
    std::cout << "**********************\n";
    std::cout << "Running the CRTestFrameOffset\n";

	CRManipulator MyRobot;

	CRFrameDh* F0 = new CRFrameDh();
	// pointer to the CRFrameDh Class, F0 will now reference the CRFrameDh not copy the structure
	CRFrameDh* F1 = new CRFrameDh();


	CRRigidBody* Link0 = new CRRigidBody();
	CRRigidBody* Link1 = new CRRigidBody();
	
	// Set info for Link 0 and add to MyRobot
    F0->setFreeVariable(CR_DH_FREE_THETA);
    F1->setFreeVariable(CR_DH_FREE_NONE);
	// F0->m_freeVar = CR_DH_FREE_THETA;
	// F1->m_freeVar = CR_DH_FREE_NONE;
	// DH free variables
		/*CR_DH_FREE_NONE
		CR_DH_FREE_R
		CR_DH_FREE_ALPHA
		CR_DH_FREE_D
		CR_DH_FREE_THETA */

	F0->setMode(CR_DH_MODE_MODIFIED);
	F1->setMode(CR_DH_MODE_MODIFIED);
	// DH modes
	/*	CR_DH_MODE_CLASSIC
		CR_DH_MODE_MODIFIED */

	/*double drvienangle;
	F0->getFreeValue(drvienangle); */

	F0->setParameters(0, 0, 0.2, 0, 3.1415/2.0);
	F1->setParameters(0.1, 0, 0, 0, 0);
	// Set Parameters
	/*(double 	r,
		double 	alpha,
		double 	d,
		double 	theta,
		double  offset		-		free variable offset distance or angle
		)*/

	Link0->setFrame(F0);
	Link1->setFrame(F1);
	MyRobot.addLink(Link0);
	MyRobot.addLink(Link1);

		
	F0->setFreeValue(0);

	// Now get the configuration values
	int dof;
	Eigen::VectorXd jointAngles;
	dof = MyRobot.getDegreesOfFreedom();
	jointAngles = MyRobot.getConfiguration();
	std::cout << "MyRobot has " << dof << " DOF, with joint angles = ("
	<< jointAngles.transpose() << ") rad" << std::endl;

	// Now get the Forward Kinematics and Jacobian
	Eigen::MatrixXd Jacobian, FwdKin;
	FwdKin = MyRobot.getForwardKinematics();
	// MyRobot.getJacobian(Jacobian);
	std::cout << "Forward Kinematics = \n" << FwdKin << std::endl;
	// std::cout << "Jacobian = \n" << Jacobian << std::endl;

}

