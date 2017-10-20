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

// Use the CoreRobotics namespace
using namespace CoreRobotics;


// -------------------------------------------------------------
void test_CRInverseKinematics(void) {

	std::cout << "*************************************\n";
	std::cout << "Demonstration of CRInverseKinematics.\n";
    std::cout << std::fixed; std::cout.precision(4);
    
    // Set the Euler convention we will use throughout the example
    // Although CoreRobotics offers the flexibility to choose a
    // different convention for each method, in general it is good
    // to adopt the same convention throughout a problem for
    // consistency.
    CREulerMode convention = CR_EULER_MODE_XYZ;

	// ------------------------------------------
	// Create the robot

	// create several rigid body links
    CRFrameEuler* F0 = new CRFrameEuler(0, 0, 0, 0, 0, 0,
                                        convention,
                                        CR_EULER_FREE_ANG_G);
	CRFrameEuler* F1 = new CRFrameEuler(1, 0, 0, 0, 0, 0,
                                        convention,
                                        CR_EULER_FREE_ANG_G);
	CRFrameEuler* F2 = new CRFrameEuler(2, 0, 0, 0, 0, 0,
                                        convention,
                                        CR_EULER_FREE_ANG_G);
    CRFrameEuler* F3 = new CRFrameEuler(1, 0, 0, 0, 0, 0,
                                        convention,
                                        CR_EULER_FREE_NONE);
	CRRigidBody* Link0 = new CRRigidBody(F0);
	CRRigidBody* Link1 = new CRRigidBody(F1);
	CRRigidBody* Link2 = new CRRigidBody(F2);
    CRRigidBody* Link3 = new CRRigidBody(F3);
    
    // Create a new robot & add the links
    CRManipulator* MyRobot = new CRManipulator();
    
	MyRobot->addLink(Link0);
	MyRobot->addLink(Link1);
	MyRobot->addLink(Link2);
    int attachLink = MyRobot->addLink(Link3);


	// create a tool frame and add to MyRobot
	CRFrameEuler* Tool = new CRFrameEuler(0, 0, 0, 0, 0, 0,
                                          convention,
                                          CR_EULER_FREE_NONE);
	int toolIndex = MyRobot->addTool(attachLink, Tool);
    
    
    // ------------------------------------------
    // Initialize a clock to time the solver
    CRClock timer = CRClock();
    double et;


	// ------------------------------------------
	// Solve several inverse kinematics problems
    
    // Set up an inverse kinematics object and attach the robot
    CRInverseKinematics ikSolver = CRInverseKinematics(MyRobot,
                                                       toolIndex,
                                                       convention);
    
    // Set up some variables we will use
    Eigen::VectorXd q0(3);          // initial configuration
    Eigen::VectorXd qSolved(3);     // configuration that solves p = fk(qSolved)
    Eigen::Matrix<double, 6, 1> p;  // tool set point pose
    Eigen::MatrixXd fk;             // forward kinematics (for testing the result)
    
    
    // **********************
    // CASE 1 : Solver should find a solution within default tolerance (1 mm),
    // step size (1), and gain (0.1)
    std::cout << "---------------------------------------------\n";
    std::cout << "CASE 1: Use the default solver parameters.\n";
    
    // Set the initial configuration of the robot
    q0 << 0.1, -0.2, 0.0;
    MyRobot->setConfiguration(q0);
    
    // Define a set point pose
    p << 2.5, 0, 0, 0, 0, 0;
    
    
    // Now solve the inverse kinematics for the point
    timer.startTimer();
    CRResult result = ikSolver.solve(p, q0, qSolved);
    et = timer.getElapsedTime();
    
    if ( result == CR_RESULT_SUCCESS ){
        printf("Non-singular solution found in %8.6f s!\n",et);
        std::cout << qSolved << std::endl;
        
        // Now push the new joints through the robot to see if it worked
        MyRobot->setConfiguration(qSolved);
        fk = MyRobot->getForwardKinematics();
        
        std::cout << "The forward kinematics for this solution are:\n";
        std::cout << fk << std::endl;
        
    } else {
        std::cout << "The solution is singular.\n";
        std::cout << qSolved << std::endl;
    }
    
    
    // **********************
    // CASE 2:
    std::cout << "---------------------------------------------\n";
    std::cout << "CASE 2: Change the default maximum iteration.\n";
    
    // Change the maximum iterations
    ikSolver.setMaxIter(100);
    
    // I.C.
    MyRobot->setConfiguration(q0);
    
    // Now solve the inverse kinematics for the point
    timer.startTimer();
    result = ikSolver.solve(p, q0, qSolved);
    et = timer.getElapsedTime();
    
    if ( result == CR_RESULT_SUCCESS ){
        printf("Non-singular solution found in %8.6f s!\n",et);
        std::cout << qSolved << std::endl;
        
        // Now push the new joints through the robot to see if it worked
        MyRobot->setConfiguration(qSolved);
        fk = MyRobot->getForwardKinematics();
        
        std::cout << "The forward kinematics for this solution are:\n";
        std::cout << fk << std::endl;
        
    } else {
		std::cout << "The solution is singular.\n";
        std::cout << qSolved << std::endl;
    }
    
    
    // **********************
    // CASE 3: Change the tolerance and gain
    std::cout << "---------------------------------------------\n";
    std::cout << "CASE 3: Change the parameters.\n";
    
    // Change the maximum iterations
    ikSolver.setMaxIter(100);
    ikSolver.setStepSize(0.2);
    ikSolver.setTolerance(0.0001);
    
    // I.C.
    MyRobot->setConfiguration(q0);
    
    // Now solve the inverse kinematics for the point
    timer.startTimer();
    result = ikSolver.solve(p, q0, qSolved);
    et = timer.getElapsedTime();
    
    if ( result == CR_RESULT_SUCCESS ){
        printf("Non-singular solution found in %8.6f s!\n",et);
        std::cout << qSolved << std::endl;
        
        // Now push the new joints through the robot to see if it worked
        MyRobot->setConfiguration(qSolved);
        fk = MyRobot->getForwardKinematics();
        
        std::cout << "The forward kinematics for this solution are:\n";
        std::cout << fk << std::endl;
        
    } else {
		std::cout << "The solution is singular.\n";
        std::cout << qSolved << std::endl;
    }
    
    
    // **********************
    // CASE 4: Assign a set point that is not reachable (singularity test)
    std::cout << "---------------------------------------------\n";
    std::cout << "CASE 4: Test a singular solution.\n";
    
    // Assign a set point outside the robot reach
    p << 5, 0, 0, 0, 0, 0;
    
    // I.C.
    MyRobot->setConfiguration(q0);
    
    // Now solve the inverse kinematics for the point
    timer.startTimer();
    result = ikSolver.solve(p, q0, qSolved);
    et = timer.getElapsedTime();
    
    if ( result == CR_RESULT_SUCCESS ){
        printf("Non-singular solution found in %8.6f s!\n",et);
        std::cout << qSolved << std::endl;
        
        // Now push the new joints through the robot to see if it worked
        MyRobot->setConfiguration(qSolved);
        fk = MyRobot->getForwardKinematics();
        
        std::cout << "The forward kinematics for this solution are:\n";
        std::cout << fk << std::endl;
        
    } else {
		std::cout << "The solution is singular.\n";
        std::cout << qSolved << std::endl;
    }
    
    
    
    // **********************
    // CASE 5: Try 200 steps (external)
    std::cout << "---------------------------------------------\n";
    std::cout << "CASE 5: Reduced pose vector.\n";
    
    // Assign a set point
    Eigen::Matrix<bool, 6, 1> elems;
    elems << 1, 1, 0, 0, 0, 1;
    
    Eigen::VectorXd pRed(3);
    pRed << 2.5, 0, 0;              // (x, y, g)
    
    // Change parameters
    ikSolver.setStepSize(0.1);
    ikSolver.setTolerance(0.001);
    
    // I.C.
    MyRobot->setConfiguration(q0);
    
    
    // Now solve the inverse kinematics for the point
    timer.startTimer();
    result = ikSolver.solve(pRed, elems, q0, qSolved);
    et = timer.getElapsedTime();
    
    if ( result == CR_RESULT_SUCCESS ){
        printf("Non-singular solution found in %8.6f s!\n",et);
        std::cout << qSolved << std::endl;
        
        // Now push the new joints through the robot to see if it worked
        MyRobot->setConfiguration(qSolved);
        fk = MyRobot->getForwardKinematics();
        
        std::cout << "The forward kinematics for this solution are:\n";
        std::cout << fk << std::endl;
        
    } else {
		std::cout << "The solution is singular.\n";
        std::cout << qSolved << std::endl;
    }
    
    
    
    // **********************
    // CASE 6: Try 200 steps (external)
    std::cout << "---------------------------------------------\n";
    std::cout << "CASE 6: Single step convergence.\n";
    
    // Assign a set point
    p << 2.5, 0, 0, 0, 0, 0;
    
    // Change parameters
    ikSolver.setMaxIter(1);
    
    // I.C.
    MyRobot->setConfiguration(q0);
    
    // Define a configruation
    Eigen::VectorXd q(3);
    q = q0;
    
    
    // Now solve the inverse kinematics for the point
    for (int i = 0; i < 100; i++){
        timer.startTimer();
        result = ikSolver.solve(p, q, qSolved);
        et = timer.getElapsedTime();
        
        q = qSolved;
        
        if ( result == CR_RESULT_SUCCESS ){
            printf("Solution found in %8.6f s!\n",et);
            
            // Now push the new joints through the robot to see if it worked
            MyRobot->setConfiguration(q);
            fk = MyRobot->getForwardKinematics();
            
        } else {
			std::cout << "The solution is singular.\n";
        }
    }
    
    
    

}
// -------------------------------------------------------------
