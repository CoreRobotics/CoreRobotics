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

#ifndef CRNullSpace_hpp
#define CRNullSpace_hpp

//=====================================================================
// Includes
#include "model/CRManipulator.hpp"
#include "core/CRTypes.hpp"

//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics  {

//=====================================================================
/*!
 \file CRNullSpace.hpp
 \brief Implements a class to solve manipulator nullspace movements.
 */
//---------------------------------------------------------------------
/*!
 \class CRNullSpace
 \ingroup controllers
 
 \brief This class provides methods for solving the manipulator nullspace
 control problem.
 
 \details
 ## Description

 CRNullSpace implements the orthogonal projection matrix to find a vector
 in the nullspace of the Jacobian given an arbetrary vector of joint space
 velocities.  The SVD based generalized inverse function in CRMath is used
 to find the sudoinverse of the Jacobian.

 CRNullSpace find the orthogonal projection of the arbetrary joint space
 vector usingthe following equation,
 
 \f$ \dot{q}_{null} = (I - J^\dagger J) \dot{q}_0 \f$,
 
 where \f$\dot{q}_0\f$ is the desired joint velocities, \f$J\f$ is the robot
 jacobian, and \f$\dot{q}_{null}\f$ is a set of joint velocities in the nullspace
 of the robot jacobian.
 
 These methods are used to interface with the nullspace controller:
 - CRNullSpace::setRobot sets the manipulator nullspace to solve
 - CRNullSpace::setToolIndex set the tool index of the 
 associated manipulator (see CRManipulator::addTool) for which we are
 setting a desired nullspace command.  Note that a tool must be added in the manipulator
 prior to using the nullspace solver.
 - CRNullSpace::setEulerMode sets the Euler convention of the pose
 vector
 - CRNullSpace::solve solves the nullspace control problem described above for
 an initial configuration and a desired set of joint velocities.  The method is
 overloaded to allow for solving for a reduced pose vector (see CRFrame::getPose)
 
 Optimization parameters can be set:
 - CRNullSpace::setSingularThresh sets the threshold for numerically
 determining if the Jacobian is singular by comparing singular values of
 the SVD to the threshold. (Default = 0.1)
 
 
 ## Example
 This example demonstrates use of the CRNullSpace class.
 \include example_CRNullSpace.cpp
 
 ## References
 [3] B. Siciliano and O. Khatib, "Springer Handbook of Robotics",
 Springer, 2016.\n\n 
 
 */
//=====================================================================
class [[deprecated(CR_DEPRECATED)]] CRNullSpace {

//---------------------------------------------------------------------
// Constructor and Destructor
public:

	//! Class constructor
	CRNullSpace(const CRManipulator& i_robot,
                unsigned int i_toolIndex,
                CREulerMode i_eulerMode);

//---------------------------------------------------------------------
// Get/Set Methods
public:

	//! Set the robot to be used
	void setRobot(const CRManipulator& i_robot) {this->m_robot = i_robot;}

	//! Set robot tool index to use to compute the nullspace.  Note that a tool must be specified in the robot.
	void setToolIndex(unsigned int i_toolIndex) {this->m_toolIndex = i_toolIndex;}

	//! Set the Euler angle convention of the IK solver.  This must match the
    //  Euler convention used to supply the set point in the solve() method.
	void setEulerMode(CREulerMode i_eulerMode) {this->m_eulerMode = i_eulerMode;}

	//! Get the Euler convention
	CREulerMode getEulerMode(void) {return this->m_eulerMode;}

	//! Set the minimum threshold for a non-singular matrix
	void setSingularThresh(double i_thresh) {this->m_svdTol = i_thresh;}

	//! Get the minimum threshold for a non-singular matrix
	double getSingularThresh(void) {return this->m_svdTol;}

	//! Set the minimum step size
	void setMinStepSize(double i_stepSize) {this->m_stepSize = i_stepSize;}

	//! Get the minimum step size
	double getMinStepSize(void) {return this->m_stepSize;}

	//! Set the maximum number of iterations
	void setMaxIter(double i_maxIter) {this->m_maxIter = i_maxIter;}

	//! Get the maximum number of iterations
	double getMaxIter(void) {return this->m_maxIter;}

	//! Set the tolerance for checking if the nullspace is trivial
	void setTrivialTolerance(double i_trivialTolerance) {this->m_trivTol = i_trivialTolerance;}

	//! Get the tolerance for checking if the nullspace is trivial
	double getTrivialTolerance(void) {return this->m_trivTol;}

//---------------------------------------------------------------------
// Public Methods
public:

	//! Solve for the joint velocities that yield no tool motion, or tool motion only in the degrees of freedom specified in i_poseElements
	CRResult solve(Eigen::VectorXd i_jointMotion,
                   Eigen::VectorXd i_q0,
                   Eigen::VectorXd &o_nullSpaceJointMotion);

	CRResult solve(Eigen::VectorXd i_jointMotion,
                   Eigen::VectorXd i_q0,
                   Eigen::Matrix<bool, 6, 1> i_poseElements,
                   Eigen::VectorXd &o_nullSpaceJointMotion);

	CRResult solve(Eigen::VectorXd i_jointMotion,
                   Eigen::VectorXd i_q0,
                   Eigen::Matrix<bool, 6, 1> i_poseElements,
                   Eigen::MatrixXd i_w,
                   Eigen::VectorXd &o_nullSpaceJointMotion);

	CRResult solve(Eigen::VectorXd i_jointMotion,
                   Eigen::VectorXd i_q0,
                   Eigen::Matrix<int, 6, 1> i_poseElementsInt,
                   Eigen::VectorXd &o_nullSpaceJointMotion);

//---------------------------------------------------------------------
// Protected Members
protected:

	//! Manipulator object to solve
	CRManipulator m_robot;

	//! Index of the manipulator tool for which to solve the IK
	unsigned int m_toolIndex;

	//! Euler Convention to use
	CREulerMode m_eulerMode;

	//! Tolerance for computing if a matrix is singular using SVD svals
	double m_svdTol;

	//! The minimum step size to use when computing the nullspace
	double m_stepSize;

	//! The maximum number of iterations to compute when finding the null space
	double m_maxIter;

	//! The tolerance used for checking if the nullspace is trivial
	double m_trivTol;

};

//=====================================================================
// End namespace
}

#endif
