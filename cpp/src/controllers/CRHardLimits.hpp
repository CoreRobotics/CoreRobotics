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
 \version 0.0
 
 */
//=====================================================================

#ifndef CRHardLimits_hpp
#define CRHardLimits_hpp

//=====================================================================
// Includes
#include "CRManipulator.hpp"
#include "CRTypes.hpp"
#include "CRInverseKinematics.hpp"
#include "CRNullSpace.hpp"
#include <limits>

//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics {

//=====================================================================
/*!
 \file CRHardLimits.hpp
 \brief Implements a class to solve manipulator inverse kinematics and
 nullspace movements under hard joint limits.
 */
//---------------------------------------------------------------------
/*!
 \class CRHardLimits
 \ingroup controllers
 
 \brief This class provides methods for solving the manipulator inverse
 kinematics and nullspace control problems under hard joint limits.
 
 \details
 ## Description

 ### ToDo

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

 ### ToDo

 This example demonstrates use of the CRNullSpace class.
 \include test_CRNullSpace.cpp
 
 ## References

 ### ToDo

 [3] B. Siciliano and O. Khatib, "Springer Handbook of Robotics",
 Springer, 2016.\n\n 
 
 */
//=====================================================================
class CRHardLimits {

//---------------------------------------------------------------------
// Constructor and Destructor
public:

	//! Class constructor
	CRHardLimits(CRInverseKinematics* i_IKSolver);

	CRHardLimits(CRInverseKinematics* i_IKSolver,
                 CRNullSpace* i_NullSpaceSolver);

//---------------------------------------------------------------------
// Get/Set Methods
public:

	//! Set the IK solver to use
	void setIKSolver(CRInverseKinematics* i_solver) {this->m_IKSolver = i_solver;}

	//! Get the IK solver
	CRInverseKinematics* getIKSolver(void) {return this->m_IKSolver;}

	//! Set the NullSpace solver to use
	void setNullSpaceSolver(CRNullSpace* i_solver) {
		this->m_NullSpaceSolver = i_solver;
		this->m_useNullSpace = true;}

	//! Get the NullSpace solver
	CRNullSpace* getNullSpaceSolver(void) {return this->m_NullSpaceSolver;}

	//! Enable or Disable the NullSpace solver
	void useNullSpace(bool i_useNullSpace) {this->m_useNullSpace = i_useNullSpace;}

	//! Get NullSpace usage status
	bool nullSpaceStatus(void) {return this->m_useNullSpace;}

	//! Set pose elements
	void setPoseElements(Eigen::Matrix<bool, 6, 1> i_poseElements)
		{this->m_poseElements = i_poseElements;}
	void setPoseElements(Eigen::Matrix<int, 6, 1> i_poseElements)
		{this->m_poseElements = i_poseElements.cast<bool>();}

	//! Get pose elements
	Eigen::Matrix<bool, 6, 1> getPoseElements(void)
		{return this->m_poseElements;}
	//Eigen::Matrix<int, 6, 1> getPoseElements(void)
	//	{return this->m_poseElements.cast<int>();}

//---------------------------------------------------------------------
// Limits Methods

	//! Set the limits on a given joint
	void setJointLimits(int i_jointIndex, double i_lowerLimit, double i_upperLimit) {
		this->m_lowerLimits(i_jointIndex) = i_lowerLimit;
		this->m_upperLimits(i_jointIndex) = i_upperLimit;}

	//! Set the upper limit on a given joint
	void setJointUpperLimit(int i_jointIndex, double i_upperLimit) {
		this->m_upperLimits(i_jointIndex) = i_upperLimit;}

	//! Set the lower limit on a given joint
	void setJointLowerLimit(int i_jointIndex, double i_lowerLimit) {
		this->m_lowerLimits(i_jointIndex) = i_lowerLimit;}

	//! Get the upper limit on a given joint
	double getJointUpperLimit(int i_jointIndex) {return this->m_upperLimits(i_jointIndex);}

	//! Get the lower limit on a given joint
	double getJointLowerLimit(int i_jointIndex) {return this->m_lowerLimits(i_jointIndex);}

	//! Set the upper and lower limits on all joints
	void setJointLimits(Eigen::VectorXd i_lowerLimits, Eigen::VectorXd i_upperLimits) {
		this->m_lowerLimits = i_lowerLimits;
		this->m_upperLimits = i_upperLimits;}

	//! Set the upper limits on all joints
	void setJointUpperLimits(Eigen::VectorXd i_upperLimits) {
		this->m_upperLimits = i_upperLimits;}

	//! Set the lower limits on all joints
	void setJointLowerLimits(Eigen::VectorXd i_lowerLimits) {
		this->m_lowerLimits = i_lowerLimits;}

	//! Get the upper limits on all joints
	Eigen::VectorXd getJointUpperLimits(void) {return this->m_upperLimits;}

	//! Get the lower limits on all joints
	Eigen::VectorXd getJointLowerLimits(void) {return this->m_lowerLimits;}

//---------------------------------------------------------------------
// Setpoint Methods
public:

	//! Set the initial joint configuration
	void setQ0(Eigen::VectorXd i_q0) {this->m_q0 = i_q0;}

	//! Get the initial joint configuration
	Eigen::VectorXd getQ0(void) {return this->m_q0;}
	
	//! Set the desired tool pose
	void setToolPose(Eigen::VectorXd i_setPoint) {this->m_setPoint = i_setPoint;}

	//! Get the desired tool pose
	Eigen::VectorXd getToolPose(void) {return this->m_setPoint;}

	//! Set the desired Null Space joint motion
	void setJointMotion(Eigen::VectorXd i_jointMotion) {
		this->m_jointMotion = i_jointMotion;
		this->m_useNullSpace = true;}

	//! Get the desired Null Space joint motion
	Eigen::VectorXd getJointMotion(void) {return this->m_jointMotion;}

//---------------------------------------------------------------------
// Solve Methods
public:

	//! Solve the IK and NullSpace control problems with the hard joint limits specified.
	CRResult solve(Eigen::VectorXd &o_qSolved);

//---------------------------------------------------------------------
// Protected Members
protected:

	//! The IK Solver to use
	CRInverseKinematics* m_IKSolver;

	//! The Null Space Solver to use
	CRNullSpace* m_NullSpaceSolver;

	//! Enable or disable the Null Space Solver
	bool m_useNullSpace;

	//! The pose elements to use
	Eigen::Matrix<bool, 6, 1> m_poseElements;

	//! The upper limits on joint position
	Eigen::VectorXd m_upperLimits;

	//! The lower limits on joint position
	Eigen::VectorXd m_lowerLimits;

	//! The robot initial configuration
	Eigen::VectorXd m_q0;

	//! The robot IK Solver set point
	Eigen::VectorXd m_setPoint;

	//! The robot desired nullspace joint motion
	Eigen::VectorXd m_jointMotion;

};

//=====================================================================
// End namespace
}

#endif
