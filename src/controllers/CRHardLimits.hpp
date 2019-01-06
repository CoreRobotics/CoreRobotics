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

#ifndef CRHardLimits_hpp
#define CRHardLimits_hpp

//=====================================================================
// Includes
#include "models/CRManipulator.hpp"
#include "core/CRTypes.hpp"
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

 CRHardLimits implaments a class which creates existing inverse kinematics
 and nullspace controller objects and applies hard limits to these classes.

 CRHardLimits finds a solution in joint space using,

 \f$\dot{q} = (JW)^\dagger\dot{x}+(I-(JW)^\dagger J)\dot{q}_N\f$,

 where \f$J\f$ is the robot jacobian, \f$W\f$ is a matrix starting off at
 identity, \f$\dot{x}\f$ is the desired tool velocity, \f$\dot{q}_N\f$ is the
 desired nullspace joint motion, and \f$\dot{q}\f$ is the resulting joint space
 velocities.
 
 The method works by starting with an identity matrix \f$W\f$ and removing entries
 for the most saturated joint and rerunning the solvers until the result meets the
 joint limits, or an inversion of a singular matrix is attempted where the initial
 condition is returned as the result.

 The solvers can be retrieved for the purposes of editing solver parameters using
 the following methods:
 - CRHardLimits::getIKSolver
 - CRHardLimits::getNullSpaceSolver

 Pose Elements can be set using the functions:
 - CRHardLimits::setPoseElements

 Joint limits can be set using the functions:
 - CRHardLimits::setJointLimits set the upper and lower joint limits for a single
 joint, or set all limits by passing an upper and lower limit vector
 - CRHardLimits::setJointUpperLimit set the upper limit on a given joint
 - CRHardLimits::setJointLowerLimit set the lower limit on a given joint
 - CRHardLimits::setJointUpperLimits set all upper limits with a vector
 - CRHardLimits::setJointLowerLimits set all lower limits with a vector

 To a remove a limit it can be set to CRMaxJointRotation or CRMinJointRotation for
 upper and lower limits respectively.

 Solver inputs can be set using the function:
 - CRHardLimits::setQ0 sets the robot initial condition
 - CRHardLimits::setToolPose sets the desired tool pose for the ik solver
 - CRHardLimits::setJointMotion sets the desired robot nullspace joint motion

 Finally the solver can be run using
 - CRHardLimits::solve
 
 
 ## Example

 This example demonstrates use of the CRHardLimits class.
 \include example_CRHardLimits.cpp
 
 ## References

 [1] F. Flacco, A. De Luca, and O. Khatib, "Control of Redundant Robots Under Hard Joint Constraints: Saturation in the Null Space", IEEE Transactions on Robotics, 2015.\n\n
 
 */
//=====================================================================
class CRHardLimits {

//---------------------------------------------------------------------
// Constructor and Destructor
public:

	//! Class constructor
	CRHardLimits(const CRManipulator& i_robot,
	             unsigned int i_toolIndex,
	             CREulerMode i_eulerMode);

	CRHardLimits(const CRManipulator& i_robot,
	             unsigned int i_toolIndex,
	             CREulerMode i_eulerMode,
	             bool i_useNullSpace);

//---------------------------------------------------------------------
// Get/Set Methods
public:

	//! Get the IK solver
	CRInverseKinematics* getIKSolver(void) {return this->m_IKSolver;}

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

	//! The CRManipulator object
	CRManipulator m_robot;

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
