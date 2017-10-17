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

#include "CRHardLimits.hpp"
#include "CRMath.hpp"
#include "CRTypes.hpp"

//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics {

//=====================================================================

//! Maximum value of a double, used for removing upper joint limits
double CRMaxJointRotation = std::numeric_limits<double>::max();

//! Minimum value of a double, used for removing lower joint limits
double CRMinJointRotation = -std::numeric_limits<double>::max();

//=====================================================================
/*!
 The constructor creates a hard limits solver. This solver uses an
 inverse kinematics solver to solve the inverse kinematics problem
 with hard joint limits.\n
 
 \param[in]     i_IKSolver      the CoreRobotics::CRInverseKinematics
                                object to use to solve the inverse
                                kinematics problem
 */
//---------------------------------------------------------------------
CRHardLimits::CRHardLimits(CRInverseKinematics* i_IKSolver) {
	this->m_IKSolver = i_IKSolver;
	this->m_upperLimits.resize(this->m_IKSolver->getManipulator()->getDegreesOfFreedom());
	this->m_lowerLimits.resize(this->m_IKSolver->getManipulator()->getDegreesOfFreedom());
	for (int i = 0; i < this->m_IKSolver->getManipulator()->getDegreesOfFreedom(); i++) {
		this->m_upperLimits(i) = CRMaxJointRotation;
		this->m_lowerLimits(i) = CRMinJointRotation;
	}
	this->m_useNullSpace = false;
	this->m_poseElements << 1, 1, 1, 1, 1, 1;
}

//=====================================================================
/*!
 The constructor creates a hard limits solver. This solver uses an
 inverse kinematics solver and a nullspace solver to solve the inverse
 kinematics problem with hard joint limits.\n
 
 \param[in]     i_IKSolver               the CoreRobotics::CRInverseKinematics
                                         object to use to solve the inverse
                                         kinematics problem
 \param[in]     i_NullSpaceSolver        the CoreRobotics::CRNullSpace
                                         object to use to solve the null space
                                         control problem
 */
//---------------------------------------------------------------------
CRHardLimits::CRHardLimits(CRInverseKinematics* i_IKSolver,
                           CRNullSpace* i_NullSpaceSolver) {
	this->m_IKSolver = i_IKSolver;
	this->m_NullSpaceSolver = i_NullSpaceSolver;
	this->m_upperLimits.resize(this->m_IKSolver->getManipulator()->getDegreesOfFreedom());
	this->m_lowerLimits.resize(this->m_IKSolver->getManipulator()->getDegreesOfFreedom());
	for (int i = 0; i < this->m_IKSolver->getManipulator()->getDegreesOfFreedom(); i++) {
		this->m_upperLimits(i) = CRMaxJointRotation;
		this->m_lowerLimits(i) = CRMinJointRotation;
	}
	this->m_useNullSpace = true;
	this->m_poseElements << 1, 1, 1, 1, 1, 1;
}

//=====================================================================
/*!
 This method computes the joint angles that solve the inverse kinematics
 problem and, if desired, the nullspace control problem. This is completed
 with the set hard joint limits set in the class.\n
 
 \param[out]    o_qSolved       the new configuration
 \return                        a CRResult flag indicating if the 
                                operation encountered a singularity
 */
//---------------------------------------------------------------------
CRResult CRHardLimits::solve(Eigen::VectorXd &o_qSolved) {
	CRResult result = CR_RESULT_SUCCESS;
	Eigen::MatrixXd W = Eigen::MatrixXd::Identity(this->m_IKSolver->getManipulator()->getDegreesOfFreedom(),
                                                  this->m_IKSolver->getManipulator()->getDegreesOfFreedom());
	Eigen::VectorXd q(this->m_IKSolver->getManipulator()->getDegreesOfFreedom());
	Eigen::VectorXd qNull(this->m_IKSolver->getManipulator()->getDegreesOfFreedom());
	Eigen::VectorXd qSolved(this->m_IKSolver->getManipulator()->getDegreesOfFreedom());
	Eigen::MatrixXd limits(this->m_IKSolver->getManipulator()->getDegreesOfFreedom(), 2);
	Eigen::Index row_index, col_index;
	bool limits_broken = true;
	while (limits_broken) {
		result = this->m_IKSolver->solve(this->m_setPoint,
                                         this->m_poseElements,
                                         this->m_q0,
                                         W,
                                         qSolved);
		if (this->m_useNullSpace) {
			result = this->m_NullSpaceSolver->solve(this->m_jointMotion,
		                                            this->m_q0,
		                                            this->m_poseElements,
		                                            W,
		                                            qNull);
			qSolved += qNull;
		}
		if (result == CR_RESULT_SINGULAR) {
			o_qSolved = this->m_q0;
			return result;
		}
		limits_broken = false;
		limits << (qSolved - this->m_upperLimits), (this->m_lowerLimits - qSolved);
		if (0 < limits.maxCoeff(&row_index, &col_index)) {
			limits_broken = true;
			W(row_index, row_index) = 0;
		}
	}
	o_qSolved = qSolved;
	return result;
}

//=====================================================================
// End namespace
}		
