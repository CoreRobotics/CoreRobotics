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

#include "CRHardLimits.hpp"
#include "core/CRTypes.hpp"
#include "math/CRMatrix.hpp"

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
 The constructor creates a hard limits solver. This solver creates an
 inverse kinematics and, if desired, a nullspace solver to solve the inverse
 kinematics problem with hard joint limits.\n

 \param[in]     i_robot           the CoreRobotics::CRManipulator object
                                  to be used for solving the inverse
                                  kinematics
 \param[in]     i_toolIndex       the tool index to use for solving the
                                  inverse kinematics problem
 \param[in]     i_eulerMode       the Euler convention of the pose vector
 \param[in]     i_useNullSpace    A boolean flag to indicate whether a
                                  nullspace solver should be created
 */
//---------------------------------------------------------------------
CRHardLimits::CRHardLimits(const CRManipulator &i_robot,
                           unsigned int i_toolIndex, CREulerMode i_eulerMode,
                           bool i_useNullSpace) {
  this->m_robot = i_robot;
  this->m_IKSolver =
      new CRInverseKinematics(this->m_robot, i_toolIndex, i_eulerMode);
  this->m_useNullSpace = i_useNullSpace;
  if (this->m_useNullSpace) {
    this->m_NullSpaceSolver =
        new CRNullSpace(this->m_robot, i_toolIndex, i_eulerMode);
  }
  this->m_upperLimits = Eigen::VectorXd::Constant(
      this->m_robot.getDegreesOfFreedom(), CRMaxJointRotation);
  this->m_lowerLimits = Eigen::VectorXd::Constant(
      this->m_robot.getDegreesOfFreedom(), CRMinJointRotation);
  this->m_poseElements << 1, 1, 1, 1, 1, 1;
}

//=====================================================================
/*!
 The constructor creates a hard limits solver. This solver creates an
 inverse kinematics solver to solve the inverse kinematics problem with
 hard joint limits.\n

 \param[in]     i_robot           the CoreRobotics::CRManipulator object
                                  to be used for solving the inverse
                                  kinematics
 \param[in]     i_toolIndex       the tool index to use for solving the
                                  inverse kinematics problem
 \param[in]     i_eulerMode       the Euler convention of the pose vector
 */
//---------------------------------------------------------------------
CRHardLimits::CRHardLimits(const CRManipulator &i_robot,
                           unsigned int i_toolIndex, CREulerMode i_eulerMode) {
  this->m_robot = i_robot;
  this->m_IKSolver =
      new CRInverseKinematics(this->m_robot, i_toolIndex, i_eulerMode);
  this->m_useNullSpace = false;
  this->m_upperLimits = Eigen::VectorXd::Constant(
      this->m_robot.getDegreesOfFreedom(), CRMaxJointRotation);
  this->m_lowerLimits = Eigen::VectorXd::Constant(
      this->m_robot.getDegreesOfFreedom(), CRMinJointRotation);
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
  Eigen::MatrixXd W = Eigen::MatrixXd::Identity(
      this->m_robot.getDegreesOfFreedom(), this->m_robot.getDegreesOfFreedom());
  Eigen::VectorXd q(this->m_robot.getDegreesOfFreedom());
  Eigen::VectorXd qNull(this->m_robot.getDegreesOfFreedom());
  Eigen::VectorXd qSolved(this->m_robot.getDegreesOfFreedom());
  Eigen::MatrixXd limits(this->m_robot.getDegreesOfFreedom(), 2);
  Eigen::Index row_index, col_index;
  limits << (this->m_q0 - this->m_upperLimits),
      (this->m_lowerLimits - this->m_q0);
  if (0 < limits.maxCoeff()) {
    o_qSolved = this->m_q0;
    return CR_RESULT_BAD_IC;
  }
  bool limits_broken = true;
  while (limits_broken) {
    result = this->m_IKSolver->solve(this->m_setPoint, this->m_poseElements,
                                     this->m_q0, W, qSolved);
    if (this->m_useNullSpace) {
      result = this->m_NullSpaceSolver->solve(this->m_jointMotion, this->m_q0,
                                              this->m_poseElements, W, qNull);
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
