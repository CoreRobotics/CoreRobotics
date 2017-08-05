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

#include "CRNullSpace.hpp"
#include "CRMath.hpp"

//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics {


//=====================================================================
/*!
 The constructor creates an nullspace solver.  Specifically,
 the algorithm uses an orthogonal projection matrix.  The SVD is
 used to compute the Jacobian generalized inverse.\n
 
 \param[in]     i_robot         the CoreRobotics::CRManipulator object 
                                to be used for solving for the null space
 \param[in]     i_toolIndex     the index of the robot tool for which 
                                the IK is being solved, see 
                                CoreRobotics::CRManipulator::addTool()
 \param[in]     i_eulerMode     the Euler convention of the pose vector
 */
//---------------------------------------------------------------------
CRNullSpace::CRNullSpace(CRManipulator* i_robot,
                         unsigned int i_toolIndex,
                         CREulerMode i_eulerMode)
{
	this->setRobot(i_robot);
	this->setToolIndex(i_toolIndex);
	this->setEulerMode(i_eulerMode);
	this->setSingularThresh(1.0e-1);
}


//=====================================================================
/*!

 This method computes the joint velocities that produce no tool movement.
 Desired joint valocities are specified via i_velocities, and an initial
 condition for the joint angles is specified via i_q0. The method returns
 a vector of joint velocities in the nullspace of the jacobian.\n
 
 \param[in]     i_velcities     the desired joint velocities
 \param[in]     i_q0            the intial configuration of the robot
 \return                        the joint velcities in the nullspace of
                                robot jacobian
 */
//---------------------------------------------------------------------
Eigen::VectorXd CRNullSpace::solve(Eigen::VectorXd i_velocities,
                                   Eigen::VectorXd i_q0)
{
	this->m_robot->setConfiguration(i_q0);
	Eigen::MatrixXd J, Jinv;
	J = this->m_robot->jacobian(this->m_toolIndex,
                                this->m_eulerMode);
	CRMath::svdInverse(J, this->m_svdTol, Jinv);
	return (Eigen::MatrixXd::Identity(this->m_robot->getDegreesOfFreedom(), this->m_robot->getDegreesOfFreedom()) - (Jinv * J)) * i_velocities;
}

//=====================================================================
/*!

 This method computes the joint velocities that produce no tool movement
 in the directions specified by i_poseElements.  Desired joint valocities
 are specified via i_velocities, and an initial condition for the joint
 angles is specified via i_q0. The method returns a vector of joint velocities
 in the nullspace of the jacobian.\n
 
 \param[in]     i_velcities     the desired joint velocities
 \param[in]     i_poseElements  a boolean vector indiciating which
                                elements of the pose vector are specified
							    in i_setPoint (see CRFrame::getPose)
 \param[in]     i_q0            the intial configuration of the robot
 \return                        the joint velcities in the nullspace of
                                robot jacobian
 */
//---------------------------------------------------------------------
Eigen::VectorXd CRNullSpace::solve(Eigen::VectorXd i_velocities,
                                   Eigen::Matrix<bool, 6, 1> i_poseElements,
                                   Eigen::VectorXd i_q0)
{
	this->m_robot->setConfiguration(i_q0);
	Eigen::MatrixXd J, Jinv;
	J = this->m_robot->jacobian(this->m_toolIndex,
                                this->m_eulerMode,
								i_poseElements);
	CRMath::svdInverse(J, this->m_svdTol, Jinv);
	return (Eigen::MatrixXd::Identity(this->m_robot->getDegreesOfFreedom(), this->m_robot->getDegreesOfFreedom()) - (Jinv * J)) * i_velocities;
}

//=====================================================================
/*!

 This method computes the joint velocities that produce no tool movement
 in the directions specified by i_poseElements.  Desired joint valocities
 are specified via i_velocities, and an initial condition for the joint
 angles is specified via i_q0. The method returns a vector of joint velocities
 in the nullspace of the jacobian.\n
 
 \param[in]     i_velcities     the desired joint velocities
 \param[in]     i_poseElements  a integer vector indiciating which
                                elements of the pose vector are specified
							    in i_setPoint (see CRManipulator::jacobian)
 \param[in]     i_q0            the intial configuration of the robot
 \return                        the joint velcities in the nullspace of
                                robot jacobian
 */
//---------------------------------------------------------------------
Eigen::VectorXd CRNullSpace::solve(Eigen::VectorXd i_velocities,
                                   Eigen::Matrix<int, 6, 1> i_poseElementsInt,
                                   Eigen::VectorXd i_q0)
{
	Eigen::Matrix<bool, 6, 1> i_poseElements = i_poseElementsInt.cast<bool>();
	return CRNullSpace::solve(i_velocities, i_poseElements, i_q0);
}

//=====================================================================
// End namespace
}
