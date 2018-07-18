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

#include "InverseKinematics.hpp"
#include "Matrix.hpp"

//=====================================================================
// CoreRobotics namespace
namespace cr {
    
    
//=====================================================================
/*!
 The constructor creates an inverse kinematics solver.  Specifically,
 the algorithm uses the Jacobian pseudoinverse approach.  The SVD is
 used to compute the Jacobian inverse.\n
 
 \param[in]     i_robot         the cr::world::Manipulator object 
                                to be used for solving the inverse 
                                kinematics
 \param[in]     i_toolIndex     the index of the robot tool for which 
                                the IK is being solved, see 
                                cr::world::Manipulator::addTool()
 \param[in]     i_eulerMode     the Euler convention of the pose vector
 */
//---------------------------------------------------------------------
InverseKinematics::InverseKinematics(const world::Manipulator& i_robot,
                                         unsigned int i_toolIndex,
                                         EulerMode i_eulerMode)
{
    this->setRobot(i_robot);
    this->setToolIndex(i_toolIndex);
    this->setEulerMode(i_eulerMode);
    this->setTolerance(0.001);  // 1 mm (rad)
    this->setMaxIter(1);        // 1 step optimizer
    this->setStepSize(0.1);     // 0.1 step gain - this was Alexi's default
    this->setSingularThresh(1.0e-1);
	this->setDampingFactor(0);	// traditional least squares
}
    
    
//=====================================================================
/*!
 Get the DLS jacobian inverse.\n
 
 \param[in]     i_jac    - jacobian matrix (without damping)
 \param[out]    o_jacInv - inverse jacobian matrix (with damping)
 \return        cr::Result result of the inversion operation
 */
//---------------------------------------------------------------------
core::Result InverseKinematics::getJacInv(Eigen::MatrixXd i_jac,
                                          Eigen::MatrixXd &o_jacInv)
{
    // SVD matrices
    Eigen::MatrixXd U, V, E;
    Eigen::VectorXd Sigma;
    
    // Compute the SVD
    core::Result result = Matrix::svd(i_jac, this->m_svdTol, U, Sigma, V);
    // result = CRMath::svdInverse(J, this->m_svdTol, Jinv);
    
    // compute the Damped singular values (E)
    // See http://www.andreasaristidou.com/publications/CUEDF-INFENG,%20TR-632.pdf (DLS)
    Eigen::VectorXd e = Sigma.array() / (Sigma.array().square() + pow(this->m_dampingFactor, 2));
    E = e.asDiagonal();
    
    // compute the generalized inverse jacobian
    o_jacInv = V * E * U.transpose();
    return result;
}


//=====================================================================
/*!
 This method computes the joint angles that solve the specified set 
 point.  An initial condition for the joint angles is specified via 
 i_q0.  The method returns a flag indicating if the pseudoinverse is
 singular and no solution can be found.\n
 
 \param[in]     i_setPoint      the pose vector set point.
 \param[in]     i_q0            the intial configuration to use for 
                                the iterations.
 \param[out]    o_qSolved       the new configuration
 \return        cr::Result result of the inversion operation
 */
//---------------------------------------------------------------------
core::Result InverseKinematics::solve(const Eigen::Matrix<double, 6, 1>& i_setPoint,
                                      Eigen::VectorXd i_q0,
                                      Eigen::VectorXd &o_qSolved)
{
    
    // indicator if solution is singular
    core::Result result = core::CR_RESULT_SUCCESS;         // break the algorithm if it is singular
    
    // set up variables
    Eigen::Matrix<double, 6, 1> error;  // error (setPoint - fk(q))
    Eigen::Matrix<double, 6, 1> pose;   // the value of the FK at iteration i
    Eigen::MatrixXd J, Jinv;               // Jacobian inverse
    Eigen::VectorXd q;                  // the configuration
    unsigned int iter = 0;
    
    // set the initial configuration
    q = i_q0;
    
    // set the initial robot configuration
    this->m_robot.setConfiguration(q);
    
    // return the pose of the robot for the initial configuration
    pose = this->m_robot.getToolPose(this->m_toolIndex, this->m_eulerMode);
    
    // compute the error by comparing the pose
    error = i_setPoint-pose;
    
    // optimization routine
    while ((error.norm() >= this->m_tolerance) &&
           (iter < this->m_maxIter) &&
           (result != core::CR_RESULT_SINGULAR)) {
        
        // Get the Jacobian in J
        J = this->m_robot.jacobian(this->m_toolIndex,
                                    this->m_eulerMode);
        
        // compute the generalized inverse jacobian
		result = this->getJacInv(J, Jinv);
        
        // Perform the iteration step
        q += this->m_stepSize * Jinv * error;
        
        // Now compute the error for the new config
        this->m_robot.setConfiguration(q);
        pose = this->m_robot.getToolPose(this->m_toolIndex, this->m_eulerMode);
        error = i_setPoint-pose;
        
        // update the iterator
        iter++;
        
    }
    
	// Put the robot back in its original configuration
	this->m_robot.setConfiguration(i_q0);

	// output the solution
	o_qSolved = q;

	// return result
	return result;
}
    

//=====================================================================
/*!
This method computes the joint angles that solve the specified set
point.  An initial condition for the joint angles is specified via
i_q0.  The method returns a flag indicating if the pseudoinverse is
singular and no solution can be found.\n

\param[in]     i_setPoint      the pose vector set point.
\param[in]     i_poseElements  a boolean vector indiciating which
                               elements of the pose vector are specified
							   in i_setPoint (see Frame::getPose)
\param[in]     i_q0            the intial configuration to use for
							   the iterations.
\param[out]    o_qSolved       the new configuration
\return                        a Result flag indicating if the
operation encountered a singularity
*/
//---------------------------------------------------------------------
core::Result InverseKinematics::solve(Eigen::VectorXd& i_setPoint,
                                      Eigen::Matrix<bool, 6, 1> i_poseElements,
                                      Eigen::VectorXd i_q0,
                                      Eigen::VectorXd &o_qSolved)
{
    
    // indicator if solution is singular
    core::Result result = core::CR_RESULT_SUCCESS;         // break the algorithm if it is singular
    
    // set up variables
    Eigen::VectorXd error;          // error (setPoint - fk(q))
    Eigen::VectorXd pose;           // the value of the FK at iteration i
    Eigen::MatrixXd J, Jinv;        // robot Jacobian and inverse
    Eigen::VectorXd q;              // the configuration
    unsigned int iter = 0;
    
    // set the initial configuration
    q = i_q0;
    
    // set the initial robot configuration
    this->m_robot.setConfiguration(q);
    
    // return the pose of the robot for the initial configuration
    pose = this->m_robot.getToolPose(this->m_toolIndex,
                                      this->m_eulerMode,
                                      i_poseElements);
    
    // compute the error by comparing the pose
    error = i_setPoint-pose;
    
    
    // optimization routine
    while ((error.norm() >= this->m_tolerance) &&
           (iter < this->m_maxIter) &&
           (result != core::CR_RESULT_SINGULAR)) {
        
        // Get the Jacobian in J
        J = this->m_robot.jacobian(this->m_toolIndex,
                                    this->m_eulerMode,
                                    i_poseElements);
        
        // compute the generalized inverse jacobian
        result = this->getJacInv(J, Jinv);
        
        // Perform the iteration step
        q += this->m_stepSize * Jinv * error;
        
        // Now compute the error for the new config
        this->m_robot.setConfiguration(q);
        pose = this->m_robot.getToolPose(this->m_toolIndex,
                                          this->m_eulerMode,
                                          i_poseElements);
        error = i_setPoint-pose;
        
        // update the iterator
        iter++;
        
    }
    
    // Put the robot back in its original configuration
    this->m_robot.setConfiguration(i_q0);
    
    // output the solution
    o_qSolved = q;

    // return result
    return result;
}


//=====================================================================
/*!
This method computes the joint angles that solve the specified set
point.  An initial condition for the joint angles is specified via
i_q0.  The method returns a flag indicating if the pseudoinverse is
singular and no solution can be found.\n

\param[in]     i_setPoint      the pose vector set point.
\param[in]     i_poseElements  a boolean vector indiciating which
                               elements of the pose vector are specified
							   in i_setPoint (see Frame::getPose)
\param[in]     i_q0            the intial configuration to use for
							   the iterations.
\param[in]     i_w             a matrix which will be multiplied by the
                               robot jacobian for the purpose of hard limits
\param[out]    o_qSolved       the new configuration
\return                        a Result flag indicating if the
operation encountered a singularity
*/
//---------------------------------------------------------------------
core::Result InverseKinematics::solve(Eigen::VectorXd& i_setPoint,
                                      Eigen::Matrix<bool, 6, 1> i_poseElements,
                                      Eigen::VectorXd i_q0,
                                      Eigen::MatrixXd i_w,
                                      Eigen::VectorXd &o_qSolved)
{
    
    // indicator if solution is singular
    core::Result result = core::CR_RESULT_SUCCESS;         // break the algorithm if it is singular
    
    // set up variables
    Eigen::VectorXd error;          // error (setPoint - fk(q))
    Eigen::VectorXd pose;           // the value of the FK at iteration i
    Eigen::MatrixXd J, Jinv;        // robot Jacobian and inverse
    Eigen::VectorXd q;              // the configuration
    unsigned int iter = 0;
    
    // set the initial configuration
    q = i_q0;
    
    // set the initial robot configuration
    this->m_robot.setConfiguration(q);
    
    // return the pose of the robot for the initial configuration
    pose = this->m_robot.getToolPose(this->m_toolIndex,
                                      this->m_eulerMode,
                                      i_poseElements);
    
    // compute the error by comparing the pose
    error = i_setPoint-pose;
    
    
    // optimization routine
    while ((error.norm() >= this->m_tolerance) &&
           (iter < this->m_maxIter) &&
           (result != core::CR_RESULT_SINGULAR)) {
        
        // Get the Jacobian in J
        J = this->m_robot.jacobian(this->m_toolIndex,
                                    this->m_eulerMode,
                                    i_poseElements);

		// Multiply by W for hard limits
		J = J * i_w;
        
        // compute the generalized inverse jacobian
        result = this->getJacInv(J, Jinv);
        
        // Perform the iteration step
        q += this->m_stepSize * Jinv * error;
        
        // Now compute the error for the new config
        this->m_robot.setConfiguration(q);
        pose = this->m_robot.getToolPose(this->m_toolIndex,
                                          this->m_eulerMode,
                                          i_poseElements);
        error = i_setPoint-pose;
        
        // update the iterator
        iter++;
        
    }
    
    // Put the robot back in its original configuration
    this->m_robot.setConfiguration(i_q0);
    
    // output the solution
    o_qSolved = q;

    // return result
    return result;
}


//=====================================================================
/*!
This method computes the joint angles that solve the specified set
point.  An initial condition for the joint angles is specified via
i_q0.  The method returns a flag indicating if the pseudoinverse is
singular and no solution can be found.\n

\param[in]     i_setPoint         the pose vector set point.
\param[in]     i_poseElementsInt  a integer vector indiciating which
                                  elements of the pose vector are specified
							      in i_setPoint (see world::Manipulator::jacobian)
\param[in]     i_q0               the intial configuration to use for
							      the iterations.
\param[out]    o_qSolved          the new configuration
\return                           a Result flag indicating if the
operation encountered a singularity
*/
//---------------------------------------------------------------------
core::Result InverseKinematics::solve(Eigen::VectorXd& i_setPoint,
                                      Eigen::Matrix<int, 6, 1> i_poseElementsInt,
                                      Eigen::VectorXd i_q0,
                                      Eigen::VectorXd &o_qSolved)
{
	Eigen::Matrix<bool, 6, 1> i_poseElements = i_poseElementsInt.cast<bool>();
	return InverseKinematics::solve(i_setPoint, i_poseElements, i_q0, o_qSolved);
}

//=====================================================================
// End namespace
}
