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
 \version 0.0
 
 */
//=====================================================================

#include "CRInverseKinematics.hpp"
#include "CRMath.hpp"


//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics {
    
    
//=====================================================================
/*!
 The constructor creates an inverse kinematics solver.  Specifically,
 the algorithm uses the Jacobian pseudoinverse approach.  The SVD is
 used to compute the Jacobian inverse.\n
 
 \param[in] i_robot - the CoreRobotics::CRManipulator object to be
 used for solving the inverse kinematics
 \param[in] i_toolIndex - the index of the robot tool for which the IK
 is being solved, see CoreRobotics::CRManipulator::addTool()
 \param[in] i_eulerMode - the Euler convention of the pose vector
 */
//---------------------------------------------------------------------
CRInverseKinematics::CRInverseKinematics(CRManipulator* i_robot,
                                         unsigned int i_toolIndex,
                                         CREulerMode i_eulerMode)
{
    this->setRobot(i_robot);
    this->setToolIndex(i_toolIndex);
    this->setEulerMode(i_eulerMode);
    this->setTolerance(0.001);  // 1 mm (rad)
    this->setMaxIter(1);        // 1 step optimizer
    this->setStepSize(0.1);     // 0.1 step gain - this was Alexi's default
    this->setSingularThresh(1.0e-1);
}


//=====================================================================
/*!
 This method computes the joint angles that solve the specified set 
 point.  An initial condition for the joint angles is specified via 
 i_q0.  The method returns a flag indicating if the pseudoinverse is
 singular and no solution can be found.\n
 
 \param[in] i_setPoint - the pose vector set point.  If i_poseElements
 is specified, then the size of this vector must be equal to the number
 of true values in the i_poseElements.
 \param[in] i_poseElements - [optional] a boolean vector indiciating which
 elements of the full pose vector are used in i_setPoint.
 \param[in] i_q0 - the intial configuration to use for the iterations.
 \param[out] o_qSolved - the new configuration
 \return - a CRResult flag indicating if the operation encountered a
 singularity
 */
//---------------------------------------------------------------------
CRResult CRInverseKinematics::solve(Eigen::Matrix<double, 6, 1> i_setPoint,
                                    Eigen::VectorXd i_q0,
                                    Eigen::VectorXd &o_qSolved)
{
    
    // indicator if solution is singular
    CRResult result = CR_RESULT_SUCCESS;         // break the algorithm if it is singular
    
    // set up variables
    Eigen::Matrix<double, 6, 1> error;  // error (setPoint - fk(q))
    Eigen::Matrix<double, 6, 1> pose;   // the value of the FK at iteration i
    Eigen::MatrixXd J, Jinv;            // robot Jacobian and inverse
    Eigen::VectorXd q;                  // the configuration
    unsigned int iter = 0;
    
    // set the initial configuration
    q = i_q0;
    
    // set the initial robot configuration
    this->m_robot->setConfiguration(q);
    
    // return the pose of the robot for the initial configuration
    this->m_robot->getToolPose(this->m_toolIndex, this->m_eulerMode, pose);
    
    // compute the error by comparing the pose
    error = i_setPoint-pose;
    
    // optimization routine
    while ((error.norm() >= this->m_tolerance) &&
           (iter < this->m_maxIter) &&
           (result != CR_RESULT_SINGULAR)) {
        
        // Get the Jacobian in J
        J = this->m_robot->jacobian(this->m_toolIndex,
                                    this->m_eulerMode);
        
        // Invert with SVD
        result = CRMath::svdInverse(J, this->m_svdTol, Jinv);
        
        // Perform the iteration step
        q += this->m_stepSize * Jinv * error;
        
        // Now compute the error for the new config
        this->m_robot->setConfiguration(q);
        this->m_robot->getToolPose(this->m_toolIndex, this->m_eulerMode, pose);
        error = i_setPoint-pose;
        
        // update the iterator
        iter++;
        
    }
    
    // Put the robot back in its original configuration
    this->m_robot->setConfiguration(i_q0);
    
    // set the output configuration
    if (result == CR_RESULT_SINGULAR){
        o_qSolved = i_q0;    // return the initial condition
    } else {
        o_qSolved = q;        // return solution
    }
    
    // return result
    return result;
}
    

CRResult CRInverseKinematics::solve(Eigen::VectorXd i_setPoint,
                                    Eigen::Matrix<bool, 6, 1> i_poseElements,
                                    Eigen::VectorXd i_q0,
                                    Eigen::VectorXd &o_qSolved)
{
    
    // indicator if solution is singular
    CRResult result = CR_RESULT_SUCCESS;         // break the algorithm if it is singular
    
    // set up variables
    Eigen::VectorXd error;          // error (setPoint - fk(q))
    Eigen::VectorXd pose;           // the value of the FK at iteration i
    Eigen::MatrixXd J, Jinv;        // robot Jacobian and inverse
    Eigen::VectorXd q;              // the configuration
    unsigned int iter = 0;
    
    // set the initial configuration
    q = i_q0;
    
    // set the initial robot configuration
    this->m_robot->setConfiguration(q);
    
    // return the pose of the robot for the initial configuration
    this->m_robot->getToolPose(this->m_toolIndex,
                               this->m_eulerMode,
                               i_poseElements,
                               pose);
    
    // compute the error by comparing the pose
    error = i_setPoint-pose;
    
    
    // optimization routine
    while ((error.norm() >= this->m_tolerance) &&
           (iter < this->m_maxIter) &&
           (result != CR_RESULT_SINGULAR)) {
        
        // Get the Jacobian in J
        J = this->m_robot->jacobian(this->m_toolIndex,
                                    this->m_eulerMode,
                                    i_poseElements);
        
        // Invert with SVD
        result = CRMath::svdInverse(J, this->m_svdTol, Jinv);
        
        // Perform the iteration step
        q += this->m_stepSize * Jinv * error;
        
        // Now compute the error for the new config
        this->m_robot->setConfiguration(q);
        this->m_robot->getToolPose(this->m_toolIndex,
                                   this->m_eulerMode,
                                   i_poseElements,
                                   pose);
        error = i_setPoint-pose;
        
        // update the iterator
        iter++;
        
    }
    
    // Put the robot back in its original configuration
    this->m_robot->setConfiguration(i_q0);
    
    // set the output configuration
    if (result == CR_RESULT_SINGULAR){
        o_qSolved = i_q0;    // return the initial condition
    } else {
        o_qSolved = q;        // return solution
    }
    
    // return result
    return result;
}


//=====================================================================
// End namespace
}
