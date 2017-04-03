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

#ifndef CRInverseKinematics_hpp
#define CRInverseKinematics_hpp

//=====================================================================
// Includes
#include "Eigen/Dense"
#include "CRManipulator.hpp"

//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics {
    
//=====================================================================
/*!
 \file CRInverseKinematics.hpp
 \brief Implements a class that handles sensor models.
 */
//---------------------------------------------------------------------
/*!
 \class CRInverseKinematics
 \ingroup controllers
 
 \brief This class implements a sensor model.
 
 \details
 \section Description
 CRSensorModel implements a sensor model from a supplied observation
 callback function.  Specifically, CRSensorModel sets up a container
 for the model
 
 \f$ z = h(x) \f$,
 
 where \f$x\f$ is the state vector, and \f$z\f$ is the sensor 
 measurement vector.
 
 These methods are used to interface with the Sensor Model:
 - CRSensorModel::setState
 
 \section Example
 This example demonstrates use of the CRSensorModel class.
 \code
 
 
 
 \endcode
 
 \section References
 [1] O. Khatib, Lecture Notes (CS327A): "Advanced Robotic Manipulation",
 http://www.in.tum.de/fileadmin/user_upload/Lehrstuehle/Lehrstuhl_XXIII/AdvancedRoboticManipulation.pdf 2005.\n\n
 
 [2] A. Aristidou and J. Lasenby, "Inverse Kinematics: a review of 
 existing techniques and introduction of a new fast iterative solver,"
 University of Cambridge, Technical Report, 2009.
 http://www.andreasaristidou.com/publications/CUEDF-INFENG,%20TR-632.pdf
 \n\n
 
 */
//=====================================================================
class CRInverseKinematics {
    
//---------------------------------------------------------------------
// Constructor and Destructor
public:
    
    //! Class constructor
    CRInverseKinematics(CRManipulator* in_robot,
                        unsigned int in_toolIndex,
                        CREulerMode in_eulerMode,
                        Eigen::Matrix<bool, 6, 1> in_poseElements);
    CRInverseKinematics(CRManipulator* in_robot,
                        unsigned int in_toolIndex,
                        CREulerMode in_eulerMode);
    
//---------------------------------------------------------------------
// Get/Set Methods
public:
    
    //! Set the robot to be used
    void setRobot(CRManipulator* in_robot) {this->m_robot = in_robot;}
    
    //! Set robot tool index to use to compute the IK.  Note that a tool must be specified in the robot.
    void setToolIndex(unsigned int in_toolIndex) {this->m_toolIndex = in_toolIndex;}
    
    //! Set the Euler angle convention of the IK solver.  This must match the
    //  Euler convention used to supply the set point in the solve() method.
    void setEulerMode(CREulerMode in_eulerMode) {this->m_eulerMode = in_eulerMode;}
    
    //! Set the pose elements of interest (x, y, z, a, b, g)^T
    void setPoseElements(Eigen::Matrix<bool, 6, 1> in_poseElements){
        this->m_poseElements = in_poseElements;
    }
    
    //! Set the algorithm convergence tolerance
    void setTolerance(double in_tolerance) {this->m_tolerance = in_tolerance;}
    
    //! Set the maximum number of iterations the algorithm can run
    void setMaxIter(unsigned int in_maxIter) {this->m_maxIter = in_maxIter;}
    
    //! Set the iteration step size (i.e. the gain)
    void setStepSize(double in_stepSize) {this->m_stepSize = in_stepSize;}
    
    
//---------------------------------------------------------------------
// Public Methods
public:
    
    //! Solve for the joint angles (q) that yield the desired setPoint
    bool solve(Eigen::Matrix<double, 6, 1> in_setPoint,
               Eigen::VectorXd in_q0,
               Eigen::VectorXd &out_qSolved);
    
    bool solve(Eigen::VectorXd in_setPoint,
               Eigen::Matrix<bool, 6, 1> in_poseElements,
               Eigen::VectorXd in_q0,
               Eigen::VectorXd &out_qSolved);
    
//---------------------------------------------------------------------
// Protected Members
protected:
    
    //! Manipulator object to solve
    CRManipulator* m_robot;
    
    //! Index of the manipulator tool for which to solve the IK
    unsigned int m_toolIndex;
    
    //! Euler Convention to use
    CREulerMode m_eulerMode;
    
    //! Convergence threshold
    double m_tolerance;
    
    //! Maximum interations
    unsigned int m_maxIter;
    
    //! Optimizer step size (gain)
    double m_stepSize;
    
    //! Pose elements selection vector
    Eigen::Matrix<bool, 6, 1> m_poseElements;
    
    //! Tolerance for computing if a matrix is singular using SVD svals
    double m_svdTol = 1.0e-1;
    
    
};

//=====================================================================
// End namespace
}


#endif
