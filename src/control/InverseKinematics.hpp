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

#ifndef InverseKinematics_hpp
#define InverseKinematics_hpp

//=====================================================================
// Includes
#include "Eigen/Dense"
#include "Manipulator.hpp"
#include "Types.hpp"

//=====================================================================
// CoreRobotics namespace
namespace cr {
    
//=====================================================================
/*!
 \file InverseKinematics.hpp
 \brief Implements a class to solve manipulator inverse kinematics.
 */
//---------------------------------------------------------------------
/*!
 \class InverseKinematics
 \ingroup controllers
 
 \brief This class provides methods for solving the manipulator inverse
 kinematics (IK) problem.
 
 \details
 ## Description
 InverseKinematics implements the Jacobian generalized inverse 
 technique using the SVD for finding a manipulator configuration that
 yields a desired pose.
 
 \f$ x = f(q) \f$,
 
 where \f$x\f$ is the desired pose, \f$f\f$ is the forward kinematics
 function, and \f$q\f$ is the configuration we are trying to find.  The
 algorithm accomplishes this by iteratively updating the configuration 
 according to
 
 \f$ q_{k+1} = q_k + \gamma J^\dagger (x - f(q_k)) \f$
 
 These methods are used to interface with the IK controller:
 - InverseKinematics::setRobot sets the manipulator IK to solve
 - InverseKinematics::setToolIndex set the tool index of the 
 associated manipulator (see Manipulator::addTool) for which we are
 setting a desired pose.  Note that a tool must be added in the manipulator
 prior to using the IK solve.
 - InverseKinematics::setEulerMode sets the Euler convention of the pose
 vector
 - InverseKinematics::solve solves the IK problem described above for
 an initial configuration and a desired pose vector.  The method is overloaded
 to allow for solving for a reduced pose vector (see Frame::getPose)
 
 Optimization parameters can be set:
 - InverseKinematics::setTolerance sets the convergence tolerance of
 the algorithm.  The tolerance is compared to the 2-norm of the pose error.
 (Default = 0.001)
 - InverseKinematics::setMaxIter sets the maximum number of iterations the
 solver can run prior to stopping. (Default = 1)
 - InverseKinematics::setStepSize sets the convergence gain \f$\gamma\f$ 
 in the above equation. (Default = 0.1)
 - InverseKinematics::setDampingFactor sets the damping factor in the
 approximation of Jinv.  High damping is more immune to singularities (Default = 0)
 - InverseKinematics::setSingularThresh sets the threshold for numerically
 determining if the Jacobian is singular by comparing singular values of
 the SVD to the threshold. (Default = 0.1)
 
 
 ## Example
 This example demonstrates use of the InverseKinematics class.
 \include example_InverseKinematics.cpp
 
 ## References
 [1] O. Khatib, Lecture Notes (CS327A): "Advanced Robotic Manipulation",
 http://www.in.tum.de/fileadmin/user_upload/Lehrstuehle/Lehrstuhl_XXIII/AdvancedRoboticManipulation.pdf 2005.\n\n
 
 [2] A. Aristidou and J. Lasenby, "Inverse Kinematics: a review of
 existing techniques and introduction of a new fast iterative solver,"
 University of Cambridge, Technical Report, 2009.
 http://www.andreasaristidou.com/publications/CUEDF-INFENG,%20TR-632.pdf
 \n\n
 
 */
//=====================================================================
class InverseKinematics {
    
//---------------------------------------------------------------------
// Constructor and Destructor
public:
    
    //! Class constructor
    InverseKinematics(const Manipulator& i_robot,
                        unsigned int i_toolIndex,
                        CREulerMode i_eulerMode);
    
//---------------------------------------------------------------------
// Get/Set Methods
public:
    
    //! Set the robot to be used
    void setRobot(const Manipulator& i_robot) {this->m_robot = i_robot;}
    
    //! Get the robot being used
    Manipulator getRobot(void) {return m_robot;}
    
    //! Set robot tool index to use to compute the IK.  Note that a tool must be specified in the robot.
    void setToolIndex(unsigned int i_toolIndex) {this->m_toolIndex = i_toolIndex;}
    
    //! Get robot tool index
    unsigned int getToolIndex(void) {return this->m_toolIndex;}
    
    //! Set the Euler angle convention of the IK solver.  This must match the
    //  Euler convention used to supply the set point in the solve() method.
    void setEulerMode(CREulerMode i_eulerMode) {this->m_eulerMode = i_eulerMode;}
    
    //! Get the Euler convention
    CREulerMode getEulerMode(void) {return this->m_eulerMode;}
    
    //! Set the algorithm convergence tolerance
    void setTolerance(double i_tolerance) {this->m_tolerance = i_tolerance;}
    
    //! Get the algorithm convergence tolerance
    double getTolerance(void) {return this->m_tolerance;}
    
    //! Set the maximum number of iterations the algorithm can run
    void setMaxIter(unsigned int i_maxIter) {this->m_maxIter = i_maxIter;}
    
    //! Get the algorithm convergence tolerance
    unsigned int getMaxIter(void) {return this->m_maxIter;}
    
    //! Set the iteration step size (i.e. the gain)
    void setStepSize(double i_stepSize) {this->m_stepSize = i_stepSize;}
    
    //! Get the iteration step size (i.e. the gain)
    double getStepSize(void) {return this->m_stepSize;}

	//! Set the damping factor (for damped least squares solvers)
	void setDampingFactor(double i_dampingFactor) { this->m_dampingFactor = i_dampingFactor; }

	//!  Get the damping factor (for damped least squares solvers)
	double getDampingFactor(void) { return this->m_dampingFactor; }
    
    //! Set the minimum threshold for a non-singular matrix
    void setSingularThresh(double i_thresh) {this->m_svdTol = i_thresh;}
    
    //! Get the minimum threshold for a non-singular matrix
    double getSingularThresh(void) {return this->m_svdTol;}
    
    //! Get the DLS jacobian inverse
    Result getJacInv(Eigen::MatrixXd i_jac, Eigen::MatrixXd &o_jacInv);

    
//---------------------------------------------------------------------
// Public Methods
public:
    
    //! Solve for the joint angles (q) that yield the desired setPoint
    Result solve(const Eigen::Matrix<double, 6, 1>& i_setPoint,
				   Eigen::VectorXd i_q0,
				   Eigen::VectorXd &o_qSolved);
    
    Result solve(Eigen::VectorXd& i_setPoint,
				   Eigen::Matrix<bool, 6, 1> i_poseElements,
				   Eigen::VectorXd i_q0,
				   Eigen::VectorXd &o_qSolved);

	Result solve(Eigen::VectorXd& i_setPoint,
                   Eigen::Matrix<bool, 6, 1> i_poseElements,
                   Eigen::VectorXd i_q0,
                   Eigen::MatrixXd i_w,
                   Eigen::VectorXd &o_qSolved);

	Result solve(Eigen::VectorXd& i_setPoint,
				   Eigen::Matrix<int, 6, 1> i_poseElementsInt,
				   Eigen::VectorXd i_q0,
				   Eigen::VectorXd &o_qSolved);
    
//---------------------------------------------------------------------
// Protected Members
protected:
    
    //! Manipulator object to solve
    Manipulator m_robot;
    
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

	//! Damping term (DLS)
	double m_dampingFactor;
    
    //! Tolerance for computing if a matrix is singular using SVD svals
    double m_svdTol;
    
    
};

//=====================================================================
// End namespace
}


#endif
