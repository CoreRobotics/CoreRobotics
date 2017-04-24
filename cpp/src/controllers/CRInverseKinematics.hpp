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
#include "CRTypes.hpp"

//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics {
    
//=====================================================================
/*!
 \file CRInverseKinematics.hpp
 \brief Implements a class to solve manipulator inverse kinematics.
 */
//---------------------------------------------------------------------
/*!
 \class CRInverseKinematics
 \ingroup controllers
 
 \brief This class provides methods for solving the manipulator inverse
 kinematics (IK) problem.
 
 \details
 ## Description
 CRInverseKinematics implements the Jacobian generalized inverse 
 technique using the SVD for finding a manipulator configuration that
 yields a desired pose.
 
 \f$ x = f(q) \f$,
 
 where \f$x\f$ is the desired pose, \f$f\f$ is the forward kinematics
 function, and \f$q\f$ is the configuration we are trying to find.  The
 algorithm accomplishes this by iteratively updating the configuration 
 according to
 
 \f$ q_{k+1} = q_k + \gamma J^\dagger (x - f(q_k)) \f$
 
 These methods are used to interface with the IK controller:
 - CRInverseKinematics::setRobot sets the manipulator IK to solve
 - CRInverseKinematics::setToolIndex set the tool index of the 
 associated manipulator (see CRManipulator::addTool) for which we are
 setting a desired pose.  Note that a tool must be added in the manipulator
 prior to using the IK solve.
 - CRInverseKinematics::setEulerMode sets the Euler convention of the pose
 vector
 - CRInverseKinematics::solve solves the IK problem described above for
 an initial configuration and a desired pose vector.  The method is overloaded
 to allow for solving for a reduced pose vector (see CRFrame::getPose)
 
 Optimization parameters can be set:
 - CRInverseKinematics::setTolerance sets the convergence tolerance of
 the algorithm.  The tolerance is compared to the 2-norm of the pose error.
 (Default = 0.001)
 - CRInverseKinematics::setMaxIter sets the maximum number of iterations the
 solver can run prior to stopping. (Default = 1)
 - CRInverseKinematics::setStepSize sets the convergence gain \f$\gamma\f$ 
 in the above equation. (Default = 0.1)
 - CRInverseKinematics::setDampingFactor sets the damping factor in the
 approximation of Jinv.  High damping is more immune to singularities (Default = 0)
 - CRInverseKinematics::setSingularThresh sets the threshold for numerically
 determining if the Jacobian is singular by comparing singular values of
 the SVD to the threshold. (Default = 0.1)
 
 
 ## Example
 This example demonstrates use of the CRInverseKinematics class.
 \code
 
 #include <iostream>
 #include "CoreRobotics.hpp"
 
 // Use the CoreRobotics namespace
 using namespace CoreRobotics;
 
 
 // -------------------------------------------------------------
 void main(void) {
 
     std::cout << "*************************************\n";
     std::cout << "Demonstration of CRInverseKinematics.\n";
     std::cout << std::fixed; std::cout.precision(4);
     
     // Set the Euler convention we will use throughout the example
     // Although CoreRobotics offers the flexibility to choose a
     // different convention for each method, in general it is good
     // to adopt the same convention throughout a problem for
     // consistency.
     CREulerMode convention = CR_EULER_MODE_XYZ;
     
     // ------------------------------------------
     // Create the robot
     
     // create several rigid body links
     CRFrameEuler* F0 = new CRFrameEuler(0, 0, 0, 0, 0, 0,
     convention,
     CR_EULER_FREE_ANG_G);
     CRFrameEuler* F1 = new CRFrameEuler(1, 0, 0, 0, 0, 0,
     convention,
     CR_EULER_FREE_ANG_G);
     CRFrameEuler* F2 = new CRFrameEuler(2, 0, 0, 0, 0, 0,
     convention,
     CR_EULER_FREE_ANG_G);
     CRFrameEuler* F3 = new CRFrameEuler(1, 0, 0, 0, 0, 0,
     convention,
     CR_EULER_FREE_NONE);
     CRRigidBody* Link0 = new CRRigidBody(F0);
     CRRigidBody* Link1 = new CRRigidBody(F1);
     CRRigidBody* Link2 = new CRRigidBody(F2);
     CRRigidBody* Link3 = new CRRigidBody(F3);
     
     // Create a new robot & add the links
     CRManipulator* MyRobot = new CRManipulator();
     
     MyRobot->addLink(Link0);
     MyRobot->addLink(Link1);
     MyRobot->addLink(Link2);
     int attachLink = MyRobot->addLink(Link3);
     
     
     // create a tool frame and add to MyRobot
     CRFrameEuler* Tool = new CRFrameEuler(0, 0, 0, 0, 0, 0,
     convention,
     CR_EULER_FREE_NONE);
     int toolIndex = MyRobot->addTool(attachLink, Tool);
     
     
     // Set up an inverse kinematics object and attach the robot
     CRInverseKinematics ikSolver = CRInverseKinematics(MyRobot,
                                                        toolIndex,
                                                        convention);
 
     // Change the maximum iterations
     ikSolver.setMaxIter(100);
 
     // Set up some variables we will use
     Eigen::VectorXd q0(3);          // initial configuration
     Eigen::VectorXd qSolved(3);     // configuration that solves p = fk(qSolved)
     Eigen::Matrix<double, 6, 1> p;  // tool set point pose
     Eigen::MatrixXd fk;             // forward kinematics (for testing the result)
     
     // Set the initial configuration of the robot
     q0 << 0.1, -0.2, 0.0;
     MyRobot->setConfiguration(q0);
     
     // Define a set point pose
     p << 2.5, 0, 0, 0, 0, 0;
     
     
     // Now solve the inverse kinematics for the point
     bool result = ikSolver.solve(p, q0, qSolved);
     
     if ( result ){
         printf("Non-sinular solution found!\n");
         std::cout << qSolved << std::endl;
         
         // Now push the new joints through the robot to see if it worked
         MyRobot->setConfiguration(qSolved);
         MyRobot->getForwardKinematics(fk);
         
         std::cout << "The forward kinematics for this solution are:\n";
         std::cout << fk << std::endl;
     
     } else {
         std::cout << "The solution is singular.\n";
         std::cout << qSolved << std::endl;
     }
 
 }
 
 \endcode
 
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
class CRInverseKinematics {
    
//---------------------------------------------------------------------
// Constructor and Destructor
public:
    
    //! Class constructor
    CRInverseKinematics(CRManipulator* i_robot,
                        unsigned int i_toolIndex,
                        CREulerMode i_eulerMode);
    
//---------------------------------------------------------------------
// Get/Set Methods
public:
    
    //! Set the robot to be used
    void setRobot(CRManipulator* i_robot) {this->m_robot = i_robot;}
    
    //! Set robot tool index to use to compute the IK.  Note that a tool must be specified in the robot.
    void setToolIndex(unsigned int i_toolIndex) {this->m_toolIndex = i_toolIndex;}
    
    //! Set the Euler angle convention of the IK solver.  This must match the
    //  Euler convention used to supply the set point in the solve() method.
    void setEulerMode(CREulerMode i_eulerMode) {this->m_eulerMode = i_eulerMode;}
    
    //! Get the Euler convention
    CREulerMode getEulerMode(void) {return this->m_eulerMode;}
    
    //! Set the algorithm convergence tolerance
    void setTolerance(double i_tolerance) {this->m_tolerance = i_tolerance;}
    
    //! Get the algorithm convergence tolerance
    double setTolerance(void) {return this->m_tolerance;}
    
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
    
    
//---------------------------------------------------------------------
// Public Methods
public:
    
    //! Solve for the joint angles (q) that yield the desired setPoint
    CRResult solve(const Eigen::Matrix<double, 6, 1>& i_setPoint,
				   Eigen::VectorXd i_q0,
				   Eigen::VectorXd &o_qSolved);
    
    CRResult solve(Eigen::VectorXd& i_setPoint,
				   Eigen::Matrix<bool, 6, 1> i_poseElements,
				   Eigen::VectorXd i_q0,
				   Eigen::VectorXd &o_qSolved);
    
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

	//! Damping term (DLS)
	double m_dampingFactor;
    
    //! Tolerance for computing if a matrix is singular using SVD svals
    double m_svdTol;
    
    
};

//=====================================================================
// End namespace
}


#endif
