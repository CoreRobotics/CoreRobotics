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

#ifndef CRKalmanFilter_hpp
#define CRKalmanFilter_hpp

//=====================================================================
// Includes
#include "CRTypes.hpp"
#include "CRMath.hpp"
#include "Eigen/Dense"
#include <cmath>

//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics {

//=====================================================================
/*!
 \file CRKalmanFilter.hpp
 \brief Implements a discrete time Kalman filter.
 */
//---------------------------------------------------------------------
/*!
 \class CRKalmanFilter
 \ingroup estimators
 
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
 \include test_CRHardLimits.cpp
 
 ## References

 [1] F. Flacco, A. De Luca, and O. Khatib, "Control of Redundant Robots Under Hard Joint Constraints: Saturation in the Null Space", IEEE Transactions on Robotics, 2015.\n\n
 
 */
//=====================================================================
class CRKalmanFilter {

//---------------------------------------------------------------------
// Constructor and Destructor
public:

	//! Class constructor
	CRKalmanFilter(Eigen::MatrixXd i_A,
	               Eigen::MatrixXd i_B,
                   Eigen::MatrixXd i_C,
                   Eigen::MatrixXd i_Q,
                   Eigen::MatrixXd i_R,
                   Eigen::VectorXd i_x0,
                   Eigen::MatrixXd i_Sigma0) {
		this->m_A = i_A;
		this->m_B = i_B;
		this->m_C = i_C;
		this->m_Q = i_Q;
		this->m_R = i_R;
		this->m_x = i_x0;
		this->m_Sigma = i_Sigma0;
		this->m_tol = 0.001;
		this->m_dt = NAN;
	}

	CRKalmanFilter(Eigen::MatrixXd i_A,
	               Eigen::MatrixXd i_B,
                   Eigen::MatrixXd i_C,
                   Eigen::MatrixXd i_Q,
                   Eigen::MatrixXd i_R,
                   Eigen::VectorXd i_x0,
                   Eigen::MatrixXd i_Sigma0,
                   double i_dt) {
		this->m_A = i_A;
		this->m_B = i_B;
		this->m_C = i_C;
		this->m_Q = i_Q;
		this->m_R = i_R;
		this->m_x = i_x0;
		this->m_Sigma = i_Sigma0;
		this->m_tol = 0.001;
		this->m_dt = i_dt;
		CRMath::svdInverse(this->m_R, this->m_tol, this->m_Rinv);
	}

//---------------------------------------------------------------------
// Get/Set Methods
public:

	//! Get the A matrix
	Eigen::MatrixXd getA(void) {return this->m_A;}

	//! Set the A matrix
	void setA(Eigen::MatrixXd i_A) {this->m_A = i_A;}

	//! Get the B matrix
	Eigen::MatrixXd getB(void) {return this->m_B;}

	//! Set the B matrix
	void setB(Eigen::MatrixXd i_B) {this->m_B = i_B;}

	//! Get the C matrix
	Eigen::MatrixXd getC(void) {return this->m_C;}

	//! Set the C matrix
	void setC(Eigen::MatrixXd i_C) {this->m_C = i_C;}

	//! Get the Q matrix
	Eigen::MatrixXd getQ(void) {return this->m_Q;}

	//! Set the Q matrix
	void setQ(Eigen::MatrixXd i_Q) {this->m_Q = i_Q;}

	//! Get the R matrix
	Eigen::MatrixXd getR(void) {return this->m_R;}

	//! Set the R matrix
	CRResult setR(Eigen::MatrixXd i_R) {
		this->m_R = i_R;
		if (std::isnan(this->m_dt)) {
			return CR_RESULT_SUCCESS;
		} else {
			return CRMath::svdInverse(this->m_R, this->m_tol, this->m_Rinv);
		}
	}

	//! Get the state vector
	Eigen::VectorXd getState(void) {return this->m_x;}

	//! Get the covariance matrix
	Eigen::MatrixXd getCovariance(void) {return this->m_Sigma;}

	//! Get svd inverse tolerance
	double getTolerance(void) {return this->m_tol;}

	//! Set the svd inverse tolerance
	void setTolerance(double i_tol) {this->m_tol = i_tol;}

	//! Get the step size
	double getStepSize(void) {return this->m_dt;}

	//! Set the step size
	void setStepSize(double i_dt) {this->m_dt = i_dt;}

//---------------------------------------------------------------------
// Solve Methods
public:

	//! Run one step of the Kalman filter with the given input and measurement
	CRResult step(Eigen::VectorXd i_input,
                  Eigen::VectorXd i_measurement);

//---------------------------------------------------------------------
// Protected Members
protected:

	//! The A matrix
	Eigen::MatrixXd m_A;

	//! The B matrix
	Eigen::MatrixXd m_B;

	//! The C matrix
	Eigen::MatrixXd m_C;

	//! The Q matrix
	Eigen::MatrixXd m_Q;

	//! The R matrix
	Eigen::MatrixXd m_R;

	//! The inverse of the R matrix
	Eigen::MatrixXd m_Rinv;

	//! The state vector
	Eigen::VectorXd m_x;

	//! The covariance matrix
	Eigen::MatrixXd m_Sigma;

	//! The svd inverse tolerance
	double m_tol;

	//! The time step
	double m_dt;
};

//=====================================================================
// End namespace
}

#endif
