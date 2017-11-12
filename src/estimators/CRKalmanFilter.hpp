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
 \brief Implements continuous and discrete time Kalman filters.
 */
//---------------------------------------------------------------------
/*!
 \class CRKalmanFilter
 \ingroup estimators
 
 \brief This class provides methods for solving continuous and discrete
        time state estimation problems using a kalman filter.
 
 \details
 ## Description

 CRKalman filter is a module with implements the standard discrete and
 continuous time Kalman filters. This module expects a continuous time
 system in the form,

 \f$ \dot{x}=Ax+Bu+v \f$

 \f$ z=Cx+w \f$,

 where \f$x\f$ is the state vector, \f$z\f$ is a measurement, and \f$v\f$
 and \f$w\f$ are zero mean gaussian random vectors with covariance matricies
 \f$Q\f$ and \f$R\f$ respectively.

 In discrete time the modules expects a system,

 \f$ x_{t+1}=Ax_t+Bu_t+v \f$

 \f$ z_t=Cx_t \f$,

 where all the variable are defined in the same way as the continuous time
 version.

 Once the sovler is initialized it can stepped forward in time by providing
 an input and measurement vector to the function,
 - CRKalmanFilter::step

 One optimization parameter is available.
 - CRKalmanFilter::setTolerance sets the tolerance used for the SVD based matrix inversion
 
 
 ## Example

 This example demonstrates use of the CRKalmanFilter class.
 \include test_CRKalmanFilter.cpp
 
 ## References

 [1] Wikipedia: "Kalman filter",
 https://en.wikipedia.org/wiki/Kalman_filter 2017.\n\n
 
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
