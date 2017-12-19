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

#ifndef CRMath_hpp
#define CRMath_hpp

//=====================================================================
// Includes
#include "Eigen/Dense"
#include "CRTypes.hpp"


//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics {

//=====================================================================
/*!
\file CRMath.hpp
\brief Implements utility math functions.
*/
//---------------------------------------------------------------------
/*!
\class CRMath
\ingroup math

\brief This class implements various math related helper functions.

\details
## Description
This class implements various math related helper functions.
 
- CRMath::deg2rad converts degress to radians
- CRMath::rad2deg converts radians to degrees.
- CRMath::forwardEulerStep performs forward Euler integration.
- CRMath::rungeKuttaStep performs 4th order Runga-Kutta integration.
- CRMath::svd performs singular value decomposition.
- CRMath::svdInverse performs matrix inverse using SVD.
- CRMath::rotAboutX constructs a rotation matrix for rotation about x axis.
- CRMath::rotAboutY constructs a rotation matrix for rotation about y axis.
- CRMath::rotAboutZ constructs a rotation matrix for rotation about z axis.
- CRMath::normL1 takes the L1 norm of a vector.
- CRMath::normL2 takes the L2 norm of a vector.
- CRMath::normLinf takes the L-infinity norm of a vector.
- CRMath::wrapToPi wraps angles to +/- $\pi$ radians.
 
## Example
This example shows usage of the math functions.
\include test_CRMath.cpp
 
## References
[1] Kreyszig, E., Advanced Engineering Mathematics, Ed.9,
John Wiley & Sons, 2011.

*/

//=====================================================================
class CRMath {

//---------------------------------------------------------------------
// Static conversion methods
public:
    
    //! Convert angles in degrees to radians
    static double deg2rad(const double i_deg) { return M_PI * i_deg / 180.0; }
    
    //! Convert angles in radians to degrees
    static double rad2deg(const double i_rad) { return 180.0 * i_rad / M_PI; }
    
    
#ifndef SWIG
//---------------------------------------------------------------------
// Numerical integration routines
public:
    
    //! Forward euler integration
    static Eigen::VectorXd forwardEulerStep(Eigen::VectorXd(i_dynamicSystem)(double,
                                                                             Eigen::VectorXd,
                                                                             Eigen::VectorXd),
                                            double i_t,
                                            Eigen::VectorXd i_x,
                                            Eigen::VectorXd i_u,
                                            double i_dt);
    
    //! Runge-Kutta 4th order integration
    static Eigen::VectorXd rungeKuttaStep(Eigen::VectorXd(i_dynamicSystem)(double,
                                                                           Eigen::VectorXd,
                                                                           Eigen::VectorXd),
                                          double i_t,
                                          Eigen::VectorXd i_x,
                                          Eigen::VectorXd i_u,
                                          double i_dt);
#endif

//---------------------------------------------------------------------
// Matrix factorization
public:

	//! Singular Value decomposition (SVD)
	static CRResult svd(Eigen::MatrixXd i_A,
						double i_tol,
						Eigen::MatrixXd& o_U,
						Eigen::VectorXd& o_Sigma,
						Eigen::MatrixXd& o_V);

    
//---------------------------------------------------------------------
// Matrix inversion routines
public:
    
    //! SVD-based matrix inverse
    static CRResult svdInverse(Eigen::MatrixXd i_A,
                               double i_tol,
                               Eigen::MatrixXd& o_Ainv);


//---------------------------------------------------------------------
// Rotation matrices
public:

	//! standard rotation about the x axis
	static Eigen::Matrix3d rotAboutX(double i_ang) {
		Eigen::Matrix3d o_rot;
		o_rot << 1, 0, 0, 0, cos(i_ang), -sin(i_ang), 0, sin(i_ang), cos(i_ang);
		return o_rot;
	}

	//! standard rotation about the y axis
	static Eigen::Matrix3d rotAboutY(double i_ang) {
		Eigen::Matrix3d o_rot;
		o_rot << cos(i_ang), 0, sin(i_ang), 0, 1, 0, -sin(i_ang), 0, cos(i_ang);
		return o_rot;
	}

	//! standard rotation about the z axis
	static Eigen::Matrix3d rotAboutZ(double i_ang) {
		Eigen::Matrix3d o_rot;
		o_rot << cos(i_ang), -sin(i_ang), 0, sin(i_ang), cos(i_ang), 0, 0, 0, 1;
		return o_rot;
	}

//---------------------------------------------------------------------
// Vector norms
public:

	//! L-1 vector norm
	// http://mathworld.wolfram.com/L1-Norm.html
	static double normL1(Eigen::VectorXd x) {
		return x.array().abs().sum();
	};

	//! L-2 vector norm
	// http://mathworld.wolfram.com/L2-Norm.html
	static double normL2(Eigen::VectorXd x) {
		return sqrt(x.array().square().sum());
	};

	//! L-infinity vector norm
	// http://mathworld.wolfram.com/L-Infinity-Norm.html
	static double normLinf(Eigen::VectorXd x) {
		return x.array().abs().maxCoeff();
	};

	//! Wrap angle (rad) to +/- pi
	static Eigen::VectorXd wrapToPi(Eigen::VectorXd angle) {
		for (int i = 0; i < angle.size(); i++) {
			angle(i) = wrapToPi(angle(i));
		}
		return angle;
	}

	//! Wrap angle (rad) to +/- pi
	static double wrapToPi(double angle) {
		angle = fmod(angle + M_PI, 2 * M_PI);
		if (angle < 0) {
			angle += 2 * M_PI;
		}
		return angle - M_PI;
	}
    
};


//=====================================================================
// End namespace
}

#endif /* CRMath_hpp */
