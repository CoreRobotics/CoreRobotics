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
CRMath implements math static helper functions.
 
 
## References
[1] Kreyszig, E., Advanced Engineering Mathematics, Ed.9,
John Wiley & Sons, 2011.

*/

//=====================================================================

//! CoreRobotics value for PI
const double CR_PI = 3.1415926535897932384626433832795;

//=====================================================================
class CRMath {

//---------------------------------------------------------------------
// Static conversion methods
public:
    
    //! Convert angles in degrees to radians
    static double deg2rad(const double i_deg) { return CR_PI * i_deg / 180.0; }
    
    //! Convert angles in radians to degrees
    static double rad2deg(const double i_rad) { return 180.0 * i_rad / CR_PI; }
    
    
//---------------------------------------------------------------------
// Numerical integration routines
public:
    
    //! Forward euler integration
    static Eigen::VectorXd forwardEulerStep(Eigen::VectorXd(i_dyanmicSystem)(double,
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
    
    
};


//=====================================================================
// End namespace
}

#endif /* CRMath_hpp */
