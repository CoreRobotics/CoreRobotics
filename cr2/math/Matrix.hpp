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

#ifndef Matrix_hpp
#define Matrix_hpp

//=====================================================================
// Includes
#include "Eigen/Dense"
#include "core/Types.hpp"


//=====================================================================
// CoreRobotics namespace
namespace cr {

//=====================================================================
/*!
\file Matrix.hpp
\brief Implements matrix maths.
*/
//---------------------------------------------------------------------
/*!
\class Matrix
\ingroup math

\brief This class implements matrix and vector math.

\details
## Description
This class implements matrix and vector math.
 
 - Matrix::reducedVector returns a sub vector
 - Matrix::reducedMatrix returns a sub matrix
 - Matrix::svd performs singular value decomposition.
 - Matrix::svdInverse performs matrix inverse using SVD.
 - Matrix::rotAboutX constructs a rotation matrix for rotation about x axis.
 - Matrix::rotAboutY constructs a rotation matrix for rotation about y axis.
 - Matrix::rotAboutZ constructs a rotation matrix for rotation about z axis.
 - Matrix::normL1 takes the L1 norm of a vector.
 - Matrix::normL2 takes the L2 norm of a vector.
 - Matrix::normLinf takes the L-infinity norm of a vector.
 
## Example
This example shows usage of the math functions.
\include example_CRMath.cpp
 
## References
[1] Kreyszig, E., Advanced Engineering Mathematics, Ed.9,
John Wiley & Sons, 2011.

*/

//=====================================================================
class Matrix {
    
//---------------------------------------------------------------------
// Matrix downselection
public:

    //! Return sub vector of indicated indices
    static Eigen::VectorXd reducedVector(Eigen::VectorXd i_x,
                                         Eigen::VectorXi i_indices);
    
    //! Return sub matrix of indicated indices
    static Eigen::MatrixXd reducedMatrix(Eigen::MatrixXd i_x,
                                         Eigen::VectorXi i_rowIndices,
                                         Eigen::VectorXi i_colIndices);
    
//---------------------------------------------------------------------
// Matrix factorization
public:

	//! Singular Value decomposition (SVD)
	static core::Result svd(Eigen::MatrixXd i_A,
                            double i_tol,
                            Eigen::MatrixXd& o_U,
                            Eigen::VectorXd& o_Sigma,
                            Eigen::MatrixXd& o_V);

    
//---------------------------------------------------------------------
// Matrix inversion routines
public:
    
    //! SVD-based matrix inverse
    static core::Result svdInverse(Eigen::MatrixXd i_A,
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
    
};


//=====================================================================
// End namespace
}

#endif /* CRMath_hpp */
