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

#include "CRMatrix.hpp"
#include "Eigen/Dense"
#include "Eigen/SVD"


//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics {



//=====================================================================
/*!
This method returns the SVD of a matrix\n.

For a matrix A, the SVD yields:

\f[
A = U \Sigma V^*
\f]

The method utilizes the Jacobi SVD.  For large matrices, this will be very slow.

\param [in] i_A - the matrix to be decomposed
\param [out] i_tol - the tolerance placed on the singular values to determine if the matrix is singular
\param [out] o_U - the U matrix in the factorization
\param [out] o_Sigma - the vector of singular values
\param [out] o_V - the V matrix in the factorization
\return - a result flag (see: CRTypes::CRResult) indicating if any of the
singular values were below the specified tolerance (i.e. singular)

*/
//---------------------------------------------------------------------
CRResult CRMatrix::svd(Eigen::MatrixXd i_A,
					 double i_tol,
					 Eigen::MatrixXd& o_U,
					 Eigen::VectorXd& o_Sigma,
					 Eigen::MatrixXd& o_V)
{
	CRResult result = CR_RESULT_SUCCESS;

	// Compute the SVD of A
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(i_A, Eigen::ComputeThinU | Eigen::ComputeThinV);

	// Take the SVD
	Eigen::VectorXd sVals = svd.singularValues();

	// check for singular condition
	for (int i = 0; i < sVals.size(); i++) {
		if (sVals(i) <= i_tol) {
			result = CR_RESULT_SINGULAR;
			break;
		}
	}

	// Return SVD matrices
	o_U = svd.matrixU();
	o_Sigma = sVals.array();
	o_V = svd.matrixV();

	// return the result
	return result;
}

    
    
//=====================================================================
/*!
 This method performs generalized matrix inversion using the SVD method\n.
 
 For a matrix A, the SVD yields:
 
 \f[
 A = U \Sigma V^*
 \f]
 
 The generalized inverse is then:
 
 \f[
 A^\dagger = V \Sigma^{-1} U^*
 \f]
 
 The method utilizes the Jacobi SVD for the inverse.  For large matrices,
 this will be very slow.
 
 \param [in] i_A - the matrix to invert
 \param [in] i_tol - the tolerance placed on the singular values to determine if the matrix is singular
 \param [out] o_Ainv - the generalized inverse of Matrix A (\f$A^\dagger\f$)
 \return - a result flag (see: CRTypes::CRResult) indicating if the operation was successful
 or if a singularity was encountered in the operation.
 
 */
//---------------------------------------------------------------------
CRResult CRMatrix::svdInverse(Eigen::MatrixXd i_A, double i_tol, Eigen::MatrixXd& o_Ainv)
{
    CRResult result = CR_RESULT_SUCCESS;
    
    // Compute the SVD of A
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(i_A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    
    // Take the SVD
    Eigen::VectorXd sVals = svd.singularValues();
    
    // check for singular condition
    for (int i = 0; i < sVals.size(); i++){
        if (sVals(i) <= i_tol){
            result = CR_RESULT_SINGULAR;
            break;
        }
    }
    
    // Compute the generalized inverse using SVD
    Eigen::VectorXd sValsInverse = sVals.array().inverse();
    Eigen::MatrixXd SigmaInv = sValsInverse.asDiagonal();
    o_Ainv = svd.matrixV() * SigmaInv * svd.matrixU().transpose();
    
    // return the result
    return result;
}


//=====================================================================
// End namespace
}


