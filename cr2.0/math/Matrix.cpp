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

#include "Matrix.hpp"
#include "Eigen/Dense"
#include "Eigen/SVD"

//=====================================================================
// CoreRobotics namespace
namespace cr {
    
    
    
//=====================================================================
/*!
 This method returns a reduced vector.\n.
 
 \param [in] i_x - the input vector
 \param [out] i_indices - a vector of indices to return.  Note that the
 size of i_indices corresponds to the size of the output vector.
 \return - the reduced vector
 */
//---------------------------------------------------------------------
Eigen::VectorXd Matrix::reducedVector(Eigen::VectorXd i_x,
                                        Eigen::VectorXi i_indices)
{
    Eigen::VectorXd y;
    y.setZero(i_indices.size());
    
    // now perform the down selection
    for (int k = 0; k < i_indices.size(); k++){
        y(k) = i_x(i_indices(k));
    }
    return y;
}

    

//=====================================================================
/*!
 This method returns a reduced matrix.\n.
 
 \param [in] i_x - the input matrix
 \param [out] i_rowIndices - a vector of rows indices to return.  Note
 that the size of i_rowIndices corresponds to the number of rows in the
 output matrix.
 \param [out] i_colIndices - a vector of column indices to return.  Note
 that the size of i_colIndices corresponds to the number of columns in the
 output matrix.
 \return - the reduced matrix
 */
//---------------------------------------------------------------------
Eigen::MatrixXd Matrix::reducedMatrix(Eigen::MatrixXd i_x,
                                        Eigen::VectorXi i_rowIndices,
                                        Eigen::VectorXi i_colIndices)
{
    Eigen::MatrixXd y;
    y.setZero(i_rowIndices.size(), i_colIndices.size());
    
    // now perform the down selection
    for (int i = 0; i < i_rowIndices.size(); i++){
        for (int j = 0; j < i_colIndices.size(); j++){
            y(i, j) = i_x(i_rowIndices(i), i_colIndices(j));
        }
    }
    return y;
}



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
\return - a result flag (see: Types::Result) indicating if any of the
singular values were below the specified tolerance (i.e. singular)

*/
//---------------------------------------------------------------------
core::Result Matrix::svd(Eigen::MatrixXd i_A,
                         double i_tol,
                         Eigen::MatrixXd& o_U,
                         Eigen::VectorXd& o_Sigma,
                         Eigen::MatrixXd& o_V)
{
	core::Result result = core::CR_RESULT_SUCCESS;

	// Compute the SVD of A
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(i_A, Eigen::ComputeThinU | Eigen::ComputeThinV);

	// Take the SVD
	Eigen::VectorXd sVals = svd.singularValues();

	// check for singular condition
	for (int i = 0; i < sVals.size(); i++) {
		if (sVals(i) <= i_tol) {
			result = core::CR_RESULT_SINGULAR;
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
 \return - a result flag (see: Types::Result) indicating if the operation was successful
 or if a singularity was encountered in the operation.
 
 */
//---------------------------------------------------------------------
core::Result Matrix::svdInverse(Eigen::MatrixXd i_A,
                                double i_tol,
                                Eigen::MatrixXd& o_Ainv)
{
    core::Result result = core::CR_RESULT_SUCCESS;
    
    // Compute the SVD of A
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(i_A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    
    // Take the SVD
    Eigen::VectorXd sVals = svd.singularValues();
    
    // check for singular condition
    for (int i = 0; i < sVals.size(); i++){
        if (sVals(i) <= i_tol){
            result = core::CR_RESULT_SINGULAR;
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


