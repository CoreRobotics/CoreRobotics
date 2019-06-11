/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#include "Matrix.hpp"
#include "Eigen/Dense"
#include "Eigen/SVD"

namespace cr {
namespace math {

//------------------------------------------------------------------------------
/*!
 This method returns a reduced vector.\n.

 \param [in] i_x - the input vector
 \param [out] i_indices - a vector of indices to return.  Note that the
 size of i_indices corresponds to the size of the output vector.
 \return - the reduced vector
 */
//------------------------------------------------------------------------------
const Eigen::VectorXd Matrix::reducedVector(const Eigen::VectorXd& i_x,
                                            const Eigen::VectorXi& i_indices) {
  Eigen::VectorXd y;
  y.setZero(i_indices.size());

  // now perform the down selection
  for (int k = 0; k < i_indices.size(); k++) {
    y(k) = i_x(i_indices(k));
  }
  return y;
}

//------------------------------------------------------------------------------
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
//------------------------------------------------------------------------------
const Eigen::MatrixXd Matrix::reducedMatrix(const Eigen::MatrixXd& i_x,
                                            const Eigen::VectorXi& i_rowIndices,
                                            const Eigen::VectorXi& i_colIndices) {
  Eigen::MatrixXd y;
  y.setZero(i_rowIndices.size(), i_colIndices.size());

  // now perform the down selection
  for (int i = 0; i < i_rowIndices.size(); i++) {
    for (int j = 0; j < i_colIndices.size(); j++) {
      y(i, j) = i_x(i_rowIndices(i), i_colIndices(j));
    }
  }
  return y;
}

//------------------------------------------------------------------------------
/*!
This method returns the SVD of a matrix\n.

For a matrix A, the SVD yields:

\f[
A = U \Sigma V^*
\f]

The method utilizes the Jacobi SVD.  For large matrices, this will be very slow.

\param [in] i_A - the matrix to be decomposed
\param [out] i_tol - the tolerance placed on the singular values to determine if
the matrix is singular
\param [out] o_U - the U matrix in the factorization
\param [out] o_Sigma - the vector of singular values
\param [out] o_V - the V matrix in the factorization
\return - a result flag (see: Types::Result) indicating if any of the
singular values were below the specified tolerance (i.e. singular)

*/
//------------------------------------------------------------------------------
core::Result Matrix::svd(const Eigen::MatrixXd& i_A,
                         double i_tol,
                         Eigen::MatrixXd &o_U,
                         Eigen::VectorXd &o_Sigma,
                         Eigen::MatrixXd &o_V) {
  core::Result result = core::CR_RESULT_SUCCESS;

  // Compute the SVD of A
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(i_A, Eigen::ComputeThinU |
                                                 Eigen::ComputeThinV);

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

//------------------------------------------------------------------------------
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
 \param [in] i_tol - the tolerance placed on the singular values to determine if
 the matrix is singular
 \param [out] o_Ainv - the generalized inverse of Matrix A (\f$A^\dagger\f$)
 \return - a result flag (see: Types::Result) indicating if the operation was
 successful
 or if a singularity was encountered in the operation.

 */
//------------------------------------------------------------------------------
core::Result Matrix::svdInverse(const Eigen::MatrixXd& i_A,
                                double i_tol,
                                Eigen::MatrixXd &o_Ainv) {
  core::Result result = core::CR_RESULT_SUCCESS;

  // Compute the SVD of A
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(i_A, Eigen::ComputeThinU |
                                                 Eigen::ComputeThinV);

  // Take the SVD
  Eigen::VectorXd sVals = svd.singularValues();

  // check for singular condition
  for (int i = 0; i < sVals.size(); i++) {
    if (sVals(i) <= i_tol) {
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

} // namepsace math
} // namepsace cr
