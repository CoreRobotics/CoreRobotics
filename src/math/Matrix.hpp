/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#ifndef CR_MATRIX_HPP_
#define CR_MATRIX_HPP_

#include "Eigen/Dense"
#include "core/Types.hpp"

namespace cr {
namespace math {

//------------------------------------------------------------------------------
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
//------------------------------------------------------------------------------
class Matrix {

  //! Matrix downselection
public:
  //! Return sub vector of indicated indices
  static const Eigen::VectorXd reducedVector(const Eigen::VectorXd& i_x,
                                             const Eigen::VectorXi& i_indices);

  //! Return sub matrix of indicated indices
  static const Eigen::MatrixXd reducedMatrix(const Eigen::MatrixXd& i_x,
                                             const Eigen::VectorXi& i_rowIndices,
                                             const Eigen::VectorXi& i_colIndices);

  //! Matrix factorization
public:
  //! Singular Value decomposition (SVD)
  static core::Result svd(const Eigen::MatrixXd& i_A, 
                          double i_tol,
                          Eigen::MatrixXd &o_U, 
                          Eigen::VectorXd &o_Sigma,
                          Eigen::MatrixXd &o_V);

  //! Matrix inversion routines
public:
  //! SVD-based matrix inverse
  static core::Result svdInverse(const Eigen::MatrixXd& i_A,
                                 double i_tol,
                                 Eigen::MatrixXd &o_Ainv);

  //! Rotation matrices
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

  //! Vector norms
public:
  //! L-1 vector norm
  // http://mathworld.wolfram.com/L1-Norm.html
  static double normL1(Eigen::VectorXd x) { return x.array().abs().sum(); };

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

} // namepsace math
} // namepsace cr

#endif
