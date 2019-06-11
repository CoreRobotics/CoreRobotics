/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#include "gtest/gtest.h"
#include <cr/math>
#include <iostream>

// Use the CoreRobotics namespace
using namespace cr::math;
using namespace cr::core;

//
// Test the reductions
//
TEST(Matrix, Reductions) {

  // simple vector
  Eigen::VectorXd x(3);
  x << 1, 2, 3;

  // Reduced vector
  Eigen::VectorXi idx(2);
  idx << 0, 2;
  Eigen::VectorXd y = Matrix::reducedVector(x, idx);

  EXPECT_DOUBLE_EQ(1, y(0));
  EXPECT_DOUBLE_EQ(3, y(1));
  EXPECT_EQ(2, y.size());

  // simple matrix
  Eigen::Matrix3d A;
  A << 1, 2, 3, 4, 5, 6, 7, 8, 9;

  // Reduced vector
  Eigen::VectorXi rIdx(1);
  Eigen::VectorXi cIdx(2);
  rIdx << 1;
  cIdx << 0, 2;
  Eigen::MatrixXd Y = Matrix::reducedMatrix(A, rIdx, cIdx);

  EXPECT_EQ(1, Y.rows());
  EXPECT_EQ(2, Y.cols());
  EXPECT_DOUBLE_EQ(4, Y(0, 0));
  EXPECT_DOUBLE_EQ(6, Y(0, 1));
}

//
// Test the svd
//
TEST(Matrix, SVD) {

  // simple matrix
  Eigen::Matrix2d A;
  A << 5, 0, 0, 1;

  // compute svd - low threshold for checking singularity of matrix
  Eigen::MatrixXd U, V;
  Eigen::VectorXd S;
  Result res = Matrix::svd(A, 1e-12, U, S, V);

  // svd of diagonal is easy (U, V should be identity and the
  // singular values should be the diagonal of matrix
  EXPECT_DOUBLE_EQ(1, U(0, 0));
  EXPECT_DOUBLE_EQ(0, U(0, 1));
  EXPECT_DOUBLE_EQ(0, U(1, 0));
  EXPECT_DOUBLE_EQ(1, U(1, 1));

  EXPECT_DOUBLE_EQ(1, V(0, 0));
  EXPECT_DOUBLE_EQ(0, V(0, 1));
  EXPECT_DOUBLE_EQ(0, V(1, 0));
  EXPECT_DOUBLE_EQ(1, V(1, 1));

  EXPECT_DOUBLE_EQ(5, S(0));
  EXPECT_DOUBLE_EQ(1, S(1));

  EXPECT_EQ(Result::CR_RESULT_SUCCESS, res);

  // now check that the singularity flag works to tolerance
  res = Matrix::svd(A, 1 + 1e-12, U, S, V);
  EXPECT_EQ(Result::CR_RESULT_SINGULAR, res);
}

//
// Test the svd inverse
//
TEST(Matrix, SVDInverse) {

  // simple matrix
  Eigen::Matrix2d A;
  A << 5, 0, 0, 1;

  // compute svd - low threshold for checking singularity of matrix
  Eigen::MatrixXd Ainv;
  Result res = Matrix::svdInverse(A, 1e-12, Ainv);

  // svd of diagonal is easy (U, V should be identity and the
  // singular values should be the diagonal of matrix
  EXPECT_DOUBLE_EQ(1.0 / 5.0, Ainv(0, 0));
  EXPECT_DOUBLE_EQ(0, Ainv(0, 1));
  EXPECT_DOUBLE_EQ(0, Ainv(1, 0));
  EXPECT_DOUBLE_EQ(1, Ainv(1, 1));

  EXPECT_EQ(Result::CR_RESULT_SUCCESS, res);

  // now check that the singularity flag works to tolerance
  res = Matrix::svdInverse(A, 1 + 1e-12, Ainv);
  EXPECT_EQ(Result::CR_RESULT_SINGULAR, res);
}

//
// Test the rotation matrices
//
TEST(Matrix, RotX) {

  // generate a rotation matrix
  double a = M_PI / 2;
  Eigen::Matrix3d R = Matrix::rotAboutX(a);

  // check to known values
  EXPECT_NEAR(1, R(0, 0), 1e-12);
  EXPECT_NEAR(0, R(0, 1), 1e-12);
  EXPECT_NEAR(0, R(0, 2), 1e-12);

  EXPECT_NEAR(0, R(1, 0), 1e-12);
  EXPECT_NEAR(0, R(1, 1), 1e-12);
  EXPECT_NEAR(-1, R(1, 2), 1e-12);

  EXPECT_NEAR(0, R(2, 0), 1e-12);
  EXPECT_NEAR(1, R(2, 1), 1e-12);
  EXPECT_NEAR(0, R(2, 2), 1e-12);
}

//
// Test the rotation matrices
//
TEST(Matrix, RotY) {

  // generate a rotation matrix
  double a = M_PI / 2;
  Eigen::Matrix3d R = Matrix::rotAboutY(a);

  // check to known values
  EXPECT_NEAR(0, R(0, 0), 1e-12);
  EXPECT_NEAR(0, R(0, 1), 1e-12);
  EXPECT_NEAR(1, R(0, 2), 1e-12);

  EXPECT_NEAR(0, R(1, 0), 1e-12);
  EXPECT_NEAR(1, R(1, 1), 1e-12);
  EXPECT_NEAR(0, R(1, 2), 1e-12);

  EXPECT_NEAR(-1, R(2, 0), 1e-12);
  EXPECT_NEAR(0, R(2, 1), 1e-12);
  EXPECT_NEAR(0, R(2, 2), 1e-12);
}

//
// Test the rotation matrices
//
TEST(Matrix, RotZ) {

  // generate a rotation matrix
  double a = M_PI / 2;
  Eigen::Matrix3d R = Matrix::rotAboutZ(a);

  // check to known values
  EXPECT_NEAR(0, R(0, 0), 1e-12);
  EXPECT_NEAR(-1, R(0, 1), 1e-12);
  EXPECT_NEAR(0, R(0, 2), 1e-12);

  EXPECT_NEAR(1, R(1, 0), 1e-12);
  EXPECT_NEAR(0, R(1, 1), 1e-12);
  EXPECT_NEAR(0, R(1, 2), 1e-12);

  EXPECT_NEAR(0, R(2, 0), 1e-12);
  EXPECT_NEAR(0, R(2, 1), 1e-12);
  EXPECT_NEAR(1, R(2, 2), 1e-12);
}
