/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#include "gtest/gtest.h"
#include <cr/noise>
#include <iostream>

// Use the CoreRobotics namespace
using namespace cr::noise;

//
// Sample from the distribution and check the histogram
//
TEST(Gmm, Predict) {

  // set up the GMM parameters
  GmmParameters gmmParams;
  GaussianParameters gaussianParams(4);

  // Add a couple Gaussians
  gaussianParams.cov.diagonal() << 1.0, 2.0, 0.1, 4.0;
  gaussianParams.mean << 1, 2, 3, 4;
  Gaussian g1(gaussianParams);
  gmmParams.add(&g1, 0.5);

  gaussianParams.cov.diagonal() << 0.1, 0.2, 4, 5;
  gaussianParams.mean << 5, 6, 7, 8;
  Gaussian g2(gaussianParams);
  gmmParams.add(&g2, 0.3);

  // cov.diagonal()  << 0.1, 0.2, 4, 5;
  gaussianParams.cov << 0.1, 0, 0, 0.2, 0, 0.2, 0, 0, 0, 0, 4.0, -0.1, 0.2, 0,
      -0.1, 5.0;
  gaussianParams.mean << 9, 10, 11, 12;
  Gaussian g3(gaussianParams);
  gmmParams.add(&g3, 0.2);

  // initialize a noise mixture model
  Gmm gmm = Gmm(gmmParams);

  // Construct the input/output booleans
  Eigen::VectorXi inputs(1);
  Eigen::VectorXi outputs(2);
  inputs << 0;
  outputs << 2, 3;

  // Predict
  Eigen::VectorXd x(1);
  x << 7.2;
  Eigen::VectorXd y_mu(2);
  Eigen::MatrixXd y_cov(2, 2);
  gmm.regression(x, inputs, outputs, y_mu, y_cov);

  // We computed the above problem with an existing GMM/GMR library in MATLAB
  // and compare the evaluations here
  double tol = 1e-12;
  EXPECT_NEAR(10.701026290884815, y_mu(0), tol);
  EXPECT_NEAR(8.236436140374021, y_mu(1), tol);
  EXPECT_NEAR(3.704887227956277, y_cov(0, 0), tol);
  EXPECT_NEAR(-0.092618710733151, y_cov(0, 1), tol);
  EXPECT_NEAR(-0.092618710733151, y_cov(1, 0), tol);
  EXPECT_NEAR(4.265976289199881, y_cov(1, 1), tol);
}
