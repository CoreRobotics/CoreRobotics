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

// declare an inverse cumulative distribution - this is the invserse
// CDF for a triangular distribution from [0,1].
Eigen::VectorXd myInvCDF(double P) {
  Eigen::VectorXd v(1);
  v(0) = sqrt(P);
  return v;
}

// declare the probability density - this is the traditional density
// defined by a distribution
double myPdensity(Eigen::VectorXd x) { return 2 * x(0); }

//
// Sample from the distribution and check the histogram
//
TEST(NoiseModel, Sample) {

  // initialize a noise model
  NoiseModel genericNoise = NoiseModel();
  genericNoise.setParameters(*myInvCDF, *myPdensity);

  // initialize parameters for experiments
  const int n = 1000000;     // number of experiments
  const int nintervals = 10; // number of intervals
  int p[nintervals] = {};

  // sample the distribution
  for (int i = 0; i < n; ++i) {
    Eigen::VectorXd v = genericNoise.sample();
    ++p[int(nintervals * v(0))];
  }

  // compute the expected distribution for each histogram bin
  double prob[10] = {};
  for (int k = 0; k < nintervals; k++) {
    double csum = 0;

    // integrate 100 steps between k and k+1
    Eigen::VectorXd point(1);
    for (int j = 0; j < 100; ++j) {
      point << double(k) + double(j) / 100.0;
      csum += genericNoise.probability(point) / 100.0;
    }
    // compute the probability
    prob[k] = csum * (double(n) / 100.0);
  }

  // We expect the histogram to be close to the binned pdf (1%)
  EXPECT_NEAR(prob[0], double(p[0]), 0.01 * double(n));
  EXPECT_NEAR(prob[1], double(p[1]), 0.01 * double(n));
  EXPECT_NEAR(prob[2], double(p[2]), 0.01 * double(n));
  EXPECT_NEAR(prob[3], double(p[3]), 0.01 * double(n));
  EXPECT_NEAR(prob[4], double(p[4]), 0.01 * double(n));
  EXPECT_NEAR(prob[5], double(p[5]), 0.01 * double(n));
  EXPECT_NEAR(prob[6], double(p[6]), 0.01 * double(n));
  EXPECT_NEAR(prob[7], double(p[7]), 0.01 * double(n));
  EXPECT_NEAR(prob[8], double(p[8]), 0.01 * double(n));
  EXPECT_NEAR(prob[9], double(p[9]), 0.01 * double(n));
}
