//=====================================================================
/*
Software License Agreement (BSD-3-Clause License)
Copyright (c) 2019, CoreRobotics.
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
#include "CoreRobotics.hpp"
#include "gtest/gtest.h"
#include <iostream>

// Use the CoreRobotics namespace
using namespace CoreRobotics;

CRNoiseGaussian *dynNoise1;
CRNoiseGaussian *dynNoise2;

// Declare a probabilistic continuous motion model - xdot = fcn(t,x,u,s)
Eigen::VectorXd probContinuousDynFcn(double t, Eigen::VectorXd x,
                                     Eigen::VectorXd u, bool sample) {
  Eigen::VectorXd w(1);
  if (sample) {
    w = dynNoise1->sample();
  } else {
    w << 0;
  }
  return -x + u + w; // motion
}

// Declare a probabilistic discrete motion model - xNext = fcn(t,x,u,s)
Eigen::VectorXd probDiscreteDynFcn(double t, Eigen::VectorXd x,
                                   Eigen::VectorXd u, bool sample) {
  Eigen::VectorXd w(1);
  if (sample) {
    w = dynNoise2->sample();
  } else {
    w << 0;
  }
  x(0) = exp(-1 * 0.01) * x(0) -
         1 * (exp(-1 * 0.01) - 1) * 1 * (u(0) + w(0)); // motion (dt = 0.01)
  return x;
}

//
// Test the continuous model.  Here we specify a continuous differential model,
// which is solved using the internal integration scheme of the CRMotionLinear
// model
//
TEST(CRMotionProbabilistic, Continuous) {
  double dt = 0.01;          // sample rate (seconds)
  Eigen::VectorXd x(1);      // state vector
  Eigen::VectorXd xb(1);     // baseline state vector
  Eigen::VectorXd u(1);      // input vector
  Eigen::MatrixXd cov(1, 1); // covariance
  Eigen::VectorXd mean(1);   // mean
  x << 1;                    // initial condition
  xb << 1;                   // I.C.
  u << 0;                    // input value
  cov << 1;                  // noise cov
  mean << 0;                 // noise mean
  dynNoise1 = new CRNoiseGaussian(cov, mean);
  CRMotionProbabilistic model =
      CRMotionProbabilistic(*probContinuousDynFcn, CR_MOTION_CONTINUOUS, x, dt);
  CRMotionProbabilistic modelBaseline =
      model;      // init another model to test that noise is sampled
  double t = 0;   // Initialize a time t
  while (t < 5) { // simulate for 5 seconds
    x = model.motion(u, true);           // simulate the motion
    xb = modelBaseline.motion(u, false); // without noise
    t = model.getTime();                 // get the simulation time
  }
  EXPECT_NEAR(5, t, dt);  // check that the simulation integrated properly
  EXPECT_NE(xb(0), x(0)); // check that noise was added to the final state
}

//
// Test the discrete model.  Here we discretize the same system using exact
// discretization:
// https://en.wikipedia.org/wiki/Discretization#Discretization_of_linear_state_space_models
// this approach gives us full control over how the system is discretized
//
TEST(CRMotionProbabilistic, Discrete) {
  double dt = 0.01;          // sample rate (seconds)
  Eigen::VectorXd x(1);      // state vector
  Eigen::VectorXd xb(1);     // baseline state vector
  Eigen::VectorXd u(1);      // input vector
  Eigen::MatrixXd cov(1, 1); // covariance
  Eigen::VectorXd mean(1);   // mean
  x << 1;                    // initial condition
  xb << 1;                   // I.C.
  u << 0;                    // input value
  cov << 1;                  // noise cov
  mean << 0;                 // noise mean
  dynNoise2 = new CRNoiseGaussian(cov, mean);
  CRMotionProbabilistic model =
      CRMotionProbabilistic(*probContinuousDynFcn, CR_MOTION_CONTINUOUS, x, dt);
  CRMotionProbabilistic modelBaseline =
      model;      // init another model to test that noise is sampled
  double t = 0;   // Initialize a time t
  while (t < 5) { // simulate for 5 seconds
    x = model.motion(u, true);           // simulate the motion
    xb = modelBaseline.motion(u, false); // without noise
    t = model.getTime();                 // get the simulation time
  }
  EXPECT_NEAR(5, t, dt);  // check that the simulation integrated properly
  EXPECT_NE(xb(0), x(0)); // check that noise was added to the final state
}
