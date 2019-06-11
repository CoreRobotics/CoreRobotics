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

// Declare a continuous dynamical system - xdot = fcn(t,x,u)
// The exact solution of this system is y = exp(-t) for u = 0
Eigen::VectorXd myDynamicalSystem(double t, Eigen::VectorXd x,
                                  Eigen::VectorXd u) {
  return -x + u; // motion
}

/// Test the runge kutta method using raw function pointer
TEST(Integration, RungeKutta) {
  double t = 0;
  double dt = 0.01;     // sample rate (seconds)
  Eigen::VectorXd x(1); // state vector
  Eigen::VectorXd u(1); // input vector
  x << 1;               // initial condition
  u << 0;               // input value

  // perform constant sample rate integration
  while (t <= 5) {
    x = Integration::rungeKuttaStep(*myDynamicalSystem, t, x, u, dt);
    t += dt;
  }

  // check that we approach the final values
  EXPECT_NEAR(5, t, dt); // check that the simulation integrated properly
  EXPECT_NEAR(exp(-t), x(0), 1e-6); // check the final state to exact solution
}

/// Test the runge kutta method using std::function
TEST(Integration, RungeKuttaStd) {
  double t = 0;
  double dt = 0.01;     // sample rate (seconds)
  Eigen::VectorXd x(1); // state vector
  Eigen::VectorXd u(1); // input vector
  x << 1;               // initial condition
  u << 0;               // input value

  // free function
  std::function<Eigen::VectorXd(double, Eigen::VectorXd, Eigen::VectorXd)>
      myDynamicalSystemPtr = myDynamicalSystem;

  // perform constant sample rate integration
  while (t <= 5) {
    x = Integration::rungeKuttaStep(myDynamicalSystemPtr, t, x, u, dt);
    t += dt;
  }

  // check that we approach the final values
  EXPECT_NEAR(5, t, dt); // check that the simulation integrated properly
  EXPECT_NEAR(exp(-t), x(0), 1e-6); // check the final state to exact solution
}

/// Test the forward euler method using raw function pointer
TEST(Integration, ForwardEuler) {
  double t = 0;
  double dt = 0.01;     // sample rate (seconds)
  Eigen::VectorXd x(1); // state vector
  Eigen::VectorXd u(1); // input vector
  x << 1;               // initial condition
  u << 0;               // input value

  // perform constant sample rate integration
  while (t <= 5) {
    x = Integration::forwardEulerStep(*myDynamicalSystem, t, x, u, dt);
    t += dt;
  }

  // check that we approach the final values
  EXPECT_NEAR(5, t, dt); // check that the simulation integrated properly
  EXPECT_NEAR(exp(-t), x(0), 1e-3); // check the final state to exact solution
}

/// Test the forward euler method using std::function
TEST(Integration, ForwardEulerStd) {
  double t = 0;
  double dt = 0.01;     // sample rate (seconds)
  Eigen::VectorXd x(1); // state vector
  Eigen::VectorXd u(1); // input vector
  x << 1;               // initial condition
  u << 0;               // input value

  // free function
  std::function<Eigen::VectorXd(double, Eigen::VectorXd, Eigen::VectorXd)>
      myDynamicalSystemPtr = myDynamicalSystem;

  // perform constant sample rate integration
  while (t <= 5) {
    x = Integration::forwardEulerStep(myDynamicalSystemPtr, t, x, u, dt);
    t += dt;
  }

  // check that we approach the final values
  EXPECT_NEAR(5, t, dt); // check that the simulation integrated properly
  EXPECT_NEAR(exp(-t), x(0), 1e-3); // check the final state to exact solution
}
