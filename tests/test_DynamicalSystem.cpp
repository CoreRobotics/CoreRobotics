/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#include "gtest/gtest.h"
#include <cr/model>
#include <iostream>

// Use the CoreRobotics namespace
using namespace cr::model;

// derive a dynamic model class
class myMotionModel : public DynamicalSystem {

public:
  myMotionModel(Eigen::VectorXd x, Eigen::VectorXd u, double dt)
      : DynamicalSystem(x, u, dt) {}

  Eigen::VectorXd motionCallback(double t, Eigen::VectorXd x,
                                 Eigen::VectorXd u) override {
    return -x + u;
  }
};

//
// Test the model with explicit time stepping.
//
TEST(DynamicalSystemModel, ExplicitTime) {

  // set up the states
  double dt = 0.01; // sample rate (seconds)

  Eigen::VectorXd x(1); // state vector
  Eigen::VectorXd u(1); // input vector

  x << 1; // initial condition
  u << 0; // input value

  // initialize the model
  myMotionModel mdl(x, u, dt);

  // loop the model
  double t = 0;   // Initialize a time t
  while (t < 5) { // simulate for 5 seconds
    mdl.step();
    t = mdl.getTime();
    x = mdl.getState();
  }

  EXPECT_NEAR(5, t, dt); // check that the simulation integrated properly
  EXPECT_NEAR(0, x(0),
              0.01); // check the final state (should be within 1% of F.V. = 0)
}
