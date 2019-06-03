/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */
 
#include "cr/core"
#include "cr/control"
#include "gtest/gtest.h"
#include <iostream>

// Use the CoreRobotics namespace
using namespace cr::control;
using namespace cr::core;

//
// Test solve method
//
TEST(MinimumJerk, Solve) {
  MinimumJerk::Parameters tgParams(2);
  Waypoint wp0, wpf;

  // Initial conditions
  Eigen::Vector2d x0, v0, a0;
  x0 << 0, 0;
  v0 << 0, -2;
  a0 << 0, 0;

  // Final conditions
  double tf = 1.2;
  Eigen::Vector2d xf, vf, af;
  xf << -0.4, 0.5;
  vf << 1.2, 1;
  af << 0, 0;

  // Check the solve function
  Result result = tgParams.solve(x0, v0, a0, xf, vf, af, tf);
  EXPECT_EQ(result, CR_RESULT_SUCCESS);

  // Now check the solve function with the waypoints
  wp0.position = x0;
  wp0.velocity = v0;
  wp0.acceleration = a0;
  wp0.time = 0;

  wpf.position = xf;
  wpf.velocity = vf;
  wpf.acceleration = af;
  wpf.time = tf;
  result = tgParams.solve(wp0, wpf);
  EXPECT_EQ(result, CR_RESULT_SUCCESS);
}

//
// Test step methods
//
TEST(MinimumJerk, Step) {
  MinimumJerk::Parameters tgParams(2);
  Waypoint wp;

  // Initial conditions
  Eigen::Vector2d x0, v0, a0;
  x0 << 0, 0;
  v0 << 0, -2;
  a0 << 0, 0;

  // Final conditions
  double tf = 1.2;
  Eigen::Vector2d xf, vf, af;
  xf << -0.4, 0.5;
  vf << 1.2, 1;
  af << 0, 0;

  // add a timer for running the internal clock
  Clock timer;

  // solve the trajectory
  tgParams.solve(x0, v0, a0, xf, vf, af, tf);

  // Check the internal timer
  MinimumJerk mjt(tgParams, wp);
  mjt.onStart();
  timer.startTimer();
  timer.sleep(tf);
  mjt.step();
  wp = mjt.getAction();
  EXPECT_NEAR(tf, wp.time, 0.01);
  EXPECT_NEAR(xf(0), wp.position(0), 0.001);
  EXPECT_NEAR(xf(1), wp.position(1), 0.001);
  EXPECT_NEAR(vf(0), wp.velocity(0), 0.001);
  EXPECT_NEAR(vf(1), wp.velocity(1), 0.001);
  EXPECT_NEAR(af(0), wp.acceleration(0), 0.001);
  EXPECT_NEAR(af(1), wp.acceleration(1), 0.001);

  // Check the value before the initial time (i.e.: t < 0)
  wp = mjt.policyCallback(-1);
  EXPECT_DOUBLE_EQ(0, wp.time);
  EXPECT_DOUBLE_EQ(x0(0), wp.position(0));
  EXPECT_DOUBLE_EQ(x0(1), wp.position(1));
  EXPECT_DOUBLE_EQ(v0(0), wp.velocity(0));
  EXPECT_DOUBLE_EQ(v0(1), wp.velocity(1));
  EXPECT_DOUBLE_EQ(a0(0), wp.acceleration(0));
  EXPECT_DOUBLE_EQ(a0(1), wp.acceleration(1));

  // Check the value at the initial time (i.e.: t = 0)
  wp = mjt.policyCallback(0);
  EXPECT_DOUBLE_EQ(0, wp.time);
  EXPECT_DOUBLE_EQ(x0(0), wp.position(0));
  EXPECT_DOUBLE_EQ(x0(1), wp.position(1));
  EXPECT_DOUBLE_EQ(v0(0), wp.velocity(0));
  EXPECT_DOUBLE_EQ(v0(1), wp.velocity(1));
  EXPECT_DOUBLE_EQ(a0(0), wp.acceleration(0));
  EXPECT_DOUBLE_EQ(a0(1), wp.acceleration(1));

  // Check the value at the end time (i.e.: t = tf)
  wp = mjt.policyCallback(tf);
  EXPECT_DOUBLE_EQ(tf, wp.time);
  EXPECT_NEAR(xf(0), wp.position(0), 1e-12);
  EXPECT_NEAR(xf(1), wp.position(1), 1e-12);
  EXPECT_NEAR(vf(0), wp.velocity(0), 1e-12);
  EXPECT_NEAR(vf(1), wp.velocity(1), 1e-12);
  EXPECT_NEAR(af(0), wp.acceleration(0), 1e-12);
  EXPECT_NEAR(af(1), wp.acceleration(1), 1e-12);

  // Check the value well past the end time (i.e.: t > tf)
  wp = mjt.policyCallback(tf + 10);
  EXPECT_DOUBLE_EQ(tf, wp.time);
  EXPECT_NEAR(xf(0), wp.position(0), 1e-12);
  EXPECT_NEAR(xf(1), wp.position(1), 1e-12);
  EXPECT_NEAR(vf(0), wp.velocity(0), 1e-12);
  EXPECT_NEAR(vf(1), wp.velocity(1), 1e-12);
  EXPECT_NEAR(af(0), wp.acceleration(0), 1e-12);
  EXPECT_NEAR(af(1), wp.acceleration(1), 1e-12);
}
