/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#include "gtest/gtest.h"
#include <cr/core>
#include <cr/runtime>
#include <iostream>
#include <memory>

#include "simpleStep.hpp"
#include "counterStep.hpp"

// Use the CoreRobotics namespace
using namespace cr::core;
using namespace cr::runtime;

// this function runs a timed test of the loop at a specified sample rate
void timed_loop_test(const double dt, const double eps) {
  LoopPtr myLoop = Loop::create();
  myLoop->setPriority(CR_PRIORITY_HIGHEST);

  auto myStep = std::make_shared<counterStep>();
  myLoop->attach(myStep);

  myLoop->setUpdateRate(dt);
  EXPECT_DOUBLE_EQ(myLoop->getUpdateRate(), dt);

  Clock timer;

  // check start run and pause
  myLoop->start();
  timer.sleep((10 + eps) * dt);
  myLoop->pause();
  EXPECT_EQ(myStep->counter, 11);

  // check nothing happens while paused
  timer.sleep(10 * dt);
  EXPECT_EQ(myStep->counter, 11);

  // check restart from pause
  myLoop->start();
  timer.sleep(10 * dt);
  myLoop->pause();
  EXPECT_EQ(myStep->counter, 21);

  // check reset on stop
  myLoop->start();
  timer.sleep(1 * dt);
  myLoop->stop();
  EXPECT_EQ(myStep->counter, -1);

  // check start after previous stop
  myLoop->start();
  timer.sleep((10 + eps) * dt);
  myLoop->pause();
  EXPECT_EQ(myStep->counter, 11);

  myLoop->stop();
}

//------------------------------------------------------------------------------
/*!
 Test the reset function (called on construction)
 */
//------------------------------------------------------------------------------
TEST(Loop, init) {

  // Create the thread element smart pointer
  double dt = 0.124;
  Loop myLoop(dt);
  EXPECT_DOUBLE_EQ(dt, myLoop.getUpdateRate());
}

//------------------------------------------------------------------------------
/*!
 Test the reset function (called on construction)
 */
//------------------------------------------------------------------------------
TEST(Loop, reset) {

  // Create the thread element smart pointer
  LoopPtr myLoop = Loop::create();

  // add the step function
  auto myStep = std::make_shared<simpleStep>(0.01);
  myLoop->attach(myStep);

  // Query the internal state to make sure the reset call happened
  EXPECT_DOUBLE_EQ(myStep->x, myStep->x0);
}

//------------------------------------------------------------------------------
/*!
 Test the thread execution functionality
 */
//------------------------------------------------------------------------------
TEST(Loop, execution100Hz) {
  double dt = 0.01;
  double eps = 1e-1;
  timed_loop_test(dt, eps);
}
