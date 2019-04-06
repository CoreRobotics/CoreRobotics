/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#include "gtest/gtest.h"
#include <cr/core>
#include <iostream>

// Use the CoreRobotics namespace
using namespace cr::core;

//
// Test the function of the clock to sleep for 100 ms
//
TEST(Clock, ClockSleepLong) {
  Clock MyClock;
  double t;
  MyClock.startTimer();
  MyClock.sleep(0.1);
  t = MyClock.getElapsedTime();
  EXPECT_NEAR(0.1, t, 0.01);
}

//
// Test the function of the clock to sleep for 1 ms
//
TEST(Clock, ClockSleepShort) {
  Clock MyClock;
  double t;
  MyClock.startTimer();
  MyClock.sleep(0.001);
  t = MyClock.getElapsedTime();
  EXPECT_NEAR(0.001, t, 0.001);
}
