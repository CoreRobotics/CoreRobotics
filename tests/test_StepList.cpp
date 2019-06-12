/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#include "gtest/gtest.h"
#include <cr/core>
#include <iostream>
#include <memory>

#include "counterStep.hpp"

// Use the CoreRobotics namespace
using namespace cr::core;

//------------------------------------------------------------------------------
/*!
 Test the reset function (called on construction)
 */
//------------------------------------------------------------------------------
TEST(StepList, init) {

  // Create the thread element
  StepList myStepList;

  // add the step function
  auto myStep1 = std::make_shared<counterStep>();
  auto myStep2 = std::make_shared<counterStep>();
  myStepList.attach(myStep1);
  myStepList.attach(myStep2);

  // check internal states
  EXPECT_DOUBLE_EQ(myStep1->counter, 0);
  EXPECT_DOUBLE_EQ(myStep2->counter, 0);

  myStepList.step();

  // check internal states
  EXPECT_DOUBLE_EQ(myStep1->counter, 1);
  EXPECT_DOUBLE_EQ(myStep2->counter, 1);

  myStepList.onStart();

  // check internal states
  EXPECT_DOUBLE_EQ(myStep1->counter, 0);
  EXPECT_DOUBLE_EQ(myStep2->counter, 0);

  myStepList.onStop();

  // check internal states
  EXPECT_DOUBLE_EQ(myStep1->counter, -1);
  EXPECT_DOUBLE_EQ(myStep2->counter, -1);
}
