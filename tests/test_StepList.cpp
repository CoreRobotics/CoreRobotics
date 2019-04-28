/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#include "gtest/gtest.h"
#include <cr/core>
#include <iostream>
#include <memory>

#include "simpleStep.hpp"

// Use the CoreRobotics namespace
using namespace cr::core;

//------------------------------------------------------------------------------
/*!
 Test the reset function (called on construction)
 */
//------------------------------------------------------------------------------
TEST(StepList, init) {

  // Create the thread element smart pointer
  StepListPtr myStepList = StepList::create();

  // add the step function
  std::shared_ptr<simpleStep> myStep1 = std::make_shared<simpleStep>(0.01);
  std::shared_ptr<simpleStep> myStep2 = std::make_shared<simpleStep>(0.05);
  myStepList->attach(myStep1);
  myStepList->attach(myStep2);

  // check internal states
  EXPECT_DOUBLE_EQ(myStep1->x, myStep1->x0);
  EXPECT_DOUBLE_EQ(myStep2->x, myStep2->x0);

  // check the internal clocks
  myStepList->step();

  // Query the internal state to make sure the reset call happened
  EXPECT_NE(myStep1->x, myStep1->x0);
  EXPECT_NE(myStep2->x, myStep2->x0);
}
