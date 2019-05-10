/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#include "gtest/gtest.h"
#include <cr/core>
#include <cr/physics>
#include <cr/signal>
#include <cr/world>
#include <iostream>

// Use the CoreRobotics namespace
using namespace cr::core;
using namespace cr::world;
using namespace cr::signal;
using namespace cr::physics;

//
// Test Slot connection
//
// TODO: test breaks on Signal::create()
/*
TEST(Slot, Step) {

  // set up 2 links
  LinkPtr myLink1 = Link::create();
  LinkPtr myLink2 = Link::create();

  myLink1->setDegreeOfFreedom(CR_EULER_FREE_POS_X);
  myLink1->setFreeValue(0.71);

  myLink2->setDegreeOfFreedom(CR_EULER_FREE_POS_X);
  myLink2->setFreeValue(0);

  EXPECT_DOUBLE_EQ(0.71, myLink1->getFreeValue());
  EXPECT_DOUBLE_EQ(0, myLink2->getFreeValue());

  // create the signal
  std::shared_ptr<Signal<double, Link>> mySignal =
      Signal<double, Link>::create(myLink1, &Link::getFreeValue);

  // check the emitter
  EXPECT_EQ(myLink1, mySignal->getEmitter());

  // create the slot
  std::shared_ptr<Slot<double, Link>> mySlot =
      Slot<double, Link>::create(mySignal, myLink2, &Link::setFreeValue);

  // check the receiver
  EXPECT_EQ(myLink2, mySlot->getReceiver());

  // perform the step
  mySlot->step();

  // check the data value
  EXPECT_DOUBLE_EQ(myLink1->getFreeValue(), mySignal->request());
  EXPECT_DOUBLE_EQ(myLink1->getFreeValue(), myLink2->getFreeValue());
}
*/