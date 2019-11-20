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

#include "customType.hpp"

// Use the CoreRobotics namespace
using namespace cr::core;
using namespace cr::world;
using namespace cr::signal;
using namespace cr::physics;

//
// Test Slot connection
//
TEST(Log, Step) {

  auto test = std::shared_ptr<TypeEmitter>(new TypeEmitter);

  // create the signal(s)
  auto myVectorSignal = Signal<Eigen::Vector3d, TypeEmitter>::create(
      test, &TypeEmitter::getVector);
  auto myBoolSignal =
      Signal<bool, TypeEmitter>::create(test, &TypeEmitter::getBool);
  auto myIntSignal =
      Signal<int, TypeEmitter>::create(test, &TypeEmitter::getInt);
  auto myFloatSignal =
      Signal<float, TypeEmitter>::create(test, &TypeEmitter::getFloat);
  auto myDoubleSignal =
      Signal<double, TypeEmitter>::create(test, &TypeEmitter::getDouble);
  auto myCustomSignal =
      Signal<CustomType, TypeEmitter>::create(test, &TypeEmitter::getCustom);

  // tap the signals as messages
  auto myVectorMessage = Tap<Eigen::Vector3d>::create(myVectorSignal);
  auto myBoolMessage = Tap<bool>::create(myBoolSignal);
  auto myIntMessage = Tap<int>::create(myIntSignal);
  auto myFloatMessage = Tap<float>::create(myFloatSignal);
  auto myDoubleMessage = Tap<double>::create(myDoubleSignal);
  auto myCustomMessage =
      Tap<CustomType, CustomSerializer>::create(myCustomSignal);

  myVectorMessage->setName("Vector");
  myBoolMessage->setName("Bool");
  myIntMessage->setName("Int");
  myFloatMessage->setName("Float");
  myDoubleMessage->setName("Double");
  myCustomMessage->setName("Custom");

  // open a log and attach the signals
  LogPtr myLog = Log::create();
  myLog->add(myVectorMessage);
  myLog->add(myBoolMessage);
  myLog->add(myIntMessage);
  myLog->add(myFloatMessage);
  myLog->add(myDoubleMessage);
  myLog->add(myCustomMessage);

  // Start the log
  Clock c;
  c.startTimer();
  myLog->setName("my_log");
  myLog->onStart();
  myLog->step();
  c.sleep(0.001);
  myLog->step();
  myLog->step();
  c.sleep(0.001);
  myLog->step();
  myLog->step();
  myLog->onStop();

  // Now we should load the log file
  // and make sure it's what we expect
  std::ifstream t;
  t.open("my_log");
  std::string buffer;
  std::string line;
  std::string str = "time,Vector[0],Vector[1],Vector[2],Bool[0],Int[0],Float[0]"
                    ",Double[0],Custom[0],Custom[1],";
  std::getline(t, line);
  EXPECT_EQ(str, line);
  while (t) {
    std::getline(t, line);
  }
  t.close();
}
