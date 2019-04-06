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

class TestTypes {
public:
  Eigen::Vector3d getVector() {
    Eigen::Vector3d p(0.0, 1.0, 2.0);
    return p;
  }
  double getDouble() { return 1.0; }
  float getFloat() { return 2.0; }
  int getInt() { return 5; }
  bool getBool() { return false; }
};

//
// Test Slot connection
//
TEST(Log, Step) {

  std::shared_ptr<TestTypes> test = std::shared_ptr<TestTypes>(new TestTypes);

  // create the signal(s)
  std::shared_ptr<Signal<Eigen::Vector3d, TestTypes>> myVectorSignal =
      Signal<Eigen::Vector3d, TestTypes>::create(test, &TestTypes::getVector);
  myVectorSignal->setName("Vector");

  std::shared_ptr<Signal<bool, TestTypes>> myBoolSignal =
      Signal<bool, TestTypes>::create(test, &TestTypes::getBool);
  myBoolSignal->setName("Bool");

  std::shared_ptr<Signal<int, TestTypes>> myIntSignal =
      Signal<int, TestTypes>::create(test, &TestTypes::getInt);
  myIntSignal->setName("Int");

  std::shared_ptr<Signal<float, TestTypes>> myFloatSignal =
      Signal<float, TestTypes>::create(test, &TestTypes::getFloat);
  myFloatSignal->setName("Float");

  std::shared_ptr<Signal<double, TestTypes>> myDoubleSignal =
      Signal<double, TestTypes>::create(test, &TestTypes::getDouble);
  myDoubleSignal->setName("Double");

  // open a log and attach the signals
  LogPtr myLog = Log::create();
  myLog->add(myVectorSignal);
  myLog->add(myBoolSignal);
  myLog->add(myIntSignal);
  myLog->add(myFloatSignal);
  myLog->add(myDoubleSignal);

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
  std::string str =
      "time,Vector[0],Vector[1],Vector[2],Bool[0],Int[0],Float[0],Double[0],";
  std::getline(t, line);
  EXPECT_EQ(str, line);
  std::cout << line << "\n";
  while (t) {
    std::getline(t, line);
    std::cout << line << "\n";
  }
  t.close();
}
