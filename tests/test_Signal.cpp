/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#include "gtest/gtest.h"
#include <cr/core>
#include <cr/physics>
#include <cr/signal>
#include <iostream>

// Use the CoreRobotics namespace
using namespace cr::physics;
using namespace cr::core;
using namespace cr::signal;

//
// Test signal connection
//
TEST(Signal, Request) {

  // set up a Frame
  std::shared_ptr<FrameEuler> myClass = std::make_shared<FrameEuler>();
  myClass->setFreeVariable(CR_EULER_FREE_POS_X);
  myClass->setFreeValue(0.71);

  // create the signal
  std::shared_ptr<Signal<double, FrameEuler>> mySignalFreeValue =
      Signal<double, FrameEuler>::create(myClass, &FrameEuler::getFreeValue);

  // check values
  EXPECT_EQ(myClass, mySignalFreeValue->getEmitter());
  EXPECT_DOUBLE_EQ(0.71, mySignalFreeValue->request());

  // let's create another signal of different type
  std::shared_ptr<Signal<Eigen::Vector3d, FrameEuler>> mySignalPose =
      Signal<Eigen::Vector3d, FrameEuler>::create(myClass,
                                                  &FrameEuler::getTranslation);

  // check values
  Eigen::Vector3d v = mySignalPose->request();
  EXPECT_EQ(myClass, mySignalPose->getEmitter());
  EXPECT_DOUBLE_EQ(0.71, v(0));
  EXPECT_DOUBLE_EQ(0.0, v(1));
  EXPECT_DOUBLE_EQ(0.0, v(2));
}

using namespace std::placeholders;

//
// Test different function prototypes
// 
TEST(Signal, Types) {

  struct MyStruct {
    double getDouble() { return m_value; }
    double getDoubleConst()  const { return m_value; }
    void setDouble(double value) { m_value = value; }
    void setDoubleConst(const double value) { m_value = value; }
    void setDoubleConstRef(const double& value) { m_value = value; }
    double m_value = 1.0;
  };

  auto test = std::shared_ptr<MyStruct>(new MyStruct);

  std::function<double(void)> f1 = std::bind(
    &MyStruct::getDouble, test.get());

  std::function<double(void)> f2 = std::bind(
    &MyStruct::getDoubleConst, test.get());

  std::function<void(double)> f3 = std::bind(
    &MyStruct::setDouble, test.get(), _1);

  std::function<void(double)> f4 = std::bind(
    &MyStruct::setDoubleConst, test.get(), _1);

  std::function<void(double)> f5 = std::bind(
    &MyStruct::setDoubleConstRef, test.get(), _1);

  // create the signal(s)
  auto myGetDouble = new MySignal<double>(f1);
  auto myGetDoubleConst = new MySignal<double>(f2);

  // create the slot(s)
  // auto mySetDouble = new MySlot<double>(f3);
  // auto mySetDoubleConst = new MySlot<double>(f4);
  // auto mySetDoubleConstRef = new MySlot<double>(f5);

  EXPECT_DOUBLE_EQ(1.0, myGetDouble->request());
  EXPECT_DOUBLE_EQ(1.0, myGetDoubleConst->request());
}
