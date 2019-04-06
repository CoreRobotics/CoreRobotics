/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#include "gtest/gtest.h"
#include <cr/core>
#include <iostream>
#include <memory>

// Use the CoreRobotics namespace
using namespace cr::core;

//------------------------------------------------------------------------------
// Define a simple step element.
//------------------------------------------------------------------------------
class simpleStep : public Step {

  // init
public:
  // constructor
  simpleStep(double dt) {
    m_dt = dt;
    x = x0;
  }

  // Primary ThreadElement functions
public:
  //! Step (called on each iteration of the thread)
  virtual void step() {

    // this is a simple low pass system with forward euler discretiation
    x = (1 - m_dt / m_tau) * x;
  };

  // Private members
public:
  //! Initial condition
  double x0 = 2.0;

  //! internal state
  double x = 0;

  //! internal parameter
  double m_tau = 0.5;

  //! sample rate
  double m_dt = 0.01;
};

//------------------------------------------------------------------------------
/*!
 Test the reset function (called on construction)
 */
//------------------------------------------------------------------------------
TEST(Loop, reset) {

  // Create the thread element smart pointer
  LoopPtr myLoop = Loop::create();

  // add the step function
  std::shared_ptr<simpleStep> myStep = std::make_shared<simpleStep>(0.01);
  myLoop->attach(myStep);

  // Query the internal state to make sure the reset call happened
  EXPECT_DOUBLE_EQ(myStep->x, myStep->x0);
}

//------------------------------------------------------------------------------
/*!
 Test the thread execution functionality
 */
//------------------------------------------------------------------------------
TEST(Loop, execution) {

  // Create the thread element smart pointer
  LoopPtr myLoop = Loop::create();

  // Set the update rate to be super slow - 20 Hz
  double dt = 0.05;

  // add the step function
  std::shared_ptr<simpleStep> myStep = std::make_shared<simpleStep>(dt);
  myLoop->attach(myStep);

  // check the update rate
  myLoop->setUpdateRate(dt);
  EXPECT_DOUBLE_EQ(myLoop->getUpdateRate(), dt);

  // Create a timer
  Clock timer;

  // Start the thread
  // std::cout << "Start() call\n";
  myLoop->start();

  // wait for 1 second
  timer.startTimer();
  timer.sleep(1.0);

  // Pause the thread
  // std::cout << "Pause() call\n";
  myLoop->pause();

  EXPECT_NEAR(myLoop->getCurrentTime(), 1.0, dt); // within dt
  EXPECT_NEAR(myStep->x, 0, 0.4 * myStep->x0);    // 1 time constant

  // wait for 1 second
  timer.sleep(1.0);

  EXPECT_NEAR(myLoop->getCurrentTime(), 1.0, dt); // within dt
  EXPECT_NEAR(myStep->x, 0, 0.4 * myStep->x0);    // 1 time constant

  // start the thread
  // std::cout << "Start() call\n";
  myLoop->start();

  // wait for 1 second
  timer.sleep(1.0);

  // Pause the thread
  // std::cout << "Pause() call\n";
  myLoop->pause();

  EXPECT_NEAR(myLoop->getCurrentTime(), 2.0, dt); // within dt
  EXPECT_NEAR(myStep->x, 0, 0.28 * myStep->x0);   // 2 time constants

  // wait for 1 second
  timer.sleep(1.0);

  // start the thread
  // std::cout << "Start() call\n";
  myLoop->start();

  // wait for 1 second
  timer.sleep(1.0);

  // stop the thread
  // std::cout << "Stop() call\n";
  myLoop->stop();

  EXPECT_NEAR(myLoop->getCurrentTime(), 3.0, 2 * dt); // within 2dt
  EXPECT_NEAR(myStep->x, 0, 0.07 * myStep->x0);       // 3 time constants

  // wait for 1 second
  timer.sleep(1.0);

  // restart the thread
  // std::cout << "Start() call\n";
  myLoop->start();

  // wait for 1 second
  timer.sleep(1.0);

  // stop the thread
  // std::cout << "Stop() call\n";
  myLoop->stop();

  EXPECT_NEAR(myLoop->getCurrentTime(), 1.0, 2 * dt); // within 2dt
  EXPECT_NEAR(myStep->x, 0, 0.4 * myStep->x0);        // 1 time constant
}
