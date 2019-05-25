/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#include "gtest/gtest.h"
#include <cr/core>
#include <cr/runtime>
#include <iostream>

// Use the CoreRobotics namespace
using namespace cr::core;
using namespace cr::runtime;

// Callback for the thread
void myCallback1(void) {
  const char *memoryName = "MyMemory1";
  // Open some shared memory as client
  SharedMemory mem(memoryName, CR_MANAGER_CLIENT);

  // create a vector of data
  Eigen::VectorXd v(1);
  v << 0;

  int i = 0;
  while (i < 10) {
    v(0) = double(i);
    mem.set("signal_3", v);
    i++;
  }
}

// Callback for the thread
void myCallback2(void *arg) {
  const char *memoryName = "MyMemory2";
  // Open some shared memory as client
  SharedMemory mem(memoryName, CR_MANAGER_CLIENT);

  // create a vector of data
  Eigen::VectorXd v(1);
  v << 0;

  int i = 0;
  while (i < 10) {
    v(0) = double(i);
    mem.set("signal_4", v);
    i++;
  }
}

//
// Test thread start function
//
TEST(Thread, Start) {
  const char *memoryName = "MyMemory1";
  SharedMemory server(memoryName, CR_MANAGER_SERVER);
  Eigen::VectorXd v(1);
  v << 1.0;
  server.addSignal("signal_3", v);

  // Create a thread
  Thread myThread;
  myThread.setCallback(*myCallback1);

  // wait for the thread to finish
  myThread.join();

  Eigen::VectorXd v2 = server.get("signal_3");
  EXPECT_EQ(1, v2.size());
  EXPECT_DOUBLE_EQ(9, v2(0));
}

//
// Test thread start function - with argumnet
//
TEST(Thread, StartArgument) {
  const char *memoryName = "MyMemory2";
  SharedMemory server(memoryName, CR_MANAGER_SERVER);
  Eigen::VectorXd v(1);
  v << 1.0;
  server.addSignal("signal_4", v);

  // Create a thread
  void *arg = nullptr;
  Thread myThread;
  myThread.setCallback(*myCallback2, arg);

  // wait for the thread to finish
  myThread.join();

  Eigen::VectorXd v2 = server.get("signal_4");
  EXPECT_EQ(1, v2.size());
  EXPECT_DOUBLE_EQ(9, v2(0));
}
