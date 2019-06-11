/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#include <cr/core>

#include <iostream>

//! This step class increments its count on every call
class counterStep : public cr::core::Step {
public:
  void onStart() override { 
    counter = 0;
  }
  void step() override { 
    counter++;
  }
  void onStop() override {
  }
  unsigned counter = 0;
};
