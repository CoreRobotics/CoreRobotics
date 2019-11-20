/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#ifndef CR_COUNTER_STEP_HPP_
#define CR_COUNTER_STEP_HPP_

#include <cr/core>

//! This step class increments its count on every call
class counterStep : public cr::core::Step {
public:
  void onStart() override { counter = 0; }
  void step() override { counter++; }
  void onStop() override { counter = -1; }
  int counter = 0;
};

#endif
