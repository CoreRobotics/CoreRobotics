/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#ifndef CR_SIMPLE_STEP_HPP_
#define CR_SIMPLE_STEP_HPP_

#include <cr/core>

class simpleStep : public cr::core::Step {

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
  void step() override {

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

#endif
