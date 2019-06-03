/*
 * Copyright (c) 2017-2019, CoreRobotics
 * www.corerobotics.org
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef CR_ASPECT_TEMPORAL_HPP_
#define CR_ASPECT_TEMPORAL_HPP_

#include "core/Clock.hpp"

//! Classes that maintain sense of time should include this macro
#define CR_ASPECT_TEMPORAL_READ \
public: \
  const double getTime() const { return m_time; } \
protected: \
  double m_time = 0.0;

#define CR_ASPECT_TEMPORAL_WRITE \
CR_ASPECT_TEMPORAL_READ \
public: \
  void setTime(const double i_time) { m_time = i_time; }

//! Classes that maintain a time step should this macro
#define CR_ASPECT_TEMPORAL_DISCRETIZED \
CR_ASPECT_TEMPORAL_READ \
public: \
  void setTimeStep(const double i_dt) { m_dt = i_dt; } \
  const double getTimeStep() const { return m_dt; } \
protected: \
  double m_dt;

//! Classes that maintain a time step should this macro
#define CR_ASPECT_TEMPORAL_RUNTIME \
public: \
  double getTime() { return m_timer.getElapsedTime(); } \
protected: \
  core::Clock m_timer;

#endif
