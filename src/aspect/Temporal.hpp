/*
 * Copyright (c) 2017-2019, CoreRobotics
 * www.corerobotics.org
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef CR_ASPECT_TEMPORAL_HPP_
#define CR_ASPECT_TEMPORAL_HPP_

#include "core/Clock.hpp"

/*!
  \def CR_ASPECT_TEMPORAL_READ
  \ingroup aspect
  Classes that can read time should include this macro

  \brief This macro adds class members.
*/
#define CR_ASPECT_TEMPORAL_READ \
public: \
  double getTime() const { return m_time; } \
protected: \
  double m_time = 0.0;

/*!
  \def CR_ASPECT_TEMPORAL_READ
  \ingroup aspect
  Classes that can read/write time should include this macro

  \brief This macro adds class members.
*/
#define CR_ASPECT_TEMPORAL_WRITE \
CR_ASPECT_TEMPORAL_READ \
public: \
  void setTime(const double i_time) { m_time = i_time; }

/*!
  \def CR_ASPECT_TEMPORAL_DISCRETIZED
  \ingroup aspect
  Classes that maintain a time step include this macro.

  \brief This macro adds class members.
*/
#define CR_ASPECT_TEMPORAL_DISCRETIZED \
CR_ASPECT_TEMPORAL_READ \
public: \
  void setTimeStep(const double i_dt) { m_dt = i_dt; } \
  double getTimeStep() const { return m_dt; } \
protected: \
  double m_dt;

/*!
  \def CR_ASPECT_TEMPORAL_RUNTIME
  \ingroup aspect
  Classes that get time from an interal clock include this macro.

  \brief This macro adds class members.
*/
#define CR_ASPECT_TEMPORAL_RUNTIME \
public: \
  double getTime() { return m_timer.getElapsedTime(); } \
protected: \
  core::Clock m_timer;

#endif
