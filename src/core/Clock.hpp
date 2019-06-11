/*
 * Copyright (c) 2017-2019, CoreRobotics
 * www.corerobotics.org
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef CR_CORE_CLOCK_HPP_
#define CR_CORE_CLOCK_HPP_

#include <chrono>

namespace cr {
namespace core {

//---------------------------------------------------------------------
/*!
 \class Clock
 \ingroup core

 \brief This class implements a clock for timing and sleeping.
 */
//---------------------------------------------------------------------
class Clock {

public:
  //! Class constructor
  Clock();

  //! Class destructor
  virtual ~Clock() = default;

public:
  //! Start the timer
  void startTimer();

  //! Get the elapsed time [s] since startTimer() was called
  double getElapsedTime();

  //! Sleep the current thread
  void sleep(const double i_time);

private:
  //! steady_clock object
  std::chrono::steady_clock m_clock;

  //! steady_clock timepoints
  std::chrono::steady_clock::time_point m_t0, m_t1;
};

} // namespace core
} // namespace cr

#endif
