/*
 * Copyright (c) 2017-2019, CoreRobotics
 * www.corerobotics.org
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef CR_CLOCK_HPP_
#define CR_CLOCK_HPP_

#include <chrono>

//---------------------------------------------------------------------
// Begin namespace
namespace cr {
namespace core {

//---------------------------------------------------------------------
/*!
 \class Clock
 \ingroup core

 \brief This class implements a clock for timing and sleeping.

 \details
 Clock implements a simple clock interface for timing and sleeping.

 - Clock::startTimer starts the clock.
 - Clock::getElapsedTime returns the elapsed time (s) since the most
 recent call of startTimer().
 - Clock::sleep sleeps the current thread (s).
 */
//---------------------------------------------------------------------
class Clock {

  // Constructor and Destructor
public:
  //! Class constructor
  Clock();

  //! Class destructor
  virtual ~Clock();

  // Public Methods
public:
  //! Start the timer
  void startTimer(void);

  //! Get the elapsed time [s] since startTimer() was called
  double getElapsedTime(void);

  //! Sleep the current thread
  void sleep(double i_time);

  // Protected Members
private:
  //! steady_clock object
  std::chrono::steady_clock m_clock;

  //! steady_clock timepoints
  std::chrono::steady_clock::time_point m_t0, m_t1;
};
}
}
// end namespace
//---------------------------------------------------------------------

#endif
