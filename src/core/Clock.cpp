/*
 * Copyright (c) 2017-2019, CoreRobotics
 * www.corerobotics.org
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 */

#include "Clock.hpp"
#include <chrono>
#include <thread>

namespace cr {
namespace core {

//---------------------------------------------------------------------
/*!
 The constructor defines a clock.\n
 */
//---------------------------------------------------------------------
Clock::Clock() {
  m_t0 = m_clock.now();
  m_t1 = m_clock.now();
}

//---------------------------------------------------------------------
/*!
 This method starts the timer.
 */
//---------------------------------------------------------------------
void Clock::startTimer(void) { this->m_t0 = this->m_clock.now(); }

//---------------------------------------------------------------------
/*!
 This method returns the elapsed time since the last call of
 the startTimer() method.

 \return - the time [s] since the last call of startTimer()
 */
//---------------------------------------------------------------------
double Clock::getElapsedTime(void) {
  m_t1 = this->m_clock.now();
  std::chrono::duration<double> elapsed = this->m_t1 - this->m_t0;
  return elapsed.count();
}

//---------------------------------------------------------------------
/*!
 This method sleeps the current thread for i_time (s).  Note that for
 short sleep durations (i.e. < 50 ms), this method will spinlock the
 current thread (maintaining full thread utilization), while for higher
 sleep durations, the method will sleep the thread to free up process.

 \param[in] i_time - the time [s] to sleep
 */
//---------------------------------------------------------------------
void Clock::sleep(const double i_time) {
  if (i_time < 0.05) {
    // spinlock the thread
    std::chrono::steady_clock::time_point tNow0 = this->m_clock.now();
    std::chrono::steady_clock::time_point tNow1 = this->m_clock.now();
    std::chrono::duration<double> et = tNow1 - tNow0;
    while (et.count() < i_time) {
      tNow1 = this->m_clock.now();
      et = tNow1 - tNow0;
    }
  } else {
    // sleep the thread to free up processor
    std::this_thread::sleep_for(std::chrono::duration<double>(i_time));
  }
}

} // namepsace core
} // namespace cr
