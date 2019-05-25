/*
 * Copyright (c) 2017-2019, CoreRobotics
 * www.corerobotics.org
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 */

#include "Loop.hpp"

// platform specific includes
#if defined(WIN32) || defined(WIN64)
#include <windows.h>
#endif

#if defined(__linux__) || defined(__APPLE__)
#include <pthread.h>
#endif

namespace cr {
namespace core {

//---------------------------------------------------------------------
/*!
The constructor sets up the thread loop.  The loop will attempt to execute
each step at the rate specified by i_updateRate.\n

\param[in]  i_updateRate  The target update rate (s).
\param[in]  i_priority    The thread priority.
*/
//---------------------------------------------------------------------
Loop::Loop(const double i_updateRate,
           const ThreadPriority i_priority)
  : m_updateRate(i_updateRate), m_priority(i_priority) {}

//---------------------------------------------------------------------
/*!
 The destructor closes the thread loop.\n
 */
//---------------------------------------------------------------------
Loop::~Loop() {

  // stop the run (only if running)
  if (m_runState != CR_RUN_STATE_STOPPED) {
    this->stop();
  }

  // delete pointers
  delete m_thread;
}

//---------------------------------------------------------------------
/*!
 Create a new Loop in memory.\n
 */
//---------------------------------------------------------------------
LoopPtr Loop::create() { return std::make_shared<Loop>(); }

//---------------------------------------------------------------------
/*!
 This function starts the thread loop.\n
 */
//---------------------------------------------------------------------
void Loop::start() {

  // check the thread state
  if (m_runState == CR_RUN_STATE_STOPPED) {

    // reset the pause offsets
    m_t0 = 0;
    m_tPaused = 0;

    // Setup the clock
    m_timer.startTimer();

    // Start the element
    m_element->onStart();

    // set run flag (before assigning the callback)
    m_runState = CR_RUN_STATE_RUNNING;

    // Assign the callback (which begins the thread)
    // see lambdas: https://msdn.microsoft.com/en-us/library/dd293608.aspx
    m_thread = new std::thread([this] { callback(); });
    updatePriority();

  } else if (m_runState == CR_RUN_STATE_PAUSED) {
    // compute the new offset
    m_t0 += m_timer.getElapsedTime() - m_tPaused;

    // set the run flag
    m_runState = CR_RUN_STATE_RUNNING;
  }
}

//---------------------------------------------------------------------
/*!
 This function pauses the thread loop.\n
 */
//---------------------------------------------------------------------
void Loop::pause() {

  // check the thread state
  if (m_runState == CR_RUN_STATE_RUNNING) {
    // set run flag
    m_runState = CR_RUN_STATE_PAUSED;

    // store the current time
    m_tPaused = m_timer.getElapsedTime();
  }
}

//---------------------------------------------------------------------
/*!
 This function stops the thread loop.\n
 */
//---------------------------------------------------------------------
void Loop::stop() {

  // check the thread state
  if (m_runState != CR_RUN_STATE_STOPPED) {
    // stop the element
    m_element->onStop();

    // set run flag
    m_runState = CR_RUN_STATE_STOPPED;

    // join the main thread (i.e. wait for it to complete before continuing)
    m_thread->join();

    // now delete it & set to null
    delete m_thread;
    m_thread = NULL;
  }
}

//---------------------------------------------------------------------
/*!
 This is the thread loop callback function.\n
 */
//---------------------------------------------------------------------
void Loop::callback() {

  // run the thread
  while (m_runState != CR_RUN_STATE_STOPPED) {
    // get the time
    double t1 = m_timer.getElapsedTime();

    // if the thread is running
    if (m_runState == CR_RUN_STATE_RUNNING) {

      // step the element
      m_element->step();
    }

    // update the frequency counter
    double et = m_timer.getElapsedTime() - t1;

    // check for task overrun and sleep until completed
    if (m_updateRate > 0) {
      m_timer.sleep(m_updateRate - et);
    }
  }
}

//---------------------------------------------------------------------
/*!
 This function returns the current run time.\n

 \return        current run time (s)
 */
//---------------------------------------------------------------------
double Loop::getCurrentTime() {

  // compute the amount of time spent paused
  double tp = m_t0;
  if (m_runState == CR_RUN_STATE_PAUSED) {
    tp += m_timer.getElapsedTime() - m_tPaused;
  }

  return m_timer.getElapsedTime() - tp;
}

//---------------------------------------------------------------------
/*!
 This method sets the internal thread priority.
 */
//---------------------------------------------------------------------
void Loop::updatePriority() {

// Windows
#if defined(WIN32) || defined(WIN64)

  // get the thread handle
  assert(m_thread);
  HANDLE hThread = m_thread->native_handle();

  // get the process
  HANDLE process = GetCurrentProcess();
  SetPriorityClass(process, HIGH_PRIORITY_CLASS);

  int tPriority = THREAD_PRIORITY_NORMAL;
  switch (m_priority) {
  case CR_PRIORITY_LOWEST: // 11
    tPriority = THREAD_PRIORITY_LOWEST;
    break;
  case CR_PRIORITY_LOW: // 12
    tPriority = THREAD_PRIORITY_BELOW_NORMAL;
    break;
  case CR_PRIORITY_NORMAL: // 13
    tPriority = THREAD_PRIORITY_NORMAL;
    break;
  case CR_PRIORITY_HIGH: // 14
    tPriority = THREAD_PRIORITY_ABOVE_NORMAL;
    break;
  case CR_PRIORITY_HIGHEST: // 15
    tPriority = THREAD_PRIORITY_HIGHEST;
    break;
  }
  SetThreadPriority(hThread, tPriority);

#endif

#if defined(__linux__) || defined(__APPLE__)

  // get the thread handle
  pthread_t hThread = m_thread->native_handle();

  // return the policy and params for the thread
  struct sched_param sch;
  int tPolicy;
  pthread_getschedparam(hThread, &tPolicy, &sch);
  int fifo_min = sched_get_priority_min(SCHED_FIFO);
  int fifo_max = sched_get_priority_max(SCHED_FIFO);
  int rr_min = sched_get_priority_min(SCHED_RR);
  int rr_max = sched_get_priority_max(SCHED_RR);

  switch (m_priority) {
  case CR_PRIORITY_LOWEST:
    sch.sched_priority = rr_min;
    tPolicy = SCHED_RR;
    break;
  case CR_PRIORITY_LOW:
    sch.sched_priority = rr_max;
    tPolicy = SCHED_RR;
    break;
  case CR_PRIORITY_NORMAL:
    sch.sched_priority = fifo_min;
    tPolicy = SCHED_FIFO;
    break;
  case CR_PRIORITY_HIGH:
    sch.sched_priority = (fifo_max + fifo_min) / 2;
    tPolicy = SCHED_FIFO;
    break;
  case CR_PRIORITY_HIGHEST:
    sch.sched_priority = fifo_max;
    tPolicy = SCHED_FIFO;
    break;
  }
  pthread_setschedparam(hThread, tPolicy, &sch);

#endif
}

} // namepsace core
} // namespace cr
