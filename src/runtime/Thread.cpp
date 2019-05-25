/*
 * Copyright (c) 2017-2019, CoreRobotics
 * www.corerobotics.org
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 */

#include "Thread.hpp"
#include <thread>

// platform specific includes
#if defined(WIN32) || defined(WIN64)
#include <windows.h>
#endif

#if defined(__linux__) || defined(__APPLE__)
#include <pthread.h>
#endif

namespace cr {
namespace runtime {

//---------------------------------------------------------------------
/*!
 The constructor defines a thread.\n

 \param[in] i_priority - the thread priority, see cr::ThreadPriority
 */
//---------------------------------------------------------------------
Thread::Thread(ThreadPriority i_priority) {
  this->m_loop = new std::thread;
  this->setPriority(i_priority);
}
Thread::Thread() {
  this->m_loop = new std::thread;
  this->setPriority(CR_PRIORITY_NORMAL);
}

//---------------------------------------------------------------------
/*!
 The destructor frees up memory.\n
 */
//---------------------------------------------------------------------
Thread::~Thread() {}

//---------------------------------------------------------------------
/*!
 This method sets the thread callback function and starts the thread.

 \param [in] i_callbackFunction - the callback function to be executed
                                by the thread
 */
//---------------------------------------------------------------------
void Thread::setCallback(void(i_callbackFunction)(void)) {
  *m_loop = std::thread(i_callbackFunction);
}

//---------------------------------------------------------------------
/*!
 This method sets the thread callback function with an argument and
 starts the thread.

 \param [in] i_callbackFunction - the callback function to be executed
 by the thread
 \param [in] i_arg              - argument to pass to thread
 */
//---------------------------------------------------------------------
void Thread::setCallback(void(i_callbackFunction)(void *), void *i_arg) {
  *m_loop = std::thread(i_callbackFunction, i_arg);
}

//---------------------------------------------------------------------
/*!
This method sets the thread priority.

\param[in] i_priority - the thread priority, see cr::ThreadPriority
*/
//---------------------------------------------------------------------
void Thread::setPriority(ThreadPriority i_priority) {

// Windows
#if defined(WIN32) || defined(WIN64)

  // get the thread handle
  HANDLE hThread = this->m_loop->native_handle();

  // get the process
  HANDLE process = GetCurrentProcess();
  SetPriorityClass(process, HIGH_PRIORITY_CLASS);

  int tPriority = THREAD_PRIORITY_NORMAL;
  switch (i_priority) {
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
  pthread_t hThread = this->m_loop->native_handle();

  // return the policy and params for the thread
  struct sched_param sch;
  int tPolicy;
  pthread_getschedparam(hThread, &tPolicy, &sch);
  int fifo_min = sched_get_priority_min(SCHED_FIFO);
  int fifo_max = sched_get_priority_max(SCHED_FIFO);
  int rr_min = sched_get_priority_min(SCHED_RR);
  int rr_max = sched_get_priority_max(SCHED_RR);

  switch (i_priority) {
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

//---------------------------------------------------------------------
/*!
 This method waits for a thread to finish its execution.  See C++
 std::thread for details.
 */
//---------------------------------------------------------------------
void Thread::join() { m_loop->join(); }

//---------------------------------------------------------------------
/*!
 This method permits the thread to execute independently from the
 thread handle.  See C++ std::thread for details.
 */
//---------------------------------------------------------------------
void Thread::detach() { m_loop->detach(); }

} // namepsace runtime
} // namespace cr
