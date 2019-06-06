/*
 * Copyright (c) 2017-2019, CoreRobotics
 * www.corerobotics.org
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef CR_LOOP_HPP_
#define CR_LOOP_HPP_

#include <thread>

#include "core/Item.hpp"
#include "core/Clock.hpp"
#include "core/Step.hpp"
#include "core/Types.hpp"
#include "Thread.hpp"

namespace cr {
namespace runtime {

/*!
Loop run state
\ingroup runtime
*/
enum RunState {
  CR_RUN_STATE_RUNNING,
  CR_RUN_STATE_STOPPED,
  CR_RUN_STATE_PAUSED,
};

/*!
Enumerator for specifying thread priority
\ingroup runtime
TODO: https://gitlab.com/powan/CoreRobotics/issues/55
*/
enum RealtimePolicy {
  CR_REALTIME_POLICY_SOFT,
  CR_REALTIME_POLICY_HARD
};

//! Loop smart pointer
class Loop;
typedef std::shared_ptr<Loop> LoopPtr;

//---------------------------------------------------------------------
/*!
 \class Loop
 \ingroup runtime

 \brief This class implements looped thread execution, which includes
 real-time control and start/stop functionality.

 \details
 Loop implements a thread loop controller class, which includes
 the following methods:

 - Loop::attach attaches a StepPtr element to be executed.  Only one
 StepPtr element can be owned by a Loop.  For graphs of StepPtr elements,
 use a core::StepList.
 - Loop::start starts the thread loop.
 - Loop::pause pauses the loop execution, but does not exit the
 thread of execution.
 - Loop::stop exits the thread of execution.
 */
//---------------------------------------------------------------------
class Loop : public cr::core::Item {

public:
  //! Class constructor
  Loop(const double i_updateRate = 0.1,
       const ThreadPriority i_priority = CR_PRIORITY_NORMAL);

  //! Class destructor
  virtual ~Loop();

  //! Create a pointer
  static LoopPtr create();

public:
  //! Start the thread execution
  virtual void start();

  //! Pause the thread execution
  void pause();

  //! Stop the thread execution
  void stop();

  //! Thread callback
  void callback();

  //! Attach the step element
  void attach(core::StepPtr i_element) { m_element = i_element; }

public:
  //! Set the internal thread priority
  void setPriority(ThreadPriority i_priority) { m_priority = i_priority; }

  //! Get the internal thread priority
  ThreadPriority getPriority() { return m_priority; }

  //! Set the update rate
  void setUpdateRate(const double a_updateRate) { m_updateRate = a_updateRate; }

  //! Get the update rate
  double getUpdateRate() { return m_updateRate; }

  //! Get the current thread run time
  double getCurrentTime();

private:
  //! Update the thread piority
  void updatePriority();

  //! thread update rate (s)
  double m_updateRate;

  //! Realtime policy
  // TODO: https://gitlab.com/powan/CoreRobotics/issues/55
  RealtimePolicy m_policy;

  //! simulation state
  RunState m_runState = CR_RUN_STATE_STOPPED;

  //! thread priority
  ThreadPriority m_priority = CR_PRIORITY_NORMAL;

  //! thread
  std::thread *m_thread = NULL;

  //! global timer
  core::Clock m_timer;

  //! amount of time spent in pause since first start from stop (s).
  double m_t0 = 0;

  //! time at last pause() call
  double m_tPaused = 0;

  //! element to step
  core::StepPtr m_element;
};

} // namespace runtime
} // namespace cr

#endif
