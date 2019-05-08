/*
 * Copyright (c) 2017-2019, CoreRobotics
 * www.corerobotics.org
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef CR_LOOP_HPP_
#define CR_LOOP_HPP_

#include "Clock.hpp"
#include "Step.hpp"
#include "Types.hpp"
#include <thread>

namespace cr {
namespace core {

//! Loop smart pointer
class Loop;
typedef std::shared_ptr<Loop> LoopPtr;

//! Enumerator for specifying thread priority
enum ThreadPriority {
  CR_PRIORITY_LOWEST,
  CR_PRIORITY_LOW,
  CR_PRIORITY_NORMAL,
  CR_PRIORITY_HIGH,
  CR_PRIORITY_HIGHEST
};

//---------------------------------------------------------------------
/*!
 \class Loop
 \ingroup core

 \brief This class implements looped thread execution, which includes
 soft real-time control and start/stop functionality.

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
class Loop {

  //! Constructor and Destructor
public:
  //! Class constructor
  Loop();
  Loop(double i_updateRate);

  //! Class destructor
  virtual ~Loop();

  //! Create a pointer
  static LoopPtr create();

  //! API
public:
  //! Start the thread execution
  void start();

  //! Pause the thread execution
  void pause();

  //! Stop the thread execution
  void stop();

  //! Thread callback
  void callback();

  //! Attach the step element
  void attach(StepPtr i_element) { m_element = i_element; }

  //! Property updates
public:
  //! Set the internal thread priority
  void setPriority(ThreadPriority i_priority);

  //! Set the update rate
  void setUpdateRate(const double a_updateRate) { m_updateRate = a_updateRate; }

  //! Get the update rate
  double getUpdateRate() { return m_updateRate; }

  //! Get the current thread run time
  double getCurrentTime();

  //! Private members
private:
  //! simulation state
  RunState m_runState = CR_RUN_STATE_STOPPED;

  //! thread
  std::thread *m_thread = NULL;

  //! global timer
  Clock m_timer;

  //! thread update rate (s)
  double m_updateRate = 0;

  //! amount of time spent in pause since first start from stop (s).
  double m_t0 = 0;

  //! time at last pause() call
  double m_tPaused = 0;

  //! element to step
  StepPtr m_element;
};

} // namespace core
} // namespace cr

#endif
