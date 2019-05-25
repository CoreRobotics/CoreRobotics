/*
 * Copyright (c) 2017-2019, CoreRobotics
 * www.corerobotics.org
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef CR_THREAD_HPP_
#define CR_THREAD_HPP_

#include <thread>

namespace cr {
namespace runtime {

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
 \class Thread
 \ingroup runtime

 \brief This class implements threading.

 \details
 Thread implements a simple thread interface for setting a callback
 and starting and stopping thread execution.

 - Thread::setCallback sets the callback function.
 - Thread::setPriority sets the priority of the thread.
 - Thread::join waits for the thread to join the main thread.
 - Thread::stop detach detaches the thread from the main thread.
 */
//---------------------------------------------------------------------
class Thread {

  //! Constructor and Destructor
public:
  //! Class constructor
  Thread();
  Thread(ThreadPriority i_priority);

  //! Class destructor
  virtual ~Thread();

  //! API
public:
  //! Set the thread callback function
  void setCallback(void(i_callbackFunction)(void));

  //! Set the thread callback function
  void setCallback(void(i_callbackFunction)(void *), void *arg);

  //! Set the thread priority
  void setPriority(ThreadPriority i_priority);

  //! Waits for a thread to finish its execution
  void join();

  //! Permits the thread to execute independently from the thread handle
  void detach();

  //! Private Members
private:
  //! thread pointer
  std::thread *m_loop;
};

} // namespace runtime
} // namespace cr

#endif
