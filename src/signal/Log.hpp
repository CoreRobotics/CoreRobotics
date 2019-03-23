/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#ifndef CR_LOG_HPP_
#define CR_LOG_HPP_

#include "Signal.hpp"
#include "core/Clock.hpp"
#include "core/Step.hpp"
#include <fstream>
#include <string>
#include <vector>

namespace cr {
namespace signal {

//! Node shared pointer
class Log;
typedef std::shared_ptr<Log> LogPtr;

//------------------------------------------------------------------------------
/*!
 \class Log
 \ingroup signal

 \brief This class implements a signal logger.

 \details
 ## Description

 */
//------------------------------------------------------------------------------
class Log : public core::Step, public core::Item {

  // Constructor and Destructor
public:
  //! Class constructor
  Log();

  //! destructor
  ~Log(){};

  //! create a new log
  static LogPtr create();

  // signal log controls
public:
  //! add a signal to the list
  void add(std::shared_ptr<SignalBase> i_signal);

  // Step method
public:
  //! step the signal log
  virtual void step();

  //! start the signal log
  virtual void onStart();

  //! stop the signal log
  virtual void onStop();

  // Protected members
protected:
  //! list of signals to log
  // std::vector<Requester<SupportedSignalTypes>*> m_signals;
  std::vector<std::shared_ptr<SignalBase>> m_signals;

  //! filename
  std::string m_filename;

  //! timer
  core::Clock m_timer;

  // ofstream object
  std::ofstream m_logFile;

  //! type (read only)
  std::string m_type = "Log";
};

} // namespace signal
} // namepsace cr

#endif
