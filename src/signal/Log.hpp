/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#ifndef CR_SIGNAL_LOG_HPP_
#define CR_SIGNAL_LOG_HPP_

#include "Message.hpp"
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

 \brief This class implements a logger that calls signal::SignalBase methods.
 */
//------------------------------------------------------------------------------
class Log : public core::Step, public core::Item {

public:
  //! Class constructor
  Log() = default;

  //! destructor
  virtual ~Log() = default;

  //! create a new log
  static LogPtr create();

public:
  //! add a message to the list
  void add(std::shared_ptr<MessageBase> i_message);

public:
  //! step the signal log
  virtual void step();

  //! start the signal log
  virtual void onStart();

  //! stop the signal log
  virtual void onStop();

protected:
  //! list of signals to log
  std::vector<std::shared_ptr<MessageBase>> m_messages;

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
} // namespace cr

#endif
