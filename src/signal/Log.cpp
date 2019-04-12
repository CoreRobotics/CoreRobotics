/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#include "signal/Log.hpp"

namespace cr {
namespace signal {

//------------------------------------------------------------------------------
/*!
 The constructor creates a log.\n
 */
//------------------------------------------------------------------------------
Log::Log() {}

//------------------------------------------------------------------------------
/*!
 Create a new signal log item.\n
 */
//------------------------------------------------------------------------------
LogPtr Log::create() { return std::make_shared<Log>(); }

//------------------------------------------------------------------------------
/*!
 This method adds a signal to the list.

 \param[in]     i_signal - the signal to add
 */
//------------------------------------------------------------------------------
void Log::add(std::shared_ptr<SignalBase> i_signal) {
  m_signals.push_back(i_signal);
}

//------------------------------------------------------------------------------
/*!
 This method steps the signal log.
 */
//------------------------------------------------------------------------------
void Log::step() {
  // get the time
  double t = m_timer.getElapsedTime();

  // write the time
  Utility::write(m_logFile, t);

  // now write each signal value
  for (unsigned i = 0; i < m_signals.size(); i++) {
    m_signals.at(i)->print(m_logFile);
  }

  // write the new line
  m_logFile << "\n";
}

//------------------------------------------------------------------------------
/*!
 This method starts the signal log.
 */
//------------------------------------------------------------------------------
void Log::onStart() {
  m_logFile.open(m_name);

  // write the time signal
  std::string str = "time";
  Utility::write(m_logFile, str);

  // write each signal header
  for (unsigned i = 0; i < m_signals.size(); i++) {
    unsigned n = m_signals.at(i)->size();
    for (unsigned j = 0; j < n; j++) {
      std::string index = "[" + std::to_string(j) + "]";
      Utility::write(m_logFile, m_signals.at(i)->getName().append(index));
    }
  }

  // write the new line
  m_logFile << "\n";

  // start the internal clock
  m_timer.startTimer();
}

//------------------------------------------------------------------------------
/*!
 This method stops the signal log.
 */
//------------------------------------------------------------------------------
void Log::onStop() {
  m_logFile.close();
}

} // namespace signal
} // namepsace cr
