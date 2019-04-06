//=====================================================================
/*
Software License Agreement (BSD-3-Clause License)
Copyright (c) 2019, CoreRobotics.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.

* Neither the name of CoreRobotics nor the names of its contributors
may be used to endorse or promote products derived from this
software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

\project CoreRobotics Project
\url     www.corerobotics.org
\author  Parker Owan

*/
//=====================================================================

#ifndef CRClock_hpp
#define CRClock_hpp

//=====================================================================
// Includes
#include "CRTypes.hpp"
#include <chrono>

//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics {

//=====================================================================
/*!
 \file CRClock.hpp
 \brief Implements a clock.
 */
//---------------------------------------------------------------------
/*!
 \class CRClock
 \ingroup core

 \brief This class implements a clock for timing code sections and
 control loop rates.

 \details
 ## Description
 CRClock implements a simple clock interface for timing the
 execution of code.

 - CRClock::startTimer starts the clock.
 - CRClock::getElapsedTime returns the elapsed time (s) since the most
 recent call of startTimer().
 - CRClock::sleep sleeps the current thread (s).

 ## Example
 This example creates and runs a simple CRClock.
 \include example_CRCore.cpp
 */
//=====================================================================
#ifndef SWIG
class[[deprecated(CR_DEPRECATED)]] CRClock {
#else
class CRClock {
#endif

  //---------------------------------------------------------------------
  // Constructor and Destructor
public:
  //! Class constructor
  CRClock();

  //! Class destructor
  virtual ~CRClock();

  //---------------------------------------------------------------------
  // Public Methods
public:
  //! Start the timer
  void startTimer(void);

  //! Get the elapsed time [s] since startTimer() was called
  double getElapsedTime(void);

  //! Sleep the current thread
  void sleep(double i_time);

  //---------------------------------------------------------------------
  // Protected Members
private:
  //! steady_clock object
  std::chrono::steady_clock m_clock;

  //! steady_clock timepoints
  std::chrono::steady_clock::time_point m_t0, m_t1;
};
//=====================================================================
// End namespace
}

#endif
