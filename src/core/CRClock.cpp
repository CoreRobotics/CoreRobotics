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

#include "CRClock.hpp"
#include <chrono>
#include <thread>


//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics  {
    
    
//=====================================================================
/*!
 The constructor defines a clock.\n
 */
//---------------------------------------------------------------------
CRClock::CRClock() {
	this->m_t0 = this->m_clock.now();
	this->m_t1 = this->m_clock.now();
}


//=====================================================================
/*!
 The destructor frees up memory.\n
 */
//---------------------------------------------------------------------
CRClock::~CRClock() { }


//=====================================================================
/*!
 This method starts the timer.
 */
//---------------------------------------------------------------------
void CRClock::startTimer(void) {
	this->m_t0 = this->m_clock.now();
}


//=====================================================================
/*!
 This method returns the elapsed time since the last call of 
 the startTimer() method.
 
 \return - the time [s] since the last call of startTimer()
 */
//---------------------------------------------------------------------
double CRClock::getElapsedTime(void) {
    m_t1 = this->m_clock.now();
    std::chrono::duration<double> elapsed = this->m_t1-this->m_t0;
    return elapsed.count();
}
    
    
//=====================================================================
/*!
 This method sleeps the current thread for i_time (s).  Note that for
 short sleep durations (i.e. < 50 ms), this method will spinlock the
 current thread (maintaining full thread utilization), while for higher
 sleep durations, the method will sleep the thread to free up process.
 
 \param[in] i_time - the time [s] to sleep
 */
//---------------------------------------------------------------------
void CRClock::sleep(double i_time) {
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


//=====================================================================
// End namespace
}


