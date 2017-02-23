//=====================================================================
/*
 Software License Agreement (BSD-3-Clause License)
 Copyright (c) 2017, CoreRobotics.
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
 \version 0.0
 
 */
//=====================================================================

#include "CRClock.hpp"
#include <chrono>
#include <thread>


//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics {
    
    
//=====================================================================
/*!
 The constructor defines a clock.\n
 */
//---------------------------------------------------------------------
CRClock::CRClock() {
	this->t0 = this->clock.now();
	this->t1 = this->clock.now();
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
void CRClock::startTimer() {
	this->t0 = this->clock.now();
}


//=====================================================================
/*!
 This method returns the elapsed time since the last call of 
 the startTimer() method.
 
 \param[out] t - the time [s] since the last call of startTimer()
 */
//---------------------------------------------------------------------
void CRClock::getElapsedTime(double &t) {
    t1 = this->clock.now();
    std::chrono::duration<double> elapsed = this->t1-this->t0;
    t = elapsed.count();
}
    
    
//=====================================================================
/*!
 This method sleeps the current thread for t seconds.
 
 \param[in] t - the time [s] to sleep
 */
//---------------------------------------------------------------------
void CRClock::sleep(double t) {
    const unsigned long ts = static_cast<unsigned long>( t * 1000000000.0 );
    std::this_thread::sleep_for(std::chrono::nanoseconds(ts));
}


//=====================================================================
// End namespace
}


