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

#include "CRThread.hpp"
#include <thread>

// platform specific includes
#if defined(WIN32) || defined(WIN64)
	#include <windows.h>
#endif

#if defined(__linux__) || defined(__APPLE__)
	#include <pthread.h>
#endif

//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics {
    
    
//=====================================================================
/*!
 The constructor defines a thread.\n

 \param[in] priority - the thread priority, see CoreRobotics::CRThreadPriority
 */
//---------------------------------------------------------------------
CRThread::CRThread(CRThreadPriority i_priority) {
	this->m_loop = new std::thread;
	this->setPriority(i_priority);
}
CRThread::CRThread() {
    this->m_loop = new std::thread;
	this->setPriority(CR_PRIORITY_NORMAL);
}


//=====================================================================
/*!
 The destructor frees up memory.\n
 */
//---------------------------------------------------------------------
CRThread::~CRThread() { }

    

//=====================================================================
/*!
 This method sets the thread callback function.
 
 \param [in] callbackFunction - the callback function to be executed 
                                by the thread
 */
//---------------------------------------------------------------------
void CRThread::setCallback(void(callbackFunction)(void)) {
    *m_loop = std::thread(callbackFunction);
}



//=====================================================================
/*!
This method sets the thread priority.

\param[in] priority - the thread priority, see CoreRobotics::CRThreadPriority
*/
//---------------------------------------------------------------------
void CRThread::setPriority(CRThreadPriority i_priority) {

	// Windows
	#if defined(WIN32) || defined(WIN64)

		// get the thread handle
		HANDLE hThread = this->m_loop->native_handle();

		// get the process
		HANDLE process = GetCurrentProcess();
		SetPriorityClass(process, HIGH_PRIORITY_CLASS);

		int tPriority = THREAD_PRIORITY_NORMAL;
		switch (i_priority) {
			case CR_PRIORITY_LOWEST:	// 11
				tPriority = THREAD_PRIORITY_LOWEST;
				break;
			case CR_PRIORITY_LOW:		// 12
				tPriority = THREAD_PRIORITY_BELOW_NORMAL;
				break;
			case CR_PRIORITY_NORMAL:	// 13
				tPriority = THREAD_PRIORITY_NORMAL;
				break;
			case CR_PRIORITY_HIGH:		// 14
				tPriority = THREAD_PRIORITY_ABOVE_NORMAL;
				break;
			case CR_PRIORITY_HIGHEST:   // 15
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
    
        switch (i_priority) {
            case CR_PRIORITY_LOWEST:
                sch.sched_priority = 1;
                break;
            case CR_PRIORITY_LOW:
                sch.sched_priority = 25;
                break;
            case CR_PRIORITY_NORMAL:
                sch.sched_priority = 50;
                break;
            case CR_PRIORITY_HIGH:
                sch.sched_priority = 75;
                break;
            case CR_PRIORITY_HIGHEST:
                sch.sched_priority = 99;
                break;
        }
        pthread_setschedparam(hThread, SCHED_FIFO, &sch);

	#endif

}


//=====================================================================
/*!
 This method starts the thread by joining it to the main thread.
 */
//---------------------------------------------------------------------
void CRThread::start() {
    m_loop->join();
}
    
    
//=====================================================================
/*!
 This method stops the thread execution.
 */
//---------------------------------------------------------------------
void CRThread::stop() {
    m_loop->detach();
}


//=====================================================================
// End namespace
}


