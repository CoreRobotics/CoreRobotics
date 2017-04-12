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

#ifndef CRThread_hpp
#define CRThread_hpp

//=====================================================================
// Includes
#include <thread>


//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics {
    
//=====================================================================
/*!
 \file CRThread.hpp
 \brief Implements thread execution.
 */
//---------------------------------------------------------------------
/*!
 \class CRThread
 \ingroup core
 
 \brief This class implements threads to enable running multiple
 control loops within a single application.
 
 \details
 \section Description
 CRThread implements a simple thread interface for setting a callback
 and starting and stopping thread execution.
 
 - CRClock::setCallback sets the callback function.
 - CRClock::start starts the thread execution.
 - CRClock::stop stops the thread execution.
 
 \section Example
 This example creates and runs a simple CRThread.
 \code
 
 #include "CoreRobotics.hpp"
 
 using namespace CoreRobotics;
 
 void callback(void);
 
 main() {
    CRThread MyThread;
    MyThread.setCallback(*callback);
    MyThread.start();
 }
 
 // define a callback function for the thread
 void callback(void){
    CRClock clock;
    int i = 0;
    double et = 0.0;
    clock.startTimer();
    while(et < 10.0){
        printf("i = %i, et = %4.2f s\n",i,et);
        i++;                        // increment the counter
        clock.sleep(1.0);           // sleep for 1 second
        clock.getElapsedTime(et);   // return the elapsed time
    }
 }
 
 \endcode
 */
//=====================================================================
//! Enumerator for specifying thread priority
enum CRThreadPriority {
	CR_PRIORITY_LOWEST,
	CR_PRIORITY_LOW,
	CR_PRIORITY_NORMAL,
	CR_PRIORITY_HIGH,
	CR_PRIORITY_HIGHEST
};

//=====================================================================
class CRThread {
    
//---------------------------------------------------------------------
// Constructor and Destructor
public:
    
    //! Class constructor
    CRThread();
	CRThread(CRThreadPriority i_priority);
    
    //! Class destructor
    virtual ~CRThread();
    
    
//---------------------------------------------------------------------
// Public Methods
public:
    
    //! Set the thread callback function
    void setCallback(void(i_callbackFunction)(void));

	//! Set the thread priority
	void setPriority(CRThreadPriority i_priority);
    
    //! Start the thread
    void start();
    
    //! Stop the thread
    void stop();
    
    
//---------------------------------------------------------------------
// Protected Members
private:

	//! thread pointer
    std::thread* m_loop;
    
    
};
//=====================================================================
// End namespace
}

#endif
