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
 
 */
//---------------------------------------------------------------------

#include "Loop.hpp"

//---------------------------------------------------------------------
// Begin namespace
namespace cr {
namespace core {
    
    
//---------------------------------------------------------------------
/*!
 The constructor sets up the thread loop without a specific update 
 rate.  The loop will attempt to execute each step as quickly as 
 possible.\n
 */
//---------------------------------------------------------------------
Loop::Loop(){
    
}


//---------------------------------------------------------------------
/*!
The constructor sets up the thread loop with a specific update 
 rate.  The loop will attempt to execute each step at the rate
 specified by i_updateRate.\n
 
\param[in]	   i_updateRate	 this sets a constant update rate (s)
*/
//---------------------------------------------------------------------
Loop::Loop(double i_updateRate)
{
    
    // Set the thread update rate (s)
    m_updateRate = i_updateRate;
}


//---------------------------------------------------------------------
/*!
 The destructor closes the thread loop.\n
 */
//---------------------------------------------------------------------
Loop::~Loop(){
    
    // stop the run (only if running)
    if (m_runState != CR_RUN_STATE_STOPPED) {
        this->stop();
    }
    
    // delete pointers
    delete m_thread;
}
    
    
//---------------------------------------------------------------------
/*!
 Create a new Loop in memory.\n
 */
//---------------------------------------------------------------------
LoopPtr Loop::create(){
    return std::make_shared<Loop>();
}


//---------------------------------------------------------------------
/*!
 This function starts the thread loop.\n
 */
//---------------------------------------------------------------------
void Loop::start(){
    
    // check the thread state
    if (m_runState == CR_RUN_STATE_STOPPED)
    {
        
        // reset the pause offsets
        m_t0 = 0;
        m_tPaused = 0;
        
        // Setup the clock
        m_timer.startTimer();
        
        // set run flag (before assigning the callback)
        m_runState = CR_RUN_STATE_RUNNING;
        
        // Assign the callback (which begins the thread)
        // see lambdas: https://msdn.microsoft.com/en-us/library/dd293608.aspx
        m_thread = new std::thread( [this] { callback(); });
        
    }
    else if (m_runState == CR_RUN_STATE_PAUSED)
    {
        // compute the new offset
        m_t0 += m_timer.getElapsedTime() - m_tPaused;
        
        // set the run flag
        m_runState = CR_RUN_STATE_RUNNING;
    }
}

//---------------------------------------------------------------------
/*!
 This function pauses the thread loop.\n
 */
//---------------------------------------------------------------------
void Loop::pause(){
    
    // check the thread state
    if (m_runState == CR_RUN_STATE_RUNNING)
    {
        // set run flag
        m_runState = CR_RUN_STATE_PAUSED;
        
        // store the current time
        m_tPaused = m_timer.getElapsedTime();
    }
}

//---------------------------------------------------------------------
/*!
 This function stops the thread loop.\n
 */
//---------------------------------------------------------------------
void Loop::stop(){
    
    // check the thread state
    if (m_runState != CR_RUN_STATE_STOPPED)
    {
        // set run flag
        m_runState = CR_RUN_STATE_STOPPED;
        
        // join the main thread (i.e. wait for it to complete before continuing)
        m_thread->join();
        
        // now delete it & set to null
        delete m_thread;
        m_thread = NULL;
    }
}

//---------------------------------------------------------------------
/*!
 This is the thread loop callback function.\n
 */
//---------------------------------------------------------------------
void Loop::callback(){
    
    // run the thread
    while (m_runState != CR_RUN_STATE_STOPPED)
    {
        // get the time
        double t1 = m_timer.getElapsedTime();
        
        // if the thread is running
        if (m_runState == CR_RUN_STATE_RUNNING){
            
            // step the element
            m_element->step();
        }
        
        // update the frequency counter
        double et = m_timer.getElapsedTime() - t1;
        
        // sleep until completed
		if (m_updateRate > 0) {
			m_timer.sleep(m_updateRate - et);
		}
    }
}

//---------------------------------------------------------------------
/*!
 This function returns the current run time.\n
 
 \return        current run time (s)
 */
//---------------------------------------------------------------------
double Loop::getCurrentTime(){
    
    // compute the amount of time spent paused
    double tp = m_t0;
    if (m_runState == CR_RUN_STATE_PAUSED)
    {
        tp += m_timer.getElapsedTime() - m_tPaused;
    }
    
    return m_timer.getElapsedTime()-tp;
}


}
}
// end namespace
//---------------------------------------------------------------------


