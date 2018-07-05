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
// Begin header definition

#ifndef CR_LOOP_HPP_
#define CR_LOOP_HPP_

#include <thread>
#include "Clock.hpp"
#include "Types.hpp"

//---------------------------------------------------------------------
// Begin namespace
namespace cr {

    
//---------------------------------------------------------------------
/*!
 \class Loop
 \ingroup core
 
 \brief This class implements looped thread execution, which includes
 soft real-time control and start/stop functionality.
 
 \details
 ## Description
 Loop implements a thread loop controller class, which includes
 the following methods:
 
 - Loop::start starts the thread loop element.
 - Loop::pause pauses the loop element execution, but does not
 exit the thread of execution.
 - Loop::stop exits the thread of execution.
 - Loop::stop stops the thread execution.
 */
//---------------------------------------------------------------------
class LoopElement;
class Loop {
    
    // Constructor and Destructor
    public:
    
        //! Class constructor
        Loop(LoopElement* i_member);
		Loop(LoopElement* i_member, double i_updateRate);
    
        //! Class destructor
        virtual ~Loop();
    
    
    // Primary control functions
    public:
    
        //! Start the thread execution
        void start();
    
        //! Pause the thread execution
        void pause();
    
        //! Stop the thread execution
        void stop();
    
        //! Thread callback
        void callback();
    
    
    // Set/get functions
    public:
    
        //! Set the update rate
        void setUpdateRate(const double a_updateRate) {m_updateRate = a_updateRate;}
    
        //! Get the update rate
        double getUpdateRate() {return m_updateRate;}
    
        //! Get the current thread run time
        double getCurrentTime();
    

    // Private members
    private:
    
        //! thread member
        LoopElement* m_element;
    
        //! simulation state
        RunState m_runState = CR_RUN_STATE_STOPPED;
    
        //! thread
        // cr::Thread* m_thread = NULL;
        std::thread* m_thread = NULL;
    
        //! global timer
        Clock m_timer;
    
        //! thread update rate (s)
        double m_updateRate = 0;
    
        //! amount of time spent in pause since first start from stop (s).
        double m_t0 = 0;
    
        //! time at last pause() call
        double m_tPaused = 0;
    
};
    
    
//---------------------------------------------------------------------
/*!
 Smart pointer to Loop.
 */
//---------------------------------------------------------------------
typedef std::shared_ptr<Loop> LoopPtr;
    
    

}
// end namespace
//---------------------------------------------------------------------

#endif