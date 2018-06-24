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
//=====================================================================

#ifndef CRThreadLoop_hpp
#define CRThreadLoop_hpp

//=====================================================================
// Includes
#include <thread>
#include "CRClock.hpp"
#include "CRTypes.hpp"


//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics {
    
//=====================================================================
/*!
 \file CRThreadLoop.hpp
 \brief Implements looped thread execution, which includes soft
 real-time control and start/stop functionality.
 */
//---------------------------------------------------------------------
    
//---------------------------------------------------------------------
/*!
 \class CRThreadLoopElement
 \ingroup core
 
 \brief This abstract class defines the methods needed to derive a
 call for a CRThreadLoop.
 
 \details
 ## Description
 This abstract class defines the methods needed to derive a
 call for a CRThreadLoop.  The primary methods to be implemented are:
 
 - CRThreadLoopElement::step() is called on each iteration of the thread
 while the thread is running.
 - CRThreadLoopElement::setPriority sets the priority of the thread.
 
 ## Example
 This example creates and runs a simple CRThreadLoop.
 \include example_CRThreadLoop.cpp
 */
//=====================================================================
class CRThreadLoop;
class CRThreadLoopElement {
    
    // Constructor and destructor
    public:
    
        //! constructor
        CRThreadLoopElement() {}
    
        //! destructor
        ~CRThreadLoopElement() { delete m_parent; }
    
    
    // Primary CRThreadLoopElement functions that must be implemented
    public:
    
        //! Step the element - called on each iteration of the thread while running
        virtual void step() = 0;
    
        //! Reset the element - called on ThreadLoop::start())
        virtual void reset() = 0;
    
    
    // Other methods called by the parent thread
    public:
    
        //! set the pointer to the parent - called by the thread on construction
        void setParent(CRThreadLoop* i_parent) {m_parent = i_parent;}
    
    
    // Protected members
    protected:
    
        //! thread parent
        CRThreadLoop* m_parent = NULL;
    
};
    
    
//---------------------------------------------------------------------
/*!
 \class CRThreadLoop
 \ingroup core
 
 \brief This class implements looped thread execution, which includes
 soft real-time control and start/stop functionality.
 
 \details
 ## Description
 CRThreadLoop implements a thread loop controller class, which includes
 the following methods:
 
 - CRThreadLoop::start starts the thread loop element.
 - CRThreadLoop::pause pauses the loop element execution, but does not
 exit the thread of execution.
 - CRThreadLoop::stop exits the thread of execution.
 - CRThreadLoop::stop stops the thread execution.
 
 ## Example
 This example creates and runs a simple CRThreadLoop.
 \include example_CRThreadLoop.cpp
 */
//=====================================================================
class CRThreadLoop {
    
    // Constructor and Destructor
    public:
    
        //! Class constructor
        CRThreadLoop(CRThreadLoopElement* i_member);
    
        //! Class destructor
        virtual ~CRThreadLoop();
    
    
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
        CRThreadLoopElement* m_element;
    
        //! simulation state
        CRRunState m_runState = CR_RUN_STATE_STOPPED;
    
        //! thread
        // CoreRobotics::CRThread* m_thread = NULL;
        std::thread* m_thread = NULL;
    
        //! global timer
        CRClock m_timer;
    
        //! thread update rate (s)
        double m_updateRate;
    
        //! amount of time spent in pause since first start from stop (s).
        double m_t0 = 0;
    
        //! time at last pause() call
        double m_tPaused = 0;
    
    
};
    
    
//--------------------------------------------------------------------------
/*!
 Smart pointer to CRThreadLoop.
 */
//--------------------------------------------------------------------------
typedef std::shared_ptr<CRThreadLoop> CRThreadLoopPtr;
    
    
//=====================================================================
// End namespace
}

#endif
