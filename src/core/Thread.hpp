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

#ifndef CR_THREAD_HPP_
#define CR_THREAD_HPP_

#include <thread>

//---------------------------------------------------------------------
// Begin namespace
namespace cr {
namespace core {
    

//! Enumerator for specifying thread priority
enum ThreadPriority {
    CR_PRIORITY_LOWEST,
    CR_PRIORITY_LOW,
    CR_PRIORITY_NORMAL,
    CR_PRIORITY_HIGH,
    CR_PRIORITY_HIGHEST
};


//---------------------------------------------------------------------
/*!
 \class Thread
 \ingroup core
 
 \brief This class implements threads to enable running multiple
 control loops within a single application.
 
 \details
 ## Description
 Thread implements a simple thread interface for setting a callback
 and starting and stopping thread execution.
 
 - Thread::setCallback sets the callback function.
 - Thread::setPriority sets the priority of the thread.
 - Thread::join waits for the thread to join the main thread.
 - Thread::stop detach detaches the thread from the main thread.
 
 ## Example
 This example creates and runs a simple Thread.
 \include example_Core.cpp
 */
//---------------------------------------------------------------------
class Thread {
    

    // Constructor and Destructor
    public:
    
        //! Class constructor
        Thread();
        Thread(ThreadPriority i_priority);
    
        //! Class destructor
        virtual ~Thread();
    
    
    // Public Methods
    public:
    
        //! Set the thread callback function
        void setCallback(void(i_callbackFunction)(void));
    
        //! Set the thread callback function
        void setCallback(void(i_callbackFunction)(void*), void* arg);

        //! Set the thread priority
        void setPriority(ThreadPriority i_priority);
    
        //! Waits for a thread to finish its execution
        void join();
    
        //! Permits the thread to execute independently from the thread handle
        void detach();
    
    
    // Protected Members
    private:

        //! thread pointer
        std::thread* m_loop;
    
};
    
    
}
}
// end namespace
//---------------------------------------------------------------------


#endif
