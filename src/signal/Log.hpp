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

#ifndef CR_LOG_HPP_
#define CR_LOG_HPP_

#include "Step.hpp"
#include "Clock.hpp"
#include "Signal.hpp"
#include <fstream>
#include <string>
#include <vector>


//---------------------------------------------------------------------
// Begin namespace
namespace cr {
namespace signal {


//! Node shared pointer
class Log;
typedef std::shared_ptr<Log> LogPtr;
    

//---------------------------------------------------------------------
/*!
 \class Log
 \ingroup signal
 
 \brief This class implements a signal logger.
 
 \details
 ## Description
 
 */
//---------------------------------------------------------------------
class Log
    : public core::Step, public core::Item
{

    // Constructor and Destructor
    public:
    
        //! Class constructor
        Log();
    
        //! destructor
        ~Log() {};
    
        //! create a new log
        static LogPtr create();
    

    // signal log controls
    public:
    
        //! add a signal to the list
        void add(std::shared_ptr<SignalBase> i_signal);
    
    
    // Step method
    public:
    
        //! step the signal log
        virtual void step();
    
        //! start the signal log
        virtual void onStart();
    
        //! stop the signal log
        virtual void onStop();
    

    // Protected members
    protected:
    
        //! list of signals to log
        // std::vector<Requester<SupportedSignalTypes>*> m_signals;
        std::vector<std::shared_ptr<SignalBase>> m_signals;
    
        //! filename
        std::string m_filename;
    
        //! timer
        core::Clock m_timer;
    
        // ofstream object
        std::ofstream m_logFile;
    
        //! type (read only)
        std::string m_type = "Log";
    
};


}
}
// end namespace
//---------------------------------------------------------------------


#endif

