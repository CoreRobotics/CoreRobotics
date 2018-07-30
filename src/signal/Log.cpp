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

#include "Log.hpp"

//---------------------------------------------------------------------
// Begin namespace
namespace cr {
namespace signal {
    
    
//---------------------------------------------------------------------
/*!
 The constructor creates a log.\n
 */
//---------------------------------------------------------------------
Log::Log()
{
    
}
    
    
//---------------------------------------------------------------------
/*!
 Create a new signal log item.\n
 */
//---------------------------------------------------------------------
LogPtr Log::create(){
    return std::make_shared<Log>();
}
    
    
//---------------------------------------------------------------------
/*!
 This method adds a signal to the list.
 
 \param[in]     i_signal - the signal to add
 */
//---------------------------------------------------------------------
void Log::add(std::shared_ptr<SignalBase> i_signal)
{
    m_signals.push_back(i_signal);
}


//---------------------------------------------------------------------
/*!
 This method steps the signal log.
 */
//---------------------------------------------------------------------
void Log::step()
{
    // get the time
    double t = m_timer.getElapsedTime();
    
    // write the time
    Utility::write(m_logFile, t);
    
    // now write each signal value
    for (int i = 0; i < m_signals.size(); i++) {
        m_signals.at(i)->print(m_logFile);
    }
    
    // write the new line
    m_logFile << "\n";
}
    
    
//---------------------------------------------------------------------
/*!
 This method starts the signal log.
 */
//---------------------------------------------------------------------
void Log::onStart()
{
    // open the file
	m_logFile.open( m_name );
	std::cout << "Log::open() called\n";
    
    // write the time signal
    std::string str = "time";
    Utility::write(m_logFile, str);
    
    // write each signal header
    for (int i = 0; i < m_signals.size(); i++) {
        unsigned n = m_signals.at(i)->size();
        for (int j = 0; j < n; j++) {
            std::string index = "[" + std::to_string(j) + "]";
            Utility::write(m_logFile, m_signals.at(i)->getName().append(index));
        }
    }
    
    // write the new line
    m_logFile << "\n";
    
    // start the internal clock
    m_timer.startTimer();
}
    
    
//---------------------------------------------------------------------
/*!
 This method stops the signal log.
 */
//---------------------------------------------------------------------
void Log::onStop()
{
    m_logFile.close();
	std::cout << "Log::close() called\n";
}
    


}
}
// end namespace
//---------------------------------------------------------------------
