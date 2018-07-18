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

#ifndef CR_CONNECTION_HPP_
#define CR_CONNECTION_HPP_

#include "Step.hpp"
#include <mutex>
#include <memory>

//---------------------------------------------------------------------
// Begin namespace
namespace cr {
namespace signal {

    
//---------------------------------------------------------------------
/*!
 \class Connection
 \ingroup signal
 
 \brief
 
 \details
 
 */
//---------------------------------------------------------------------
template <typename DataType, typename EmitterType, typename ReceiverType>
class Connection : public core::Step
{
    
//! Constructor
public:
    
    //! constructor
    Connection(EmitterType* i_emitter,
               DataType(EmitterType::*i_emitterCallback)(),
               ReceiverType* i_receiver,
               void(ReceiverType::*i_receiverCallback)(DataType)) {
        
        m_emitter = static_cast<void*>(i_emitter);
        m_receiver = static_cast<void*>(i_receiver);
        m_emitterCallback = reinterpret_cast<void*(EmitterType::*)()>(i_emitterCallback);
        m_receiverCallback = reinterpret_cast<void(ReceiverType::*)(void*)>(i_receiverCallback);
    }
    
    
// public access functions
public:
    
    //! get the emitter
    EmitterType* getEmitter() { return static_cast<EmitterType*>(m_emitter); }
    
    //! get the receiver
    ReceiverType* getReceiver() { return static_cast<ReceiverType*>(m_receiver); }
    
    //! request data
    DataType request() {
        EmitterType* c = static_cast<EmitterType*>(m_emitter);
        DataType(EmitterType::*fcn)() = reinterpret_cast<DataType(EmitterType::*)()>(m_emitterCallback);
        return (c->*fcn)();
    }
    
    //! step the data push
    void step() {
        ReceiverType* c = static_cast<ReceiverType*>(m_receiver);
        void(ReceiverType::*fcn)(DataType) = reinterpret_cast<void(ReceiverType::*)(DataType)>(m_receiverCallback);
        m_mutex.lock();
        (c->*fcn)( request() );
        m_mutex.unlock();
    }
    
    //! reset does nothing
    void reset() {};
    
    
//! Inherited access members
private:
    
    //! Emitter
    void* m_emitter;
        
    //! Receiver
    void* m_receiver;
        
    //! callback request functions
    void*(EmitterType::*m_emitterCallback)();
    
    //! callback request functions
    void(ReceiverType::*m_receiverCallback)(void*);
    
    //! mutex member
    std::recursive_mutex m_mutex;
    
};
    

}
}
// end namespace
//---------------------------------------------------------------------

#endif
