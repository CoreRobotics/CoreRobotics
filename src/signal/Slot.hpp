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

#ifndef CR_SLOT_HPP_
#define CR_SLOT_HPP_

#include "Signal.hpp"
#include "Step.hpp"
#include <mutex>
#include <memory>

//---------------------------------------------------------------------
// Begin namespace
namespace cr {
namespace signal {

    
//---------------------------------------------------------------------
/*!
 \class Slot
 \ingroup signal
 
 \brief
 
 \details
 
 */
//---------------------------------------------------------------------
template <typename DataType, typename ReceiverType>
class Slot : public core::Step
{
    
//! constructor
public:
    
    //! constructor
    Slot(std::shared_ptr<Requester<DataType>> i_signal,
         ReceiverType* i_receiver,
         void(ReceiverType::*i_callback)(DataType)) {
        m_signal = i_signal;
        m_receiver = static_cast<void*>(i_receiver);
        m_callback = reinterpret_cast<void(ReceiverType::*)(void*)>(i_callback);
    }
    
    //! Create a new slot
    static std::shared_ptr<Slot<DataType, ReceiverType>> create(
            std::shared_ptr<Requester<DataType>> i_signal,
            ReceiverType* i_receiver,
            void(ReceiverType::*i_callback)(DataType)) {
        return std::make_shared<Slot<DataType, ReceiverType>>(i_signal, i_receiver, i_callback);
    }
    
    
// public access functions
public:
    
    //! get the receiver
    ReceiverType* getReceiver() { return static_cast<ReceiverType*>(m_receiver); }
    
    //! step the data push
    virtual void step() {
        ReceiverType* c = static_cast<ReceiverType*>(m_receiver);
        void(ReceiverType::*fcn)(DataType) = reinterpret_cast<void(ReceiverType::*)(DataType)>(m_callback);
        DataType d = m_signal->request();
        m_mutex.lock();
        (c->*fcn)( d );
        m_mutex.unlock();
    }
    
    
// inherited access members
private:
        
    //! Receiver
    void* m_receiver;
    
    //! callback request functions
    void(ReceiverType::*m_callback)(void*);
    
    //! signal
    std::shared_ptr<Requester<DataType>> m_signal;
    
    //! mutex member
    std::recursive_mutex m_mutex;
    
};
    

}
}
// end namespace
//---------------------------------------------------------------------

#endif
