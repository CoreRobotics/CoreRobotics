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

#ifndef CR_SIGNAL_HPP_
#define CR_SIGNAL_HPP_

#include "Item.hpp"
#include "Utility.hpp"
#include <memory>
#include <mutex>
#include <fstream>

//---------------------------------------------------------------------
// Begin namespace
namespace cr {
namespace signal {
    
    
    
//---------------------------------------------------------------------
/*!
 \class SignalBase
 \ingroup signal
 
 \brief
 
 \details
 
 */
//---------------------------------------------------------------------
class SignalBase : public core::Item
{
    
    // public methods
    public:
    
        //! constructor
        SignalBase() {};
    
        //! get the signal dimension
        virtual unsigned size() = 0;
    
        //! print signal value to ostream
        virtual void print(std::ostream& i_log) = 0;
    
    
    // private members
    private:
    
        //! type (read only)
        std::string m_type = "Signal";
    
};
    
    
    
//---------------------------------------------------------------------
/*!
 \class Requester
 \ingroup signal
 
 \brief
 
 \details
 
 */
//---------------------------------------------------------------------
template <typename DataType>
class Requester : public std::enable_shared_from_this<Requester<DataType>>, public SignalBase
{

    // public methods
    public:
    
        //! constructor
        Requester() {};
    
        //! request function to implement
        virtual DataType request() = 0;
    
};
    
    
    
//---------------------------------------------------------------------
/*!
 \class Signal
 \ingroup signal
 
 \brief
 
 \details
 
 */
//---------------------------------------------------------------------
template <typename DataType, typename EmitterType>
class Signal : public Requester<DataType>
{
    
    // Constructor public access functions
    public:
    
        //! constructor
        Signal(std::shared_ptr<EmitterType> i_emitter,
               DataType(EmitterType::*i_callback)()) {
            // m_emitter = static_cast<void*>(i_emitter);
            m_emitter = i_emitter;
            m_callback = reinterpret_cast<void*(EmitterType::*)()>(i_callback);
        }
    
        //! Create a new signal
        static std::shared_ptr<Signal<DataType, EmitterType>> create(
            std::shared_ptr<EmitterType> i_emitter,
            DataType(EmitterType::*i_callback)()) {
                return std::make_shared<Signal<DataType, EmitterType>>(i_emitter, i_callback);
        }
    
    
    // public access functions
    public:
    
        //! get the parent
        // EmitterType* getEmitter() { return static_cast<EmitterType*>(m_emitter); }
        std::shared_ptr<EmitterType> getEmitter() { return m_emitter; }
    
        //! request data
        virtual DataType request() {
            // EmitterType* p = static_cast<EmitterType*>(m_emitter);
            DataType(EmitterType::*fcn)() = reinterpret_cast<DataType(EmitterType::*)()>(m_callback);
            m_mutex.lock();
            // DataType d = (p->*fcn)();
            DataType d = (m_emitter.get()->*fcn)();
            m_mutex.unlock();
            return d;
        }
    
    
    // signal base functions
    public:
    
        //! get the signal dimension
        virtual unsigned size() {
            DataType d = request();
            return Utility::size(d);
        }
    
        //! print signal value to ostream
        virtual void print(std::ostream& i_log) {
            DataType d = request();
            Utility::write(i_log, d);
        }
    
    
    
    //! private members
    private:
    
        //! Parent
        // void* m_emitter;
        std::shared_ptr<EmitterType> m_emitter;
    
        //! callback request functions
        void*(EmitterType::*m_callback)();
    
        //! mutex member
        std::recursive_mutex m_mutex;
    
};

    
    
}
}
// end namespace
//---------------------------------------------------------------------

#endif

