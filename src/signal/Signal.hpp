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

#include <memory>

//---------------------------------------------------------------------
// Begin namespace
namespace cr {
namespace signal {
    
    
//---------------------------------------------------------------------
/*!
 \class Signal
 \ingroup signal
 
 \brief
 
 \details
 
 */
//---------------------------------------------------------------------
template <typename DataType, typename EmitterType>
class Signal
{
    
// Constructor public access functions
public:
    
    //! constructor
    Signal(EmitterType* i_parent,
           DataType(EmitterType::*i_callback)()) {
        m_parent = static_cast<void*>(i_parent);
        m_callback = reinterpret_cast<void*(EmitterType::*)()>(i_callback);
    }
    
// public access functions
public:
    
    //! get the parent
    EmitterType* getParent() { return static_cast<EmitterType*>(m_parent); }
    
    //! request data
    DataType request() {
        EmitterType* p = static_cast<EmitterType*>(m_parent);
        DataType(EmitterType::*fcn)() = reinterpret_cast<DataType(EmitterType::*)()>(m_callback);
        return (p->*fcn)();
    }
    
//! private members
private:
    
    //! Parent
    void* m_parent;
    
    //! callback request functions
    void*(EmitterType::*m_callback)();
    
};

    
    
}
}
// end namespace
//---------------------------------------------------------------------

#endif

