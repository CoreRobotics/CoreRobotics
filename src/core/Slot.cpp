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

#include "Slot.hpp"

//---------------------------------------------------------------------
// Begin namespace
namespace CoreRobotics {
    
    
//---------------------------------------------------------------------
/*!
 The constructor sets up the slot.\n
 */
//---------------------------------------------------------------------
template <typename DataType, typename ParentType>
Slot<DataType, ParentType>::Slot(ParentType* i_parent,
           DataType(ParentType::*i_callback)())
{
    m_parent = static_cast<void*>(i_parent);
    m_callback = reinterpret_cast<void*(ParentType::*)()>(i_callback);
}

//--------------------------------------------------------------------------
/*!
 The destructor closes the slot.\n
 */
//--------------------------------------------------------------------------
template <typename DataType, typename ParentType>
Slot<DataType, ParentType>::~Slot(){
    delete m_parent;
    delete m_callback;
}

//--------------------------------------------------------------------------
/*!
 This function returns the pointer to the parent.\n
 */
//--------------------------------------------------------------------------
template <typename DataType, typename ParentType>
ParentType* Slot<DataType, ParentType>::getParent(){
    return static_cast<ParentType*>(m_parent);
}

//--------------------------------------------------------------------------
/*!
 This function requests signal data.\n
 */
//--------------------------------------------------------------------------
template <typename DataType, typename ParentType>
DataType Slot<DataType, ParentType>::request()
{
    ParentType* p = static_cast<ParentType*>(m_parent);
    DataType(ParentType::*fcn)() = reinterpret_cast<DataType(ParentType::*)()>(m_callback);
    return (p->*fcn)();
}
    
         
}
// End namespace
//------------------------------------------------------------------------------
