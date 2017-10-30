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

#ifndef CRSharedMemory_hpp
#define CRSharedMemory_hpp

//=====================================================================
// Includes
#include "CRTypes.hpp"
#include "Eigen/Dense"
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/allocators/allocator.hpp>


//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics {
    
//=====================================================================
/*!
 \file CRSharedMemory.hpp
 \brief Implements interprocess shared memory management.
 */
//---------------------------------------------------------------------
/*!
 \class CRSharedMemory
 \ingroup core
 
 \brief This class implements
 
 \details
 ## Description
 CRThread implements a simple thread interface for setting a callback
 and starting and stopping thread execution.
 
 - CRSharedMemory::setCallback
 
 ## Example
 This example shows use of the CRSharedMemory class.
 \include test_CRSharedMemory.cpp
 */
//=====================================================================
//! Enumerator for specifying thread priority
enum CRManagerRole {
    CR_MANAGER_SERVER,
    CR_MANAGER_CLIENT,
};
//=====================================================================
//! Enumerator for specifying thread priority
struct CRSignal {
    double time;
    Eigen::VectorXd data;
};
//=====================================================================
using namespace boost::interprocess;
//=====================================================================
class CRSharedMemory {
    
//---------------------------------------------------------------------
// Constructor and Destructor
public:
    
    //! Class constructor
    CRSharedMemory(const char* i_memoryName, CRManagerRole i_role);
    
    //! Class destructor
    virtual ~CRSharedMemory();
    
    
//---------------------------------------------------------------------
// Public Methods
public:
    
    //! Add a signal to the shared memory
    void addSignal(const char* i_signalName, CRSignal i_data);
    
    //! Remove a signal from the shared memory
    void removeSignal(const char* i_signalName);
    
    //! Set the signal value in shared memory
    void set(const char* i_signalName, CRSignal i_data);
    
    //! Set the signal value in shared memory
    CRSignal get(const char* i_signalName);
    
    
//---------------------------------------------------------------------
// Protected Members
private:

	//! shared memory segment manager
    managed_shared_memory* m_segment;
    
};
//=====================================================================
// End namespace
}

#endif
