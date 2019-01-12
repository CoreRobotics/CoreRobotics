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

#ifndef CRMutex_hpp
#define CRMutex_hpp

//=====================================================================
// Includes
#include <mutex>

//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics {
    
//=====================================================================
/*!
 \file CRMutex.hpp
 \brief Implements a thread mutex.
 */
//---------------------------------------------------------------------
/*!
 \class CRMutex
 \ingroup core
 
 \brief This class implements a mutex to ensure mutually exclusive
 writing of data to thread data.
 
 \details
 ## Description
 CRMutex implements a mutex to for deterministic data transfer between
 threads.
 
 - CRMutex::lock locks the thread from access.
 - CRMutex::unlock releases the thread for access.
 */

//=====================================================================
class CRMutex {
    
//---------------------------------------------------------------------
// Constructor and Destructor
public:
    
    //! Class constructor
    CRMutex();
    
    //! Class destructor
    ~CRMutex();
    
    
//---------------------------------------------------------------------
// Public Methods
public:
    
    //! Acquire the mutex
    void lock();

	//! Release the mutex
	void unlock();
    
    
//---------------------------------------------------------------------
// Protected Members
private:

	//! thread pointer
    std::recursive_mutex* m_mutex;
    
    
};
//=====================================================================
// End namespace
}

#endif