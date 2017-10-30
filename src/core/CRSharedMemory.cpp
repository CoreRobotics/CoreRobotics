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

#include "CRSharedMemory.hpp"

//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics {

//=====================================================================
//! A couple type definitions
typedef allocator<double, managed_shared_memory::segment_manager>  ShmemAllocator;
typedef vector<double, ShmemAllocator> Signal;
    

//=====================================================================
/*!
 The constructor defines a thread.\n

 \param[in] i_memoryName - name of the memory
 \param[in] i_role - Shared memory role.  Only Servers can create shared memory
 */
//---------------------------------------------------------------------
CRSharedMemory::CRSharedMemory(const char* i_memoryName,
                               CRManagerRole i_role)
{
    // Remove shared memory on construction and destruction
    struct shm_remove
    {
        shm_remove() { shared_memory_object::remove("i_memoryName"); }
        ~shm_remove(){ shared_memory_object::remove("i_memoryName"); }
    } remover;
    
    // Create a new segment with given name and size
    if (i_role == CR_MANAGER_SERVER){
        managed_shared_memory* m_segment = new managed_shared_memory(create_only, i_memoryName, 65536);
    } else if (i_role == CR_MANAGER_CLIENT){
        managed_shared_memory* m_segment = new managed_shared_memory(open_only, i_memoryName);
    }
}


//=====================================================================
/*!
 The destructor frees up memory.\n
 */
//---------------------------------------------------------------------
CRSharedMemory::~CRSharedMemory() {
    
    delete m_segment;
    
}

    

//=====================================================================
/*!
 This method adds a signal to the shared memory.
 
 \param [in] i_signalName - name of the signal to add.
 \param [in] i_data - intial data to add to vector.
 */
//---------------------------------------------------------------------
void CRSharedMemory::addSignal(const char* i_signalName, CRSignal i_data) {
    
    // Initialize shared memory STL-compatible allocator
    const ShmemAllocator alloc_inst (m_segment->get_segment_manager());
    
    //Construct a vector named "MyVector" in shared memory with argument alloc_inst
    Signal *myvector = m_segment->construct<Signal>(i_signalName)(alloc_inst);
    
    // add time
    myvector->push_back(i_data.time);
    
    //Insert data in the vector
    for(int i = 0; i < i_data.data.size(); ++i)
        myvector->push_back(i_data.data(i));
}
    

//=====================================================================
/*!
 This method removes a signal from the shared memory.
 
 \param [in] i_signalName - name of the signal to remove.
 */
//---------------------------------------------------------------------
void CRSharedMemory::removeSignal(const char* i_signalName) {
    
    m_segment->destroy<Signal>(i_signalName);
}
    
    

//=====================================================================
/*!
 This method sets signal values in the shared memory.
 
 \param [in] i_signalName - name of the signal to set.
 \param [in] i_data - data to set to vector.
 */
//---------------------------------------------------------------------
void CRSharedMemory::set(const char* i_signalName, CRSignal i_data) {
    
    // find
    Signal *myvector = m_segment->find<Signal>(i_signalName).first;
    
    // add time
    myvector->at(0) = i_data.time;
    
    //Insert data in the vector
    for(int i = 0; i < i_data.data.size(); ++i)
        myvector->at(i+1) = i_data.data(i);
}
    
    
    
//=====================================================================
/*!
 This method sets signal values in the shared memory.
 
 \param [in] i_signalName - name of the signal
 \return - value of the memory
 */
//---------------------------------------------------------------------
CRSignal CRSharedMemory::get(const char* i_signalName) {
    CRSignal sig;
    
    // find
    Signal *myvector = m_segment->find<Signal>(i_signalName).first;
    int n = myvector->size();
    
    // time
    sig.time = myvector->at(0);
    sig.data.setZero(n-1);
    
    // data
    for (int k = 0; k < n; k++){
        sig.data(k) = myvector->at(k+1);
    }
    
    return sig;
}





//=====================================================================
// End namespace
}


