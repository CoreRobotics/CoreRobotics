//=====================================================================
/*
Software License Agreement (BSD-3-Clause License)
Copyright (c) 2019, CoreRobotics.
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
#include <iostream>

//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics  {

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
    // static const char* name = i_memoryName;
    m_name = i_memoryName;
    
    // push the role
    m_role = i_role;
    
    // Create a new segment with given name and size
    if (m_role == CR_MANAGER_SERVER){
        
        // Remove shared memory on construction
        boost::interprocess::shared_memory_object::remove(m_name);
        
        //! Create the managed shared memory
        m_segment = new boost::interprocess::managed_shared_memory(boost::interprocess::create_only,
                                                                   m_name,
                                                                   65536);
        
        // Initialize shared memory STL-compatible allocator
        m_alloc_inst = new ShmemAllocator(m_segment->get_segment_manager());
        
    } else {
        
        // Open shared memory
        m_segment = new boost::interprocess::managed_shared_memory(boost::interprocess::open_only,
                                                                   m_name);
    }
}

//=====================================================================
/*!
 The destructor frees up memory.\n
 */
//---------------------------------------------------------------------
CRSharedMemory::~CRSharedMemory() {
    
    // Remove shared memory on destruction
    boost::interprocess::shared_memory_object::remove(m_name);
    
    // Remove the allocator
    if (m_role == CR_MANAGER_SERVER){
        delete m_alloc_inst;
    }
    
    // remove the segment
    delete m_segment;
}
    

//=====================================================================
/*!
 This method adds a signal to the shared memory.
 
 \param [in] i_signalName - name of the signal to add.
 \param [in] i_data - intial data vector to add to memory.
 */
//---------------------------------------------------------------------
void CRSharedMemory::addSignal(const char* i_signalName,
                               Eigen::VectorXd i_data) {
    
    // Todo: query the signals existing in the local vector
    // find i_signalName in m_signals
    // if does not exist then create and add
    // if does exist, return a CRResult that indicates the variable wasn't created
    
    // Construct a vector named "MyVector" in shared memory with argument alloc_inst
    Signal *myvector = m_segment->construct<Signal>(i_signalName)(*m_alloc_inst);
    
    //Insert data in the vector
    for(int i = 0; i < i_data.size(); ++i)
        myvector->push_back(i_data(i));
}
    

//=====================================================================
/*!
 This method removes a signal from the shared memory.
 
 \param [in] i_signalName - name of the signal to remove.
 */
//---------------------------------------------------------------------
void CRSharedMemory::removeSignal(const char* i_signalName) {
    
    // destroy the signal
    m_segment->destroy<Signal>(i_signalName);
}
    
    

//=====================================================================
/*!
 This method sets signal values in the shared memory.
 
 \param [in] i_signalName - name of the signal to set.
 \param [in] i_data - data to set to vector.
 */
//---------------------------------------------------------------------
void CRSharedMemory::set(const char* i_signalName,
                         Eigen::VectorXd i_data) {
    
    // find
    Signal *myvector = m_segment->find<Signal>(i_signalName).first;
    
    //Insert data in the vector
    for(int i = 0; i < i_data.size(); ++i)
        myvector->at(i) = i_data(i);
}
    
    
    
//=====================================================================
/*!
 This method sets signal values in the shared memory.
 
 \param [in] i_signalName - name of the signal
 \return - vector value of the memory
 */
//---------------------------------------------------------------------
Eigen::VectorXd CRSharedMemory::get(const char* i_signalName) {
    
    // initialize output
    Eigen::VectorXd data;
    
    // find
    Signal *myvector = m_segment->find<Signal>(i_signalName).first;
    int n = myvector->size();
    
    // data
    data.setZero(n);
    for (int i = 0; i < n; i++){
        data(i) = myvector->at(i);
    }
    return data;
}



//=====================================================================
// End namespace
}


