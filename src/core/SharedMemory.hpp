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

#ifndef CR_SHARED_MEMORY_HPP_
#define CR_SHARED_MEMORY_HPP_

#include "Types.hpp"
#include "Eigen/Dense"

#define BOOST_DATE_TIME_NO_LIB
#include "boost/interprocess/managed_shared_memory.hpp"
#include "boost/interprocess/containers/vector.hpp"
#include "boost/interprocess/allocators/allocator.hpp"

//---------------------------------------------------------------------
// Begin namespace
namespace cr {
namespace core {

    
//! Enumerator for specifying thread priority
enum ManagerRole {
    CR_MANAGER_SERVER,
    CR_MANAGER_CLIENT,
};
    
    
//! allocator typedef
typedef boost::interprocess::allocator<double, boost::interprocess::managed_shared_memory::segment_manager>  ShmemAllocator;
    
    
//! data typedef
typedef boost::interprocess::vector<double, ShmemAllocator> ShmemData;
    
    
//---------------------------------------------------------------------
/*!
 \class SharedMemory
 \ingroup core
 
 \brief This class implements interprocess shared memory to easily write
 and read data across processes.  This is a simple method for
 communicating between sensors and different programming languages.
 
 \details
 ## Description
 SharedMemory implements a simple shared memory interface with the
 following methods available.
 
 - SharedMemory::addSignal adds a signal to the memory
 - SharedMemory::removeSignal removes a signal from the memory
 - SharedMemory::set sets the value in shared memory
 - SharedMemory::get returns the value from shared memory
 
 ## Example
 This example shows use of the SharedMemory class.
 \include example_Core.cpp
 */
//---------------------------------------------------------------------
class SharedMemory {
    
    // Constructor and Destructor
    public:
    
        //! Class constructor
        SharedMemory(const char* i_memoryName,
                       ManagerRole i_role);
    
        //! Class destructor
        virtual ~SharedMemory();
    
    
    // Public Methods
    public:
    
        //! Add a signal to the shared memory
        void addSignal(const char* i_signalName,
                       Eigen::VectorXd i_data);
    
        //! Remove a signal from the shared memory
        void removeSignal(const char* i_signalName);
    
        //! Set the signal value in shared memory
        void set(const char* i_signalName,
                 Eigen::VectorXd i_data);
    
        //! Set the signal value in shared memory
        Eigen::VectorXd get(const char* i_signalName);
    

    // Protected Members
    private:

        //! shared memory segment manager
        boost::interprocess::managed_shared_memory* m_segment;
    
        //! shared memory allocator
        const ShmemAllocator* m_alloc_inst;
    
        //! shared memory type
        ManagerRole m_role;
    
        //! name of the memory
        const char* m_name;
    
};
    
    
}
}
// end namespace
//---------------------------------------------------------------------

#endif
