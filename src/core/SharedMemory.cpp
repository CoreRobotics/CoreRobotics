/*
 * Copyright (c) 2017-2019, CoreRobotics
 * www.corerobotics.org
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 */

#include "SharedMemory.hpp"
#include <iostream>

//---------------------------------------------------------------------
// Begin namespace
namespace cr {
namespace core {

//---------------------------------------------------------------------
/*!
 The constructor defines a thread.\n

 \param[in] i_memoryName - name of the memory
 \param[in] i_role - Shared memory role.  Only Servers can create shared memory
 */
//---------------------------------------------------------------------
SharedMemory::SharedMemory(const char *i_memoryName, ManagerRole i_role) {
  // static const char* name = i_memoryName;
  m_name = i_memoryName;

  // push the role
  m_role = i_role;

  // Create a new segment with given name and size
  if (m_role == CR_MANAGER_SERVER) {

    // Remove shared memory on construction
    boost::interprocess::shared_memory_object::remove(m_name);

    //! Create the managed shared memory
    m_segment = new boost::interprocess::managed_shared_memory(
        boost::interprocess::create_only, m_name, 65536);

    // Initialize shared memory STL-compatible allocator
    m_alloc_inst = new ShmemAllocator(m_segment->get_segment_manager());

  } else {

    // Open shared memory
    m_segment = new boost::interprocess::managed_shared_memory(
        boost::interprocess::open_only, m_name);
  }
}

//---------------------------------------------------------------------
/*!
 The destructor frees up memory.\n
 */
//---------------------------------------------------------------------
SharedMemory::~SharedMemory() {

  // Remove shared memory on destruction
  boost::interprocess::shared_memory_object::remove(m_name);

  // Remove the allocator
  if (m_role == CR_MANAGER_SERVER) {
    delete m_alloc_inst;
  }

  // remove the segment
  delete m_segment;
}

//---------------------------------------------------------------------
/*!
 This method adds a signal to the shared memory.

 \param [in] i_signalName - name of the signal to add.
 \param [in] i_data - intial data vector to add to memory.
 */
//---------------------------------------------------------------------
void SharedMemory::addSignal(const char *i_signalName, Eigen::VectorXd i_data) {

  // Todo: query the signals existing in the local vector
  // find i_signalName in m_signals
  // if does not exist then create and add
  // if does exist, return a Result that indicates the variable wasn't created

  // Construct a vector named "MyVector" in shared memory with argument
  // alloc_inst
  ShmemData *myvector =
      m_segment->construct<ShmemData>(i_signalName)(*m_alloc_inst);

  // Insert data in the vector
  for (int i = 0; i < i_data.size(); ++i)
    myvector->push_back(i_data(i));
}

//---------------------------------------------------------------------
/*!
 This method removes a signal from the shared memory.

 \param [in] i_signalName - name of the signal to remove.
 */
//---------------------------------------------------------------------
void SharedMemory::removeSignal(const char *i_signalName) {

  // destroy the signal
  m_segment->destroy<ShmemData>(i_signalName);
}

//---------------------------------------------------------------------
/*!
 This method sets signal values in the shared memory.

 \param [in] i_signalName - name of the signal to set.
 \param [in] i_data - data to set to vector.
 */
//---------------------------------------------------------------------
void SharedMemory::set(const char *i_signalName, Eigen::VectorXd i_data) {

  // find
  ShmemData *myvector = m_segment->find<ShmemData>(i_signalName).first;

  // Insert data in the vector
  for (int i = 0; i < i_data.size(); ++i)
    myvector->at(i) = i_data(i);
}

//---------------------------------------------------------------------
/*!
 This method sets signal values in the shared memory.

 \param [in] i_signalName - name of the signal
 \return - vector value of the memory
 */
//---------------------------------------------------------------------
Eigen::VectorXd SharedMemory::get(const char *i_signalName) {

  // initialize output
  Eigen::VectorXd data;

  // find
  ShmemData *myvector = m_segment->find<ShmemData>(i_signalName).first;
  int n = myvector->size();

  // data
  data.setZero(n);
  for (int i = 0; i < n; i++) {
    data(i) = myvector->at(i);
  }
  return data;
}
}
}
// end namespace
//---------------------------------------------------------------------
