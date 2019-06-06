/*
 * Copyright (c) 2017-2019, CoreRobotics
 * www.corerobotics.org
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef CR_CORE_SHARED_MEMORY_HPP_
#define CR_CORE_SHARED_MEMORY_HPP_

#include "Eigen/Dense"
#include "Types.hpp"

#define BOOST_DATE_TIME_NO_LIB
#include "boost/interprocess/allocators/allocator.hpp"
#include "boost/interprocess/containers/vector.hpp"
#include "boost/interprocess/managed_shared_memory.hpp"

namespace cr {
namespace core {

//! Enumerator for specifying thread priority
enum ManagerRole {
  CR_MANAGER_SERVER,
  CR_MANAGER_CLIENT,
};

//! allocator typedef
typedef boost::interprocess::allocator<
    double, boost::interprocess::managed_shared_memory::segment_manager>
    ShmemAllocator;

//! data typedef
typedef boost::interprocess::vector<double, ShmemAllocator> ShmemData;

//---------------------------------------------------------------------
/*!
 \class SharedMemory
 \ingroup core

 \brief This class implements interprocess shared memory to write
 and read vectors across processes.  SharedMemory implements a simple
 shared memory interface with the following methods available.

 - SharedMemory::addSignal adds a signal to the memory
 - SharedMemory::removeSignal removes a signal from the memory
 - SharedMemory::set sets the value in shared memory
 - SharedMemory::get returns the value from shared memory
 */
//---------------------------------------------------------------------
class SharedMemory {

  //! Constructor and Destructor
public:
  //! Class constructor
  SharedMemory(const char *i_memoryName, ManagerRole i_role);

  //! Class destructor
  virtual ~SharedMemory();

  //! API
public:
  //! Add a signal to the shared memory
  void addSignal(const char *i_signalName, Eigen::VectorXd i_data);

  //! Remove a signal from the shared memory
  void removeSignal(const char *i_signalName);

  //! Set the signal value in shared memory
  void set(const char *i_signalName, Eigen::VectorXd i_data);

  //! Set the signal value in shared memory
  Eigen::VectorXd get(const char *i_signalName);

  //! Private Members
private:
  //! shared memory segment manager
  boost::interprocess::managed_shared_memory *m_segment;

  //! shared memory allocator
  const ShmemAllocator *m_alloc_inst;

  //! shared memory type
  ManagerRole m_role;

  //! name of the memory
  const char *m_name;
};

} // namespace core
} // namespace cr

#endif
