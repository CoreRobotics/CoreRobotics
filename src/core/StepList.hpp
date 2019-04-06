/*
 * Copyright (c) 2017-2019, CoreRobotics
 * www.corerobotics.org
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef CR_STEP_LIST_HPP_
#define CR_STEP_LIST_HPP_

#include "Loop.hpp"
#include "Step.hpp"
#include <vector>

namespace cr {
namespace core {

//! StepList shared pointer
class StepList;
typedef std::shared_ptr<StepList> StepListPtr;

//---------------------------------------------------------------------
/*!
 \class StepList
 \ingroup core

 \brief
 This class implements a list of individual step() classes (i.e. a
 a directed graph when signals are included)

 \details
 This class implements a step list.  Individual Step() classes are
 executed in the order in which they have been added to the list.
 */
//---------------------------------------------------------------------
class StepList : public Step {

//! Constructor and Destructor
public:

  //! Class constructor
  StepList();

  //! Class destructor
  virtual ~StepList();

  //! Create
  static StepListPtr create();

//! API
public:

  //! Attach a step item to the list of vertices
  void attach(StepPtr i_vertex);

  // Derived step function
public:

  //! step the graph
  void step();

//! Private members
private:

  //! list of step elements
  std::vector<StepPtr> m_vertices;
};

}  // namepsace core
}  // namespace cr

#endif
