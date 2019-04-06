/*
 * Copyright (c) 2017-2019, CoreRobotics
 * www.corerobotics.org
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 */

#include "StepList.hpp"

namespace cr {
namespace core {

//---------------------------------------------------------------------
/*!
 The constructor sets up the StepList.\n
 */
//---------------------------------------------------------------------
StepList::StepList() {}

//---------------------------------------------------------------------
/*!
 The destructor closes the StepList.\n
 */
//---------------------------------------------------------------------
StepList::~StepList() { m_vertices.clear(); }

//---------------------------------------------------------------------
/*!
 Create a new StepList.\n
 */
//---------------------------------------------------------------------
StepListPtr StepList::create() { return std::make_shared<StepList>(); }

//---------------------------------------------------------------------
/*!
 This function adds a step element to the list.\n
 */
//---------------------------------------------------------------------
void StepList::attach(StepPtr i_element) { m_vertices.push_back(i_element); }

//---------------------------------------------------------------------
/*!
 This function steps the list of elements.\n
 */
//---------------------------------------------------------------------
void StepList::step() {

  // step each of the vertices
  for (unsigned i = 0; i < m_vertices.size(); i++) {
    m_vertices.at(i)->step();
  }
}

}  // namepsace core
}  // namespace cr
