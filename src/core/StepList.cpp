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
StepList::~StepList() {
  onStop();
  m_vertices.clear();
}

//---------------------------------------------------------------------
/*!
 This function adds a step element to the list.\n
 */
//---------------------------------------------------------------------
void StepList::attach(std::shared_ptr<Step> i_element) {
  m_vertices.push_back(i_element);
}

//---------------------------------------------------------------------
/*!
 This function steps the list of elements.\n
 */
//---------------------------------------------------------------------
void StepList::step() {
  for (unsigned i = 0; i < m_vertices.size(); i++) {
    m_vertices.at(i)->step();
  }
}

//---------------------------------------------------------------------
/*!
 This function starts the list of elements.\n
 */
//---------------------------------------------------------------------
void StepList::onStart() {
  for (unsigned i = 0; i < m_vertices.size(); i++) {
    m_vertices.at(i)->onStart();
  }
}

//---------------------------------------------------------------------
/*!
 This function starts the list of elements.\n
 */
//---------------------------------------------------------------------
void StepList::onStop() {
  for (unsigned i = 0; i < m_vertices.size(); i++) {
    m_vertices.at(i)->onStop();
  }
}

} // namespace core
} // namespace cr
