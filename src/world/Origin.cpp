/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#include "Origin.hpp"

//------------------------------------------------------------------------------
// Begin namespace
namespace cr {
namespace world {

//------------------------------------------------------------------------------
/*!
 The constructor sets up the world.\n
 */
//------------------------------------------------------------------------------
Origin::Origin() : m_rootItem(std::make_shared<Node>()) {
  // m_rootItem = std::make_shared<Node>();
}

//------------------------------------------------------------------------------
/*!
 The destructor deletes the world.\n
 */
//------------------------------------------------------------------------------
Origin::~Origin() {}

//------------------------------------------------------------------------------
/*!
 Create a whole new world.\n
 */
//------------------------------------------------------------------------------
OriginPtr Origin::create() { return std::make_shared<Origin>(); }

//------------------------------------------------------------------------------
/*!
 Set the name.\n

 \param[in]     i_name - the name to set
 */
//------------------------------------------------------------------------------
void Origin::setName(const std::string &i_name) {
  Item::setName(i_name);
  m_rootItem->setName(i_name + " root");
}

//------------------------------------------------------------------------------
/*!
 Print the output scene.\n
 */
//------------------------------------------------------------------------------
void Origin::printInfo(std::ostream &i_stream) {
  i_stream << "\nTree graph structure from world::Origin\n\n";
  m_rootItem->printInfo(i_stream);
  i_stream << "\n";
}

} // namespace world
} // namespace cr

//------------------------------------------------------------------------------
/*!
 Display operator details to an output stream.\n
 */
//------------------------------------------------------------------------------
/*
std::ostream& operator<<(std::ostream& os, cr::OriginPtr obj)
{
    os << "obj->getName()" << "\n";
    return os;
}
 */
