/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#ifndef CR_ORIGIN_HPP_
#define CR_ORIGIN_HPP_

#include "Node.hpp"
#include "core/Item.hpp"
#include <memory>

namespace cr {
namespace world {

//! Origin shared pointer
class Origin;
typedef std::shared_ptr<Origin> OriginPtr;

//------------------------------------------------------------------------------
/*!
 \class Origin
 \ingroup world

 \brief
 This class implements a scene graph node that represents the world
 origin, and thus cannot have a parent.

 \details

 */
//------------------------------------------------------------------------------
class Origin : public core::Item {

public:
  //! Class constructor
  Origin();

  //! Class destructor
  ~Origin();

  //! Create a pointer
  static OriginPtr create();

  //! set the name
  virtual void setName(std::string i_name);

  //! add a child to the list of children
  void addChild(NodePtr i_item) { m_rootItem->addChild(i_item); }

  //! remove a child from the list of children
  void removeChild(NodePtr i_item) { m_rootItem->removeChild(i_item); }

  //! print out the scene
  virtual void print(std::ostream &i_stream);

private:
  //! add a world item
  NodePtr m_rootItem;

  //! type (read only)
  std::string m_type = "Origin";
};

} // namespace world
} // namespace cr

//! Origin display operator overload
// https://stackoverflow.com/questions/476272/how-to-properly-overload-the-operator-for-an-ostream
// std::ostream& operator<<(std::ostream&, const cr::Origin&);

#endif
