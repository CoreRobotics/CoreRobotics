/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#include "Link.hpp"

namespace cr {
namespace world {

//------------------------------------------------------------------------------
/*!
 Constructor to the world item.\n
 */
//------------------------------------------------------------------------------
Link::Link() : m_comItem(std::make_shared<Node>()) {
  m_comItem->setName("COM");
}

//------------------------------------------------------------------------------
/*!
 Destructor to the world item.\n
 */
//------------------------------------------------------------------------------
Link::~Link() {}

//------------------------------------------------------------------------------
/*!
 Create a new world item.\n
 */
//------------------------------------------------------------------------------
LinkPtr Link::create() { return std::make_shared<Link>(); }

//------------------------------------------------------------------------------
/*!
 This function sets the rigid body parameters and updates internal data.

 \param[in]        i_body - the RigidBody data
 */
//------------------------------------------------------------------------------
void Link::setRigidBody(const physics::RigidBody &i_body) {
  addChild(m_comItem);
  m_body = i_body;
  physics::Frame f;
  f.setTranslation(m_body.getCenterOfMass());
  m_comItem->setLocalTransform(f);
}

//------------------------------------------------------------------------------
/*!
 This function gets the local frame transformation (relative to
 the parent frame).

 \return        the local frame transformation
 */
//------------------------------------------------------------------------------
physics::Frame Link::getLocalTransform() {
  physics::Frame f;
  Eigen::Matrix3d r = m_frame.getRotation();
  Eigen::Vector3d t = m_frame.getTranslation();
  f.setRotationAndTranslation(r, t);
  return f;
}

//------------------------------------------------------------------------------
/*!
 This function gets the global frame transformation (relative to
 the world frame).

 \return        the local frame transformation
 */
//------------------------------------------------------------------------------
physics::Frame Link::getGlobalTransform() {
  Eigen::Matrix4d T = m_frame.getTransformToParent();
  NodePtr parent = m_parent;
  while (parent != NULL) {
    T = parent->getLocalTransform().getTransformToParent() * T;
    parent = parent->getParent();
  }
  physics::Frame f;
  f.setRotationAndTranslation(T.topLeftCorner(3, 3), T.topRightCorner(3, 1));
  return f;
}

//------------------------------------------------------------------------------
/*!
 This function gets a relative frame transformation (relative to
 the frame of the intput item).

 \return        the relative frame transformation
 */
//------------------------------------------------------------------------------
physics::Frame Link::getRelativeTransform(NodePtr i_item) {
  Eigen::Matrix4d T0 = this->getGlobalTransform().getTransformToParent();
  Eigen::Matrix4d T = i_item->getGlobalTransform().getTransformToChild() * T0;
  physics::Frame f;
  f.setRotationAndTranslation(T.topLeftCorner(3, 3), T.topRightCorner(3, 1));
  return f;
}

//------------------------------------------------------------------------------
/*!
 Print the children.\n
 */
//------------------------------------------------------------------------------
void Link::printInfo(std::ostream &i_stream) {
  unsigned d = getDepth();
  for (unsigned i = 0; i < d; i++) {
    i_stream << "  ";
  }
  std::string id = "N";
  if (isLeaf()) {
    id = "L";
  } else if (isRoot()) {
    id = "R";
  }
  std::string dof = "-";
  if (getDegreeOfFreedom() == physics::CR_EULER_FREE_ANG_A) {
    dof = "A";
  } else if (getDegreeOfFreedom() == physics::CR_EULER_FREE_ANG_B) {
    dof = "B";
  } else if (getDegreeOfFreedom() == physics::CR_EULER_FREE_ANG_G) {
    dof = "G";
  } else if (getDegreeOfFreedom() == physics::CR_EULER_FREE_POS_X) {
    dof = "X";
  } else if (getDegreeOfFreedom() == physics::CR_EULER_FREE_POS_Y) {
    dof = "Y";
  } else if (getDegreeOfFreedom() == physics::CR_EULER_FREE_POS_Z) {
    dof = "Z";
  }
  i_stream << "+ [" << id << "] world::Link DOF = " << dof << " '" << getName()
           << "'\n";
  for (unsigned i = 0; i < m_children.size(); i++) {
    m_children.at(i)->printInfo(i_stream);
  }
}

} // namespace world
} // namespace cr
