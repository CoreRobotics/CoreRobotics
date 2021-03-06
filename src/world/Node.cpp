/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#include "Node.hpp"

namespace cr {
namespace world {

//------------------------------------------------------------------------------
/*!
 Constructor to the world item.\n
 */
//------------------------------------------------------------------------------
Node::Node() {}

//------------------------------------------------------------------------------
/*!
 Destructor to the world item.\n
 */
//------------------------------------------------------------------------------
Node::~Node() {
  // delete m_parent;
}

//------------------------------------------------------------------------------
/*!
 Create a new world item.\n
 */
//------------------------------------------------------------------------------
NodePtr Node::create() { return std::make_shared<Node>(); }

//------------------------------------------------------------------------------
/*!
 This function adds an item to the list of children

 \param[in]     i_item - the child to add
 */
//------------------------------------------------------------------------------
void Node::addChild(NodePtr i_item) {
  i_item->setParent(shared_from_this());
  m_children.push_back(i_item);
}

//------------------------------------------------------------------------------
/*!
 This function removes an item from the list of children

 \param[in]     i_item - the child to remove
 \return        result of the remove operation
                (CR_RESULT_SUCCESS or CR_RESULT_NOT_FOUND)
 */
//------------------------------------------------------------------------------
core::Result Node::removeChild(NodePtr i_item) {
  for (unsigned k = 0; k < m_children.size(); k++) {
    if (m_children.at(k) == i_item) {
      m_children.erase(m_children.begin() + k);
      i_item->setParent(NULL);
      return core::CR_RESULT_SUCCESS;
    }
  }
  return core::CR_RESULT_NOT_FOUND;
}

//------------------------------------------------------------------------------
/*!
 This function sets the parent of the item.

 \param[in]     i_item - the parent item
 */
//------------------------------------------------------------------------------
void Node::setParent(NodePtr i_item) { m_parent = i_item; }

//------------------------------------------------------------------------------
/*!
 This function return the parent of the item.

 \return        the parent item
 */
//------------------------------------------------------------------------------
NodePtr Node::getParent() { return m_parent; }

//------------------------------------------------------------------------------
/*!
 Check if a node is an ancestor (parent at any level) of this node.

 \param[in]     i_node - the node to check as ancestor of this node
 \return        flag indicating relationship status
 */
//------------------------------------------------------------------------------
bool Node::isAncestor(NodePtr i_node) {
  NodePtr parent = getParent();
  while (parent != NULL) {
    if (parent == i_node) {
      return true;
    }
    parent = parent->getParent();
  }
  return false;
}

//------------------------------------------------------------------------------
/*!
 Check if a node is an descendant (child at any level) of this node.

 \param[in]     i_node - the node to check as descendant of this node
 \return        flag indicating relationship status
 */
//------------------------------------------------------------------------------
bool Node::isDescendent(NodePtr i_node) {
  return i_node->isAncestor(shared_from_this());
}

//------------------------------------------------------------------------------
/*!
 This function returns a flag indicating if it is a leaf (i.e. no children).

 \return        flag (true = if there are no children)
 */
//------------------------------------------------------------------------------
bool Node::isLeaf() {
  bool isLeaf = true;
  if (m_children.size() > 0)
    isLeaf = false;
  return isLeaf;
}

//------------------------------------------------------------------------------
/*!
 This function returns a flag indicating if it is a root (i.e. no parent).

 \return        flag (true = if there is no parent)
 */
//------------------------------------------------------------------------------
bool Node::isRoot() {
  bool isRoot = false;
  if (m_parent == NULL)
    isRoot = true;
  return isRoot;
}

//------------------------------------------------------------------------------
/*!
 This function returns the depth of the node in the tree

 \return        integer depth (0 = root)
 */
//------------------------------------------------------------------------------
unsigned Node::getDepth() {
  unsigned d = 0;
  NodePtr parent = getParent();
  while (parent != NULL) {
    d++;
    parent = parent->getParent();
  }
  return d;
}

//------------------------------------------------------------------------------
/*!
 This function sets the local frame transformation (relative to
 the parent frame).

 \param[in]     i_frame - the local frame transformation
 */
//------------------------------------------------------------------------------
void Node::setLocalTransform(const physics::Frame &i_frame) {
  m_frame = i_frame;
}

//------------------------------------------------------------------------------
/*!
 This function gets the local frame transformation (relative to
 the parent frame).

 \return        the local frame transformation
 */
//------------------------------------------------------------------------------
physics::Frame Node::getLocalTransform() { return m_frame; }

//------------------------------------------------------------------------------
/*!
 This function gets the global frame transformation (relative to
 the world frame).

 \return        the local frame transformation
 */
//------------------------------------------------------------------------------
physics::Frame Node::getGlobalTransform() {
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
physics::Frame Node::getRelativeTransform(NodePtr i_item) {
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
void Node::printInfo(std::ostream &i_stream) {
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
  i_stream << "+ [" << id << "] world::Node '" << getName() << "'\n";
  for (unsigned i = 0; i < m_children.size(); i++) {
    m_children.at(i)->printInfo(i_stream);
  }
}

} // namespace world
} // namespace cr
