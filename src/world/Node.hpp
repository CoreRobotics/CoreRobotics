/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#ifndef CR_NODE_HPP_
#define CR_NODE_HPP_

#include "core/Item.hpp"
#include "physics/Frame.hpp"
#include <memory>
#include <vector>

namespace cr {
namespace world {

//! Node shared pointer
class Node;
typedef std::shared_ptr<Node> NodePtr;

//------------------------------------------------------------------------------
/*!
 \class Node
 \ingroup world

 \brief
 This class implements a basic world scene graph item for attaching to
 a Origin thread element or other Nodes.

 \details

 */
//------------------------------------------------------------------------------
class Node : public std::enable_shared_from_this<Node>, public core::Item {

  // Constructor and Destructor
public:
  //! Class constructor
  Node();

  //! Class destructor
  ~Node();

  //! Create a pointer
  static NodePtr create();

  // Tree graph controls
public:
  //! add a child to the list of children
  void addChild(NodePtr i_item);

  //! remove a child from the list of children
  core::Result removeChild(NodePtr i_item);

  //! set the parent of the item
  void setParent(NodePtr i_item);

  //! get the parent of the child
  NodePtr getParent();

  //! check if a node is an ancestor (parent at any level) of this node
  bool isAncestor(NodePtr i_node);

  //! check if a node is an descendant (child at any level) of this node
  bool isDescendent(NodePtr i_node);

  //! return if the item is a leaf (i.e. no children)
  bool isLeaf();

  //! return if the item is a root (i.e. no parent)
  bool isRoot();

  //! return the depth of the node (0 = root)
  unsigned getDepth();

  // Frame transformation controls
public:
  //! set the local frame transformation
  void setLocalTransform(const physics::Frame &i_frame);

  //! return the local frame transformation
  virtual physics::Frame getLocalTransform();

  //! return the global frame transformation
  virtual physics::Frame getGlobalTransform();

  //! return the relative frame transformation
  virtual physics::Frame getRelativeTransform(NodePtr i_item);

  // Print details
public:
  //! print the scene
  virtual void print(std::ostream &i_stream);

  // protected member data
protected:
  //! Transformation
  physics::Frame m_frame;

  //! parent item
  NodePtr m_parent = NULL;

  //! list of children
  std::vector<NodePtr> m_children;

  //! type (read only)
  std::string m_type = "Node";
};

} // namespace world
} // namespace cr

#endif
