/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#ifndef CR_LINK_HPP_
#define CR_LINK_HPP_

#include "Node.hpp"
#include "physics/FrameEuler.hpp"
#include "physics/RigidBody.hpp"
#include <memory>
#include <vector>

namespace cr {
namespace world {

//! Link shared pointer
class Link;
typedef std::shared_ptr<Link> LinkPtr;

//------------------------------------------------------------------------------
/*!
 \class Link
 \ingroup world

 \brief
 This class implements a link item for capturing driven manipulator
 kinematics and dynamics.

 \details

 */
//------------------------------------------------------------------------------
class Link : public Node {

public:
  //! Class constructor
  Link();

  //! Class destructor
  ~Link();

  //! Create a pointer
  static LinkPtr create();

  //! get the pointer to the center of mass node
  NodePtr getCenterOfMass() { return m_comItem; }

  //! set the rigid body parameters
  void setRigidBody(const physics::RigidBody &i_body);

  //! return the rigid body parameters
  physics::RigidBody getRigidBody() { return m_body; }

  //! set the degree of freedom
  void setDegreeOfFreedom(physics::EulerFreeVariable i_dof) {
    m_frame.setFreeVariable(i_dof);
  }

  //! return the degree of freedom
  physics::EulerFreeVariable getDegreeOfFreedom() {
    return m_frame.getFreeVariable();
  }

  //! set the driven value
  void setFreeValue(const double &i_value) { m_frame.setFreeValue(i_value); }

  //! return the driven value
  double getFreeValue() { return m_frame.getFreeValue(); }

  //! set the driven velocity
  void setFreeVelocity(const double &i_velocity) { m_velocity = i_velocity; }

  //! return the driven velocity
  double getFreeVelocity() { return m_velocity; }

  //! set euler mode
  void setEulerMode(const physics::EulerMode &i_mode) {
    m_frame.setMode(i_mode);
  }

  //! get euler mode
  physics::EulerMode getEulerMode() { return m_frame.getMode(); }

  //! set the local frame transformation
  void setLocalTransform(const physics::FrameEuler &i_frame) {
    m_frame = i_frame;
  }

  //! return the local frame transformation
  virtual physics::Frame getLocalTransform();

  //! return the global frame transformation
  virtual physics::Frame getGlobalTransform();

  //! return the relative frame transformation
  virtual physics::Frame getRelativeTransform(NodePtr i_item);

  //! print the scene
  virtual void print(std::ostream &i_stream);

protected:
  //! Rigid body
  physics::RigidBody m_body;

  //! frame - overloaded from Node::m_frame
  physics::FrameEuler m_frame;

  //! free value velocity
  double m_velocity = 0;

  //! center of mass node
  NodePtr m_comItem;

  //! type (read only)
  std::string m_type = "Link";
};

} // namespace world
} // namespace cr

#endif
