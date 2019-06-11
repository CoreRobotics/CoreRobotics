/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#ifndef CR_PHYSICS_RIGID_BODY_HPP_
#define CR_PHYSICS_RIGID_BODY_HPP_

#include "Frame.hpp"

namespace cr {
namespace physics {

//------------------------------------------------------------------------------
/*!
 \class RigidBody
 \ingroup physics

 \brief This class is a container for rigid body transformations and
 dynamic properties necessary for multibody representations.

 \details
 ## Description
 RigidBody implements a container for representing rigid body
 dynamics.  At a minimum it points to a cr::Frame class or
 derived subclass.

 ## Example
 This example creates an Rigid body object.
 \include example_Manipulator.cpp

 ## References
 [1] J. Craig, "Introduction to Robotics: Mechanics and Control", Ed. 3,
 Pearson, 2004.
 */
//------------------------------------------------------------------------------
class RigidBody {

  // Constructor and Destructor
public:
  //! Class constructor
  RigidBody();

  //! Class destructor
  virtual ~RigidBody();

  // Get/Set Methods
public:
  //! Set the center of mass
  void setCenterOfMass(const Eigen::Vector3d &i_com) { m_com = i_com; }

  //! Get the center of mass
  Eigen::Vector3d getCenterOfMass() { return m_com; }

  //! Set the inertia tensor
  void setInertiaTensor(const Eigen::Matrix3d &i_inertia) {
    m_inertia = i_inertia;
  }

  //! Get the inertia tensor
  Eigen::Matrix3d getInertiaTensor() { return m_inertia; }

  //! Set the mass value
  void setMass(const double &i_mass) { m_mass = i_mass; }

  //! Get the mass value
  double getMass() { return m_mass; }

  //! get the mass matrix
  Eigen::Matrix<double, 6, 6> getMassMatrix();

  // Private Members
private:
  //! center of mass
  Eigen::Vector3d m_com;

  //! inertia tensor
  Eigen::Matrix3d m_inertia;

  //! mass value
  double m_mass;
};

} // namespace physics
} // namespace cr

#endif
