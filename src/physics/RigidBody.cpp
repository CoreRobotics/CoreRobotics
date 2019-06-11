/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#include "RigidBody.hpp"

namespace cr {
namespace physics {

//------------------------------------------------------------------------------
/*!
 Constructor to the RigidBody.\n
 */
//------------------------------------------------------------------------------
RigidBody::RigidBody() {
  m_com << 0, 0, 0;
  m_mass = 0;
  m_inertia.setZero();
}

//------------------------------------------------------------------------------
/*!
 Constructor to the RigidBody.\n
 */
//------------------------------------------------------------------------------
RigidBody::~RigidBody() {}

//------------------------------------------------------------------------------
/*!
 Return the mass matrix.\n

 \return        6 x 6 mass matrix
 */
//------------------------------------------------------------------------------
Eigen::Matrix<double, 6, 6> RigidBody::getMassMatrix() {
  Eigen::Matrix<double, 6, 6> Mc;
  Mc.setZero();
  Eigen::Matrix3d mu;
  mu.setIdentity();
  mu = m_mass * mu;
  Mc.topLeftCorner(3, 3) = mu;
  Mc.bottomRightCorner(3, 3) = m_inertia;
  return Mc;
}

} // namespace physics
} // namespace cr
