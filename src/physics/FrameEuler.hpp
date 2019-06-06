/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#ifndef CR_PHYSICS_FRAME_EULER_HPP_
#define CR_PHYSICS_FRAME_EULER_HPP_

#include "Eigen/Dense"
#include "Frame.hpp"

namespace cr {
namespace physics {

//! Enumerator for handling Euler angle free variable candidates
enum EulerFreeVariable {
  CR_EULER_FREE_NONE,
  CR_EULER_FREE_POS_X,
  CR_EULER_FREE_POS_Y,
  CR_EULER_FREE_POS_Z,
  CR_EULER_FREE_ANG_A,
  CR_EULER_FREE_ANG_B,
  CR_EULER_FREE_ANG_G,
};

//------------------------------------------------------------------------------
/*!
 \class FrameEuler
 \ingroup physics

 \brief This class implements a homogeneous transformation in the
 special Euclidean group made up of rotations and translations, where
 the rotations are defined by Euler angles.

 \details
 ## Description
 FrameEuler implements an transformation where the rotation is
 specified by three Euler angles (<a href="wikipedia.org/wiki/Euler_angles">
 see Wikipedia article).</a>

 Properties of the transform can be set with methods
 - FrameEuler::setPosition, FrameEuler::setOrientation and
 FrameEuler::setPositionAndOrientation set transformation values.
 - FrameEuler::setMode sets the Euler angle convention (options are in
 cr::EulerMode).

 The free variable can be specified by the FrameEuler::m_freeVar member
 (options are in cr::EulerFreeVariable).

 ## Example
 This example creates an Euler frame class.
 \include example_Manipulator.cpp

 ## References
 [1] J. Craig, "Introduction to Robotics: Mechanics and Control", Ed. 3,
 Pearson, 2004.

 */
//------------------------------------------------------------------------------

class FrameEuler : public Frame {

  // Constructor and Destructor
public:
  //! Class constructor
  FrameEuler(double i_x, double i_y, double i_z, double i_a, double i_b,
             double i_g, EulerMode i_mode, EulerFreeVariable i_free);
  FrameEuler();

  // Get/Set Methods
public:
  //! Set the value of the free variable
  core::Result setFreeValue(double i_q);

  //! Get the value of the free variable
  double getFreeValue(void);

  //! Set the free variable
  void setFreeVariable(EulerFreeVariable i_free) { m_freeVar = i_free; }

  //! Get the free variable
  EulerFreeVariable getFreeVariable(void) { return m_freeVar; }

  //! Set the Euler convention
  void setMode(EulerMode i_mode);

  //! Get the Euler convention
  EulerMode getMode(void);

  //! Set the position values
  void setPosition(double i_x, double i_y, double i_z);

  //! Get the x position
  void getPosition(double &o_x, double &o_y, double &o_z);

  //! Set the angle values
  void setOrientation(double i_a, double i_b, double i_g);

  //! Get the angle values
  void getOrientation(double &o_a, double &o_b, double &o_g);

  //! Set the position and angle values
  void setPositionAndOrientation(double i_x, double i_y, double i_z, double i_a,
                                 double i_b, double i_g);

  //! Get the position and angle values
  void getPositionAndOrientation(double &o_x, double &o_y, double &o_z,
                                 double &o_a, double &o_b, double &o_g);

  //! (NOT RECOMMENDED) Set the rotation and translation matrices
  void setRotationAndTranslation(Eigen::Matrix3d i_rot,
                                 Eigen::Vector3d i_trans);

  // Public Methods
public:
  //! Query if the frame is driven, i.e. has a free variable
  bool isDriven(void);

  // Private Members
private:
  //! free variable indicator
  EulerFreeVariable m_freeVar;

  //! Euler convention
  EulerMode m_eulerMode;

  //! x position
  double m_posX;

  //! y position
  double m_posY;

  //! z position
  double m_posZ;

  //! alpha angle [rad]
  double m_angA;

  //! beta angle [rad]
  double m_angB;

  //! gamma angle [rad]
  double m_angG;

  // Private Methods
private:
  //! Update the rotation and translation matrices from data
  // using Frame::setRotationAndTranslation;
  void setRotationAndTranslation();
};

} // namespace physics
} // namespace cr

#endif
