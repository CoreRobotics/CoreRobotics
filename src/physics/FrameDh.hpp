/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#ifndef CR_FRAMEDH_HPP_
#define CR_FRAMEDH_HPP_

#include "Eigen/Dense"
#include "Frame.hpp"

namespace cr {
namespace physics {

//! Enumerator for handling DH parameter free variable candidates
enum DhFreeVariable {
  CR_DH_FREE_NONE,
  CR_DH_FREE_R,
  CR_DH_FREE_ALPHA,
  CR_DH_FREE_D,
  CR_DH_FREE_THETA,
};

//! Enumerator for handling DH conventions
enum DhMode {
  CR_DH_MODE_CLASSIC,
  CR_DH_MODE_MODIFIED,
};

//------------------------------------------------------------------------------
/*!
 \class FrameDh
 \ingroup physics

 \brief This class implements a homogeneous transformation in the
 special Euclidean group made up of rotations and translations, where
 the rotations and translations are defined by DH parameters.

 \details
 ## Description
 FrameDh implements a DH parameter transformation specified by 4
 parameters (<a href="wikipedia.org/wiki/Denavit–Hartenberg_parameters">
 see Wikipedia article).</a> With an additional 5th paramter for constant
 offsets for the driven variable.

 Properties of the transform can be set with methods
 - FrameDh::setParameters sets the DH parameter values.
 - FrameDh::setMode sets the DH convention (options are in
 cr::DhMode).

 The free variable can be specified by the FrameDh::m_freeVar member
 (options are in cr::DhFreeVariable).

 ## Example
 This example creates a DH parameter frame class.
 \include example_FrameDh.cpp

 ## References
 [1] J. Denavit and R. Hartenberg, "A kinematic notation for lower-pair
 mechanisms based on matrices". Trans ASME J. Appl. Mech. 23, pp. 215-221, 1955.

 [2] J. Craig, "Introduction to Robotics: Mechanics and Control", Ed. 3,
 Pearson, 2004.

 */
//------------------------------------------------------------------------------
class FrameDh : public Frame {

  // Constructor and Destructor
public:
  //! Class constructor
  FrameDh(double i_r, double i_alpha, double i_d, double i_theta, DhMode i_mode,
          DhFreeVariable i_free);
  FrameDh();

  // Get/Set Methods
public:
  //! Set the value of the free variable
  core::Result setFreeValue(double i_q);

  //! Get the value of the free variable
  double getFreeValue(void);

  //! Set the free variable
  void setFreeVariable(DhFreeVariable i_free) { m_freeVar = i_free; }

  //! Get the free variable
  DhFreeVariable getFreeVariable(void) { return m_freeVar; }

  //! Set the DH convention
  void setMode(DhMode i_mode);

  //! Get the DH convention
  DhMode getMode(void);

  //! Set the DH parameter values
  void setParameters(double i_r, double i_alpha, double i_d, double i_theta);

  //! Get the DH parameter values
  void getParameters(double &o_r, double &o_alpha, double &o_d,
                     double &o_theta);

  // Public Methods
public:
  //! Query if the frame is driven, i.e. has a free variable
  bool isDriven(void);

  // Private Members
private:
  //! free variable indicator
  DhFreeVariable m_freeVar;

  //! DH convention mode
  DhMode m_dhMode;

  //! r parameter
  double m_dhR;

  //! d parameter
  double m_dhD;

  //! alpha angle parameter [rad]
  double m_dhAlpha;

  //! theta angle parameter [rad]
  double m_dhTheta;

  //! free variable offset parameter [m] or [rad]
  // double m_freeVarOffset;

  // Private Methods
private:
  //! Update the rotation and translation matrices from data
  using Frame::setRotationAndTranslation;
  void setRotationAndTranslation();
};

} // namespace physics
} // namespace cr

#endif
