//=====================================================================
/*
Software License Agreement (BSD-3-Clause License)
Copyright (c) 2019, CoreRobotics.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.

* Neither the name of CoreRobotics nor the names of its contributors
may be used to endorse or promote products derived from this
software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

\project CoreRobotics Project
\url     www.corerobotics.org
\author  Parker Owan

*/
//=====================================================================

#ifndef CRFrameEuler_hpp
#define CRFrameEuler_hpp

//=====================================================================
// Includes
#include "CRFrame.hpp"
#include "Eigen/Dense"
#include "core/CRTypes.hpp"

//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics {

//=====================================================================
/*!
 \file CRFrameEuler.hpp
 \brief Implements a derived frame class for handling Euler rotations
 in a 3D affine transformation.
 */
//---------------------------------------------------------------------
/*!
 \class CRFrameEuler
 \ingroup physics

 \brief This class implements a homogeneous transformation in the
 special Euclidean group made up of rotations and translations, where
 the rotations are defined by Euler angles.

 \details
 ## Description
 CRFrameEuler implements an transformation where the rotation is
 specified by three Euler angles (<a href="wikipedia.org/wiki/Euler_angles">
 see Wikipedia article).</a>

 Properties of the transform can be set with methods
 - CRFrameEuler::setPosition, CRFrameEuler::setOrientation and
 CRFrameEuler::setPositionAndOrientation set transformation values.
 - CRFrameEuler::setMode sets the Euler angle convention (options are in
 CoreRobotics::CREulerMode).

 The free variable can be specified by the CRFrameEuler::m_freeVar member
 (options are in CoreRobotics::CREulerFreeVariable).

 ## Example
 This example creates an Euler frame class.
 \include example_CRManipulator.cpp

 ## References
 [1] J. Craig, "Introduction to Robotics: Mechanics and Control", Ed. 3,
 Pearson, 2004.

 */
//=====================================================================
//! Enumerator for handling Euler angle free variable candidates
#ifndef SWIG
enum[[deprecated(CR_DEPRECATED)]] CREulerFreeVariable {
#else
enum CREulerFreeVariable {
#endif
  CR_EULER_FREE_NONE, CR_EULER_FREE_POS_X, CR_EULER_FREE_POS_Y,
      CR_EULER_FREE_POS_Z, CR_EULER_FREE_ANG_A, CR_EULER_FREE_ANG_B,
      CR_EULER_FREE_ANG_G,
};

//=====================================================================
#ifndef SWIG
class[[deprecated(CR_DEPRECATED)]] CRFrameEuler : public CRFrame {
#else
class CRFrameEuler : public CRFrame {
#endif

  //---------------------------------------------------------------------
  // Constructor and Destructor
public:
  //! Class constructor
  CRFrameEuler(double i_x, double i_y, double i_z, double i_a, double i_b,
               double i_g, CREulerMode i_mode, CREulerFreeVariable i_free);
  CRFrameEuler();

  //---------------------------------------------------------------------
  // Get/Set Methods
public:
  //! Set the value of the free variable
  CRResult setFreeValue(double i_q);

  //! Get the value of the free variable
  double getFreeValue(void);

  //! Set the free variable
  void setFreeVariable(CREulerFreeVariable i_free) { m_freeVar = i_free; }

  //! Get the free variable
  CREulerFreeVariable getFreeVariable(void) { return m_freeVar; }

  //! Set the Euler convention
  void setMode(CREulerMode i_mode);

  //! Get the Euler convention
  CREulerMode getMode(void);

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

  //---------------------------------------------------------------------
  // Public Methods
public:
  //! Query if the frame is driven, i.e. has a free variable
  bool isDriven(void);

  //---------------------------------------------------------------------
  // Private Members
private:
  //! free variable indicator
  CREulerFreeVariable m_freeVar;

  //! Euler convention
  CREulerMode m_eulerMode;

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

  //---------------------------------------------------------------------
  // Private Methods
private:
  //! Update the rotation and translation matrices from data
  // using CRFrame::setRotationAndTranslation;
  void setRotationAndTranslation();
};

//=====================================================================
// End namespace
}

#endif
