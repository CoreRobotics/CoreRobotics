/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#include "FrameDh.hpp"
#include "Eigen/Dense"

namespace cr {
namespace physics {

//------------------------------------------------------------------------------
/*!
 The constructor sets the rotation and translation parameters upon
 construction, with defaults listed in parenthesis.\n

 \param[in]   i_r      - x position of the frame (0)
 \param[in]   i_alpha  - y position of the frame (0)
 \param[in]   i_d      - z position of the frame (0)
 \param[in]   i_theta  - alpha angle of the frame [rad] (0)
 \param[in]   i_mode   - DH convention (CR_DH_MODE_MODIFIED)
 \param[in]   i_free   - free variable (CR_DH_FREE_NONE)
 */
//------------------------------------------------------------------------------
FrameDh::FrameDh(double i_r, double i_alpha, double i_d, double i_theta,
                 DhMode i_mode, DhFreeVariable i_free) {
  m_dhR = i_r;
  m_dhAlpha = i_alpha;
  m_dhD = i_d;
  m_dhTheta = i_theta;
  m_dhMode = i_mode;
  m_freeVar = i_free;
  this->setRotationAndTranslation();
}
FrameDh::FrameDh() {
  m_dhR = 0.0;
  m_dhAlpha = 0.0;
  m_dhD = 0.0;
  m_dhTheta = 0.0;
  m_dhMode = CR_DH_MODE_MODIFIED;
  m_freeVar = CR_DH_FREE_NONE;
  this->setRotationAndTranslation();
}

//------------------------------------------------------------------------------
/*!
 This method sets the value of the free variable.  The method returns a
 true if the value was written and a false if m_freeVar is set to
 CR_DH_FREE_NONE.\n

 \param[in] i_q - value of the variable to be set
 \return - Result flag indicating if the parameter is writable
 */
//------------------------------------------------------------------------------
core::Result FrameDh::setFreeValue(double i_q) {
  core::Result result = core::CR_RESULT_SUCCESS;
  switch (m_freeVar) {
  case CR_DH_FREE_NONE:
    result = core::CR_RESULT_UNWRITABLE;
    break;
  case CR_DH_FREE_R:
    m_dhR = i_q;
    break;
  case CR_DH_FREE_ALPHA:
    m_dhAlpha = i_q;
    break;
  case CR_DH_FREE_D:
    m_dhD = i_q;
    break;
  case CR_DH_FREE_THETA:
    m_dhTheta = i_q;
    break;
  }
  this->setRotationAndTranslation();
  return result;
}

//------------------------------------------------------------------------------
/*!
 This method get the value of the free variable.  The method returns
 q = 0 if m_freeVar is set to CR_DH_FREE_NONE.\n

 \return - value of the free variable.
 */
//------------------------------------------------------------------------------
double FrameDh::getFreeValue() {
  double value = 0.0;
  switch (m_freeVar) {
  case CR_DH_FREE_NONE:
    break;
  case CR_DH_FREE_R:
    value = m_dhR;
    break;
  case CR_DH_FREE_ALPHA:
    value = m_dhAlpha;
    break;
  case CR_DH_FREE_D:
    value = m_dhD;
    break;
  case CR_DH_FREE_THETA:
    value = m_dhTheta;
    break;
  }
  return value;
}

//------------------------------------------------------------------------------
/*!
 This method sets the the DH convention.\n

 \param[in] i_mode - DH convention
 */
//------------------------------------------------------------------------------
void FrameDh::setMode(DhMode i_mode) {
  m_dhMode = i_mode;
  this->setRotationAndTranslation();
}

//------------------------------------------------------------------------------
/*!
 This method gets the DH convention.\n

 \return - DH convention.
 */
//------------------------------------------------------------------------------
DhMode FrameDh::getMode(void) { return m_dhMode; }

//------------------------------------------------------------------------------
/*!
 This method sets the DH parameter values.\n

 \param[in] i_r - the r value in the DH parameter transformation
 \param[in] i_alpha - the alpha value in the DH parameter transformation
 \param[in] i_d - the d value in the DH parameter transformation
 \param[in] i_theta - the theta value in the DH parameter transformation
 */
//------------------------------------------------------------------------------
void FrameDh::setParameters(double i_r, double i_alpha, double i_d,
                            double i_theta) {
  m_dhR = i_r;
  m_dhAlpha = i_alpha;
  m_dhD = i_d;
  m_dhTheta = i_theta;
  this->setRotationAndTranslation();
}

//------------------------------------------------------------------------------
/*!
 This method gets the DH parameter values.\n

 \param[out] o_r - the r value in the DH parameter transformation
 \param[out] o_alpha - the alpha value in the DH parameter transformation
 \param[out] o_d - the d value in the DH parameter transformation
 \param[out] o_theta - the theta value in the DH parameter transformation
 */
//------------------------------------------------------------------------------
void FrameDh::getParameters(double &o_r, double &o_alpha, double &o_d,
                            double &o_theta) {
  o_r = m_dhR;
  o_alpha = m_dhAlpha;
  o_d = m_dhD;
  o_theta = m_dhTheta;
}

//------------------------------------------------------------------------------
/*!
 This method returns a true if the frame is driven (i.e. has a free
 variable) or a false if the frame is not driven.\n

 \return - true = is driven, false = is not driven

 */
//------------------------------------------------------------------------------
bool FrameDh::isDriven(void) {
  if (m_freeVar == CR_DH_FREE_NONE) {
    return false;
  } else {
    return true;
  }
}

//------------------------------------------------------------------------------
// Private Methods:

//! sets the private rotation and translation members - Note that
//  anytime a parameter gets set in the frame class, this method gets
//  called to update the rotation/translation members.
void FrameDh::setRotationAndTranslation() {
  switch (m_dhMode) {
  case CR_DH_MODE_CLASSIC:
    m_rotation << cos(m_dhTheta), -sin(m_dhTheta) * cos(m_dhAlpha),
        sin(m_dhTheta) * sin(m_dhAlpha), sin(m_dhTheta),
        cos(m_dhTheta) * cos(m_dhAlpha), -cos(m_dhTheta) * sin(m_dhAlpha), 0,
        sin(m_dhAlpha), cos(m_dhAlpha);
    m_translation << m_dhR * cos(m_dhTheta), m_dhR * sin(m_dhTheta), m_dhD;
    break;
  case CR_DH_MODE_MODIFIED:
    m_rotation << cos(m_dhTheta), -sin(m_dhTheta), 0,
        sin(m_dhTheta) * cos(m_dhAlpha), cos(m_dhTheta) * cos(m_dhAlpha),
        -sin(m_dhAlpha), sin(m_dhTheta) * sin(m_dhAlpha),
        cos(m_dhTheta) * sin(m_dhAlpha), cos(m_dhAlpha);
    m_translation << m_dhR, -m_dhD * sin(m_dhAlpha), m_dhD * cos(m_dhAlpha);
    break;
  }
}

} // namespace physics
} // namespace cr
