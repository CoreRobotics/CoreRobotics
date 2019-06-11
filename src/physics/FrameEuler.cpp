/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#include "FrameEuler.hpp"
#include "Eigen/Dense"
#include "math/Matrix.hpp"

namespace cr {
namespace physics {

//------------------------------------------------------------------------------
/*!
 The constructor sets the rotation and translation parameters upon
 construction, with defaults listed in parenthesis.\n

 \param[in]   i_x      - x position of the frame (0)
 \param[in]   i_y      - y position of the frame (0)
 \param[in]   i_z      - z position of the frame (0)
 \param[in]   i_a      - alpha angle of the frame [rad] (0)
 \param[in]   i_b      - beta angle of the frame [rad] (0)
 \param[in]   i_g      - gamma angle of the frame [rad] (0)
 \param[in]   i_mode   - Euler angle convention (CR_EULER_MODE_ZXZ)
 \param[in]   i_free   - free variable (CR_EULER_FREE_NONE)
 */
//------------------------------------------------------------------------------
FrameEuler::FrameEuler(double i_x, double i_y, double i_z, double i_a,
                       double i_b, double i_g, EulerMode i_mode,
                       EulerFreeVariable i_free) {
  m_posX = i_x;
  m_posY = i_y;
  m_posZ = i_z;
  m_angA = i_a;
  m_angB = i_b;
  m_angG = i_g;
  m_eulerMode = i_mode;
  m_freeVar = i_free;
  this->setRotationAndTranslation();
}
FrameEuler::FrameEuler() {
  m_posX = 0.0;
  m_posY = 0.0;
  m_posZ = 0.0;
  m_angA = 0.0;
  m_angB = 0.0;
  m_angG = 0.0;
  m_eulerMode = CR_EULER_MODE_ZXZ;
  m_freeVar = CR_EULER_FREE_NONE;
  this->setRotationAndTranslation();
}

//------------------------------------------------------------------------------
/*!
 This method sets the value of the free variable.  The method returns
 a true if the value was written and a false if m_freeVar is set to
 CR_EULER_FREE_NONE.\n

 \param[in]   i_q   - value of the variable to be set
 \return - Result flag indicating if the parameter is writable
 */
//------------------------------------------------------------------------------
core::Result FrameEuler::setFreeValue(double i_q) {
  core::Result result = core::CR_RESULT_SUCCESS;
  switch (m_freeVar) {
  case CR_EULER_FREE_NONE:
    result = core::CR_RESULT_UNWRITABLE;
    break;
  case CR_EULER_FREE_POS_X:
    m_posX = i_q;
    break;
  case CR_EULER_FREE_POS_Y:
    m_posY = i_q;
    break;
  case CR_EULER_FREE_POS_Z:
    m_posZ = i_q;
    break;
  case CR_EULER_FREE_ANG_A:
    m_angA = i_q;
    break;
  case CR_EULER_FREE_ANG_B:
    m_angB = i_q;
    break;
  case CR_EULER_FREE_ANG_G:
    m_angG = i_q;
    break;
  }
  this->setRotationAndTranslation();
  return result;
}

//------------------------------------------------------------------------------
/*!
 This method get the value of the free variable.  The method returns
 q = NULL if m_freeVar is set to CR_EULER_FREE_NONE.\n

 \return - value of the free variable.
 */
//------------------------------------------------------------------------------
double FrameEuler::getFreeValue(void) {
  double value = 0.0;
  switch (m_freeVar) {
  case CR_EULER_FREE_NONE:
    break;
  case CR_EULER_FREE_POS_X:
    value = m_posX;
    break;
  case CR_EULER_FREE_POS_Y:
    value = m_posY;
    break;
  case CR_EULER_FREE_POS_Z:
    value = m_posZ;
    break;
  case CR_EULER_FREE_ANG_A:
    value = m_angA;
    break;
  case CR_EULER_FREE_ANG_B:
    value = m_angB;
    break;
  case CR_EULER_FREE_ANG_G:
    value = m_angG;
    break;
  }
  return value;
}

//------------------------------------------------------------------------------
/*!
 This method sets the value of the Euler convention.\n

 \param[in]   i_mode   - Euler convention
 */
//------------------------------------------------------------------------------
void FrameEuler::setMode(EulerMode i_mode) {
  m_eulerMode = i_mode;
  this->setRotationAndTranslation();
}

//------------------------------------------------------------------------------
/*!
 This method gets the value of the Euler convention.\n

 \return - Euler convention.
 */
//------------------------------------------------------------------------------
EulerMode FrameEuler::getMode(void) { return m_eulerMode; }

//------------------------------------------------------------------------------
/*!
 This method sets the position values of the frame transformation.\n

 \param[in]   i_x   - x position of the frame
 \param[in]   i_y   - y position of the frame
 \param[in]   i_z   - z position of the frame
 */
//------------------------------------------------------------------------------
void FrameEuler::setPosition(double i_x, double i_y, double i_z) {
  m_posX = i_x;
  m_posY = i_y;
  m_posZ = i_z;
  this->setRotationAndTranslation();
}

//------------------------------------------------------------------------------
/*!
 This method gets the position values of the frame transformation.\n

 \param[out]   o_x   x position of the frame
 \param[out]   o_y   y position of the frame
 \param[out]   o_z   z position of the frame
 */
//------------------------------------------------------------------------------
void FrameEuler::getPosition(double &o_x, double &o_y, double &o_z) {
  o_x = m_posX;
  o_y = m_posY;
  o_z = m_posZ;
}

//------------------------------------------------------------------------------
/*!
 This method sets the orientation values of the frame transformation.\n

 \param[in]   i_a   - alpha angle of the frame [rad]
 \param[in]   i_b   - beta angle of the frame [rad]
 \param[in]   i_g   - gamma angle of the frame [rad]
 */
//------------------------------------------------------------------------------
void FrameEuler::setOrientation(double i_a, double i_b, double i_g) {
  m_angA = i_a;
  m_angB = i_b;
  m_angG = i_g;
  this->setRotationAndTranslation();
}

//------------------------------------------------------------------------------
/*!
 This method gets the orientation values of the frame transformation.\n

 \param[out]   o_a   - alpha angle of the frame [rad]
 \param[out]   o_b   - beta angle of the frame [rad]
 \param[out]   o_g   - gamma angle of the frame [rad]
 */
//------------------------------------------------------------------------------
void FrameEuler::getOrientation(double &o_a, double &o_b, double &o_g) {
  o_a = m_angA;
  o_b = m_angB;
  o_g = m_angG;
}

//------------------------------------------------------------------------------
/*!
 This method sets the position and orientation values of the frame
 transformation.\n

 \param[in]   i_x   - x position of the frame
 \param[in]   i_y   - y position of the frame
 \param[in]   i_z   - z position of the frame
 \param[in]   i_a   - alpha angle of the frame [rad]
 \param[in]   i_b   - beta angle of the frame [rad]
 \param[in]   i_g   - gamma angle of the frame [rad]
 */
//------------------------------------------------------------------------------
void FrameEuler::setPositionAndOrientation(double i_x, double i_y, double i_z,
                                           double i_a, double i_b, double i_g) {
  setPosition(i_x, i_y, i_z);
  setOrientation(i_a, i_b, i_g);
}

//------------------------------------------------------------------------------
/*!
 TThis method gets the position and orientation values of the frame
 transformation.\n

 \param[out]   o_x   - x position of the frame
 \param[out]   o_y   - y position of the frame
 \param[out]   o_z   - z position of the frame
 \param[out]   o_a   - alpha angle of the frame [rad]
 \param[out]   o_b   - beta angle of the frame [rad]
 \param [out]  o_g   - gamma angle of the frame [rad]
 */
//------------------------------------------------------------------------------
void FrameEuler::getPositionAndOrientation(double &o_x, double &o_y,
                                           double &o_z, double &o_a,
                                           double &o_b, double &o_g) {
  getPosition(o_x, o_y, o_z);
  getOrientation(o_a, o_b, o_g);
}

//------------------------------------------------------------------------------
/*!
 NOT RECCOMENDED!
 This method sets the rotation and translation matrices for the frame
 transformation.\n

 \param[int]   i_rot     - rotation matrix (3 x 3)
 \param[out]   i_trans   - translation vector (3 x 1)
 */
//------------------------------------------------------------------------------
void FrameEuler::setRotationAndTranslation(Eigen::Matrix3d i_rot,
                                           Eigen::Vector3d i_trans) {
  this->m_rotation = i_rot;
  this->m_translation = i_trans;
  Eigen::Matrix<double, 6, 1> p = this->getPose(this->getMode());
  this->setPositionAndOrientation(p(0), p(1), p(2), p(3), p(4), p(5));
}

//------------------------------------------------------------------------------
/*!
 This method returns a true if the frame is driven (i.e. has a free
 variable) or a false if the frame is not driven.\n

 \return - true = is driven, false = is not driven

 */
//------------------------------------------------------------------------------
bool FrameEuler::isDriven(void) {
  if (m_freeVar == CR_EULER_FREE_NONE) {
    return false;
  } else {
    return true;
  }
}

//------------------------------------------------------------------------------
// Private Methods:

//! sets the private rotation and translation members - Note that
//  anytime a parameter gets set in the frame class, this method gets
//  called to compute the rotation/translation members.
void FrameEuler::setRotationAndTranslation() {

  Eigen::Matrix3d r1 = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d r2 = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d r3 = Eigen::Matrix3d::Identity();

  switch (m_eulerMode) {
  case CR_EULER_MODE_ZXZ:
    r1 = math::Matrix::rotAboutZ(m_angA);
    r2 = math::Matrix::rotAboutX(m_angB);
    r3 = math::Matrix::rotAboutZ(m_angG);
    break;
  case CR_EULER_MODE_XYX:
    r1 = math::Matrix::rotAboutX(m_angA);
    r2 = math::Matrix::rotAboutY(m_angB);
    r3 = math::Matrix::rotAboutX(m_angG);
    break;
  case CR_EULER_MODE_YZY:
    r1 = math::Matrix::rotAboutY(m_angA);
    r2 = math::Matrix::rotAboutZ(m_angB);
    r3 = math::Matrix::rotAboutY(m_angG);
    break;
  case CR_EULER_MODE_ZYZ:
    r1 = math::Matrix::rotAboutZ(m_angA);
    r2 = math::Matrix::rotAboutY(m_angB);
    r3 = math::Matrix::rotAboutZ(m_angG);
    break;
  case CR_EULER_MODE_XZX:
    r1 = math::Matrix::rotAboutX(m_angA);
    r2 = math::Matrix::rotAboutZ(m_angB);
    r3 = math::Matrix::rotAboutX(m_angG);
    break;
  case CR_EULER_MODE_YXY:
    r1 = math::Matrix::rotAboutY(m_angA);
    r2 = math::Matrix::rotAboutX(m_angB);
    r3 = math::Matrix::rotAboutY(m_angG);
    break;
  case CR_EULER_MODE_XYZ:
    r1 = math::Matrix::rotAboutX(m_angA);
    r2 = math::Matrix::rotAboutY(m_angB);
    r3 = math::Matrix::rotAboutZ(m_angG);
    break;
  case CR_EULER_MODE_YZX:
    r1 = math::Matrix::rotAboutY(m_angA);
    r2 = math::Matrix::rotAboutZ(m_angB);
    r3 = math::Matrix::rotAboutX(m_angG);
    break;
  case CR_EULER_MODE_ZXY:
    r1 = math::Matrix::rotAboutZ(m_angA);
    r2 = math::Matrix::rotAboutX(m_angB);
    r3 = math::Matrix::rotAboutY(m_angG);
    break;
  case CR_EULER_MODE_XZY:
    r1 = math::Matrix::rotAboutX(m_angA);
    r2 = math::Matrix::rotAboutZ(m_angB);
    r3 = math::Matrix::rotAboutY(m_angG);
    break;
  case CR_EULER_MODE_ZYX:
    r1 = math::Matrix::rotAboutZ(m_angA);
    r2 = math::Matrix::rotAboutY(m_angB);
    r3 = math::Matrix::rotAboutX(m_angG);
    break;
  case CR_EULER_MODE_YXZ:
    r1 = math::Matrix::rotAboutY(m_angA);
    r2 = math::Matrix::rotAboutX(m_angB);
    r3 = math::Matrix::rotAboutZ(m_angG);
    break;
  }
  m_rotation = r1 * r2 * r3;
  m_translation << m_posX, m_posY, m_posZ;
}

} // namespace physics
} // namespace cr
