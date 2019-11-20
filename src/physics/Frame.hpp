/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#ifndef CR_PHYSICS_FRAME_HPP_
#define CR_PHYSICS_FRAME_HPP_

#include "Eigen/Dense"
#include "core/Types.hpp"

namespace cr {
namespace physics {

//! Enumerator for handling Euler angle conventions
enum EulerMode {
  CR_EULER_MODE_ZXZ,
  CR_EULER_MODE_XYX,
  CR_EULER_MODE_YZY,
  CR_EULER_MODE_ZYZ,
  CR_EULER_MODE_XZX,
  CR_EULER_MODE_YXY,
  CR_EULER_MODE_XYZ,
  CR_EULER_MODE_YZX,
  CR_EULER_MODE_ZXY,
  CR_EULER_MODE_XZY,
  CR_EULER_MODE_ZYX,
  CR_EULER_MODE_YXZ,
};

//------------------------------------------------------------------------------
/*!
 \class Frame
 \ingroup physics

 \brief This class implements a basic homogeneous transformation made
 up of rotations and translations.

 \details
 ## Description
 Frame implements a homogeneous transformation specified by a
 rotation matrix \f$r\f$ and a translation vector \f$t\f$.  The
 transformation matrix
 \f[
 ^{i-1}_{i}T = \left(\begin{array}{cc}
 r & t \\ 0^\top & 1
 \end{array}\right)
 \f]
 transforms a point \f$^i p\f$ in the child frame (i) to a point
 \f$^{i-1} p\f$ in the parent frame (i-1) by the operation
 \f[
 {^{i-1} p} = ^{i-1}_{i}T {^i p}.
 \f]

 These methods return the transformation \f$T\f$ and inverse
 \f$T^{-1}\f$:
 - Frame::getTransformToParent outputs \f$^{i-1}_{i}T\f$.
 - Frame::getTransformToChild outputs \f$_{i-1}^{i}T\f$.

 These methods perform the transformation operation on a vector \f$p\f$
 - Frame::transformToParent performs \f$y = ^{i-1}_{i}T p\f$
 - Frame::transformToChild performs \f$y = _{i-1}^{i}T p\f$

 ## Example
 \include example_Manipulator.cpp

 ## References
 [1] J. Craig, "Introduction to Robotics: Mechanics and Control", Ed. 3,
 Pearson, 2004.

 */
//------------------------------------------------------------------------------
class Frame {

  // Constructor and Destructor
public:
  //! Class constructor
  Frame();
  Frame(const Eigen::Matrix3d &i_rot, const Eigen::Vector3d &i_trans);

  //! Class destructor
  virtual ~Frame() = default;

  // Get/Set Methods
public:
  //! Set the value of the free variable
  virtual core::Result setFreeValue(double i_q);

  //! Get the value of the free variable
  virtual double getFreeValue(void);

  //! Set the rotation
  void setRotation(Eigen::Matrix3d i_rot) { m_rotation = i_rot; }

  //! Set the translation
  void setTranslation(Eigen::Vector3d i_trans) { m_translation = i_trans; }

  //! Set the rotation and translation
  void setRotationAndTranslation(Eigen::Matrix3d i_rot,
                                 Eigen::Vector3d i_trans) {
    this->m_rotation = i_rot;
    this->m_translation = i_trans;
  }

  //! Return the rotation and translation
  void getRotationAndTranslation(Eigen::Matrix3d &o_rot,
                                 Eigen::Vector3d &o_trans) {
    o_rot = this->m_rotation;
    o_trans = this->m_translation;
  }

  // Public Methods
public:
  //! Get the transformation to the parent frame
  Eigen::Matrix4d getTransformToParent(void);

  //! Get the transformation to the child frame
  Eigen::Matrix4d getTransformToChild(void);

  //! Transform a point p in the child frame to a point y in the parent frame
  Eigen::Vector3d transformToParent(Eigen::Vector3d i_point);

  //! Transform a point p in the parent frame to a point y in the child frame
  Eigen::Vector3d transformToChild(Eigen::Vector3d i_point);

  //! Query if the frame is driven, i.e. has a free variable
  virtual bool isDriven(void);

  //! Get the position vector
  Eigen::Vector3d getTranslation() { return m_translation; }

  //! Get the rotation matrix
  Eigen::Matrix3d getRotation() { return m_rotation; }

  //! Get a vector of the Euler angles
  Eigen::Vector3d getOrientation(EulerMode i_mode);

  //! Get the pose vector where the Euler orientation convention is specified by
  //! mode
  Eigen::Matrix<double, 6, 1> getPose(EulerMode i_mode);
  Eigen::VectorXd getPose(EulerMode i_mode,
                          Eigen::Matrix<bool, 6, 1> i_poseElements);

  // Protected Members
protected:
  //! Rotation matrix data
  Eigen::Matrix3d m_rotation;

  //! Translation vector data
  Eigen::Vector3d m_translation;

  // Fix the static size matrices to be 128-bit aligned
  // https://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html
  // public:
  //	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace physics
} // namespace cr

#endif
