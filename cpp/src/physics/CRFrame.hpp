//=====================================================================
/*
    Software License Agreement (BSD-3-Clause License)
    Copyright (c) 2017, CoreRobotics.
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

    \author CoreRobotics Project
    \author www.corerobotics.org
    \author Parker Owan
    \version 0.0

*/
//=====================================================================

#ifndef CRFrame_hpp
#define CRFrame_hpp

//=====================================================================
// Includes
#include "../../external/eigen/Eigen/Dense"


//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics {
    
//=====================================================================
/*!
\file CRFrame.hpp
\brief Implements a basic frame class for handling 3D Euclidean affine
transformations.
*/
//---------------------------------------------------------------------
/*!
\class CRFrame
\ingroup physics
 
\brief This class implements a basic homogeneous transformation in the
special Euclidean group made up of rotations and translations.
 
\details
CRFrame implements a homogeneous transformation
*/
//=====================================================================
//! Enumerator for handling Euler angle conventions
enum CREulerMode {
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
    
//=====================================================================
class CRFrame {

//---------------------------------------------------------------------
// Constructor and Destructor
public:

    //! Class constructor
    CRFrame();
    CRFrame(Eigen::Matrix3d rot, Eigen::Vector3d trans);

    //! Class destructor
    virtual ~CRFrame();
    
    
//---------------------------------------------------------------------
// Get/Set Methods
public:

    //! Set the rotation and translation
    void setRotationAndTranslation(Eigen::Matrix3d rot, Eigen::Vector3d trans) {rotation = rot; translation = trans;}

    //! Return the rotation and translation
    void getRotationAndTranslation(Eigen::Matrix3d &rot, Eigen::Vector3d &trans) {rot = rotation; trans = translation;}
    
    
//---------------------------------------------------------------------
// Public Methods
public:
    
    //! Get the transformation to the parent frame
    void getTransformToParent(Eigen::Matrix4d &transform);
    
    //! Get the transformation to the child frame
    void getTransformToChild(Eigen::Matrix4d &transform);
    
    //! Transform points p in the child frame to points y in the parent frame
    void transformToParent(Eigen::Matrix<double, 3, Eigen::Dynamic> p, Eigen::Matrix<double, 3, Eigen::Dynamic> &y);
    
    //! Transform points p in the parent frame to points y in the child frame
    void transformToChild(Eigen::Matrix<double, 3, Eigen::Dynamic> p, Eigen::Matrix<double, 3, Eigen::Dynamic> &y);
    

    //! Get the pose vectors where the Euler orientation convention is specified by the crEulerMode enumerator
    // void getPoseVectors(CREulerMode mode, Eigen::Vector3d &position, Eigen::Vector3d &orientation);
    
//---------------------------------------------------------------------
// Protected Members
protected:
    
    //! Rotation matrix data
    Eigen::Matrix3d rotation;

    //! Translation vector data
    Eigen::Vector3d translation;


};
    
//=====================================================================
// End namespace
}

#endif
