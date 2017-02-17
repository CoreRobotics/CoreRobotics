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

#include <iostream>
#include "CRFrame.hpp"
#include "../../external/eigen/Eigen/Dense"
#include "../math/CRMath.hpp"


//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics {


//=====================================================================
/*!
 The constructor sets the rotation and translation parameters upon
 construction, with defaults listed in parenthesis.\n
 
 \param[in]  rot   - the 3x3 rotation matrix (identity)
 \param[in]  trans - the 3x1 translation vector (zeros)
 */
//---------------------------------------------------------------------
CRFrame::CRFrame(Eigen::Matrix3d rot, Eigen::Vector3d trans)
{
    rotation = rot;
    translation = trans;
}
CRFrame::CRFrame()
{
    rotation << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    translation << 0, 0, 0;
}
    
    
//=====================================================================
/*!
 This method sets the value of the free variable.  The method returns a
 false if there is no free variable.\n
 
 \param[in] q - value of the variable to be set
 */
//---------------------------------------------------------------------
bool CRFrame::setFreeValue(double q)
{
    return false;
}



//=====================================================================
/*!
 This method gets the value of the free variable.  The method returns
 q = NULL if there is no free variable.\n
 
 \param[out] q - value of the free variable.
 */
//---------------------------------------------------------------------
void CRFrame::getFreeValue(double &q)
{
    q = NULL;
}


//=====================================================================
/*!
 This method returns a 4x4 homogeneous matrix of the frame 
 transformation to the parent frame (i-1). \n
 
 \param[out] transform - the 4x4 homogeneous transformation matrix
 */
//---------------------------------------------------------------------
void CRFrame::getTransformToParent(Eigen::Matrix4d &transform) {
    Eigen::Matrix3d r = rotation;
    Eigen::Vector3d t = translation;
    transform << r(0,0), r(0,1), r(0,2), t(0,0),
    r(1,0), r(1,1), r(1,2), t(1,0),
    r(2,0), r(2,1), r(2,2), t(2,0),
    0.0, 0.0, 0.0, 1.0;
}


//=====================================================================
/*!
 This method returns a 4x4 homogeneous matrix of the frame 
 transformation to the child frame (i). \n
 
 \param[out] transform - the 4x4 homogeneous transformation matrix
 */
//---------------------------------------------------------------------
void CRFrame::getTransformToChild(Eigen::Matrix4d &transform) {
    Eigen::Matrix3d r = rotation.transpose();
    Eigen::Vector3d t = -r*translation;
    transform << r(0,0), r(0,1), r(0,2), t(0,0),
    r(1,0), r(1,1), r(1,2), t(1,0),
    r(2,0), r(2,1), r(2,2), t(2,0),
    0.0, 0.0, 0.0, 1.0;
}


//=====================================================================
/*!
 This method transforms points \f$p\f$ in the child frame to points 
 \f$y\f$ in the parent frame.\n
 
 \param[in]  p - points in the child frame
 \param[out] y - the points transformed into the parent frame
 */
//---------------------------------------------------------------------
void CRFrame::transformToParent(Eigen::Vector3d p, Eigen::Vector3d &y) {
    y = rotation*p + translation;
}


//=====================================================================
/*!
 This method transforms points \f$p\f$ in the parent frame to points 
 \f$y\f$ in the child frame.\n
 
 \param[in]  p - points in the parent frame
 \param[out] y - the points transformed into the child frame
 */
//---------------------------------------------------------------------
void CRFrame::transformToChild(Eigen::Vector3d p, Eigen::Vector3d &y) {
    y = rotation.transpose()*p - rotation.transpose()*translation;
}
    
    
//=====================================================================
/*!
 This method returns a true if the frame is driven (i.e. has a free
 variable) or a false if the frame is not driven.\n
 
 \return result - T/F indicator for the frame having a free variable.
 
 */
//---------------------------------------------------------------------
bool CRFrame::isDriven() {
    return false;
}


//=====================================================================
/*!
This method gets a vector of Euler angles of the frame.

Source: https://www.geometrictools.com/Documentation/EulerAngles.pdf

\param[in]  mode - the Euler mode enumerator
\param[out] pose - vector of Euler angles of the frame.
*/
//---------------------------------------------------------------------
void CRFrame::getEulerPose(CREulerMode mode, Eigen::Vector3d &pose)
{
    double a, b, g;

    double r00 = this->rotation(0, 0);
    double r01 = this->rotation(0, 1);
    double r02 = this->rotation(0, 2);
    double r10 = this->rotation(1, 0);
    double r11 = this->rotation(1, 1);
    double r12 = this->rotation(1, 2);
    double r20 = this->rotation(2, 0);
    double r21 = this->rotation(2, 1);
    double r22 = this->rotation(2, 2);

    switch (mode)
    {
    case CR_EULER_MODE_ZXZ:
        if (r22 < 1)
        {
            if (r22 > -1)
            {
                b = acos(r22);
                a = atan2(r02, -r12);
                g = atan2(r20, r21);
            }
            else
            {
                b = CoreRobotics::PI;
                a = -atan2(-r01, r00);
                g = 0;
            }
        }
        else
        {
            b = 0;
            a = atan2(-r02, r00);
            g = 0;
        }
        break;
    case CR_EULER_MODE_XYX:
        if (r00 < 1)
        {
            if (r00 > -1)
            {
                b = acos(r00);
                a = atan2(r10, -r20);
                g = atan2(r01, r02);
            }
            else
            {
                b = CoreRobotics::PI;
                a = -atan2(-r12, r11);
                g = 0;
            }
        }
        else
        {
            b = 0;
            a = atan2(-r12, r11);
            g = 0;
        }
        break;
    case CR_EULER_MODE_YZY:
        if (r11 < 1)
        {
            if (r11 > -1)
            {
                b = acos(r11);
                a = atan2(r21, -r01);
                g = atan2(r12, r10);
            }
            else
            {
                b = CoreRobotics::PI;
                a = -atan2(-r20, r22);
                g = 0;
            }
        }
        else
        {
            b = 0;
            a = atan2(-r20, r22);
            g = 0;
        }
        break;
    case CR_EULER_MODE_ZYZ:
        if (r22 < 1)
        {
            if (r22 > -1)
            {
                b = acos(r22);
                a = atan2(r12, r02);
                g = atan2(r21, -r20);
            }
            else
            {
                b = CoreRobotics::PI;
                a = -atan2(r10, r11);
                g = 0;
            }
        }
        else
        {
            b = 0;
            a = atan2(r10, r11);
            g = 0;
        }
        break;
    case CR_EULER_MODE_XZX:
        if (r00 < 1)
        {
            if (r00 > -1)
            {
                b = acos(r00);
                a = atan2(r20, r10);
                g = atan2(r02, -r01);
            }
            else
            {
                b = CoreRobotics::PI;
                a = -atan2(r21, r22);
                g = 0;
            }
        }
        else
        {
            b = 0;
            a = atan2(r21, r22);
            g = 0;
        }
        break;
    case CR_EULER_MODE_YXY:
        if (r11 < 1)
        {
            if (r11 > -1)
            {
                b = acos(r11);
                a = atan2(r01, r21);
                g = atan2(r10, -r12);
            }
            else
            {
                b = CoreRobotics::PI;
                a = -atan2(r02, r00);
                g = 0;
            }
        }
        else
        {
            b = 0;
            a = atan2(r02, r00);
            g = 0;
        }
        break;
    case CR_EULER_MODE_XYZ:

        if (r02 < 1)
        {
            if (r02 > -1)
            {
                b = asin(r02);
                a = atan2(-r12, r22);
                g = atan2(-r01, r00);
            }
            else
            {
                b = -CoreRobotics::PI / 2;
                a = -atan2(r10, r11);
                g = 0;
            }
        } 
        else 
        {
            b = CoreRobotics::PI / 2;
            a = atan2(r10, r11);
            g = 0;
        }
        break;
    case CR_EULER_MODE_YZX:
        if (r10 < 1)
        {
            if (10 > -1)
            {
                b = asin(r10);
                a = atan2(-r20, r00);
                g = atan2(-r12, r11);
            }
            else
            {
                b = CoreRobotics::PI / 2;
                a = -atan2(r21, r22);
                g = 0;
            }
        }
        else
        {
            b = CoreRobotics::PI / 2;
            a = atan2(r21, r22);
            g = 0;
        }
        break;
    case CR_EULER_MODE_ZXY:
        if (r21 < 1)
        {
            if (r21 > -1)
            {
                b = asin(r21);
                a = atan2(-r01, r11);
                g = atan2(-r20, r22);
            }
            else
            {
                b = CoreRobotics::PI / 2;
                a = -atan2(r02, r00);
                g = 0;
            }
        }
        else
        {
            b = CoreRobotics::PI / 2;
            a = atan2(r02, r00);
            g = 0;
        }
        break;
    case CR_EULER_MODE_XZY:
        if (r01 < 1)
        {
            if (r01 > -1)
            {
                b = asin(-r01);
                a = atan2(r21, r11);
                g = atan2(r02, r00);
            }
            else
            {
                b = CoreRobotics::PI / 2;
                a = -atan2(-r20, r22);
                g = 0;
            }
        }
        else
        {
            b = CoreRobotics::PI / 2;
            a = atan2(-r20, r22);
            g = 0;
        }
        break;
    case CR_EULER_MODE_ZYX:
        if (r20 < 1)
        {
            if (r20 > -1)
            {
                b = asin(-r20);
                a = atan2(r10, r00);
                g = atan2(r21, r22);
            }
            else
            {
                b = CoreRobotics::PI / 2;
                a = -atan2(-r12, r11);
                g = 0;
            }
        }
        else
        {
            b = -CoreRobotics::PI / 2;
            a = atan2(-r12, r11);
            g = 0;
        }
        break;
    case CR_EULER_MODE_YXZ:
        if (r12 < 1)
        {
            if (r12 > -1)
            {
                b = asin(-r12);
                a = atan2(r02, r22);
                g = atan2(r10, r11);
            }
            else
            {
                b = CoreRobotics::PI / 2;
                a = -atan2(-r01, r00);
                g = 0;
            }
        }
        else
        {
            b = CoreRobotics::PI / 2;
            a = atan2(-r01, r00);
            g = 0;
        }
        break;
    default:
        break;
    }

    pose << a, b, g;
}


//=====================================================================
// End namespace
}


=======
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

#include "CRFrame.hpp"
#include "../../external/eigen/Eigen/Dense"


//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics {


//=====================================================================
/*!
 The constructor sets the rotation and translation parameters upon
 construction, with defaults listed in parenthesis.\n
 
 \param[in]  rot   - the 3x3 rotation matrix (identity)
 \param[in]  trans - the 3x1 translation vector (zeros)
 */
//---------------------------------------------------------------------
CRFrame::CRFrame(Eigen::Matrix3d rot, Eigen::Vector3d trans)
{
    rotation = rot;
    translation = trans;
}
CRFrame::CRFrame()
{
    rotation << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    translation << 0, 0, 0;
}

    
//=====================================================================
/*!
 This method sets the value of the free variable.  The method returns a
 false if there is no free variable.\n
 
 \param[in] q - value of the variable to be set
 */
//---------------------------------------------------------------------
bool CRFrame::setFreeValue(double q)
{
    return false;
}



//=====================================================================
/*!
 This method gets the value of the free variable.  The method returns
 q = NULL if there is no free variable.\n
 
 \param[out] q - value of the free variable.
 */
//---------------------------------------------------------------------
void CRFrame::getFreeValue(double &q)
{
    q = 0.0;
}


//=====================================================================
/*!
 This method returns a 4x4 homogeneous matrix of the frame 
 transformation to the parent frame (i-1). \n
 
 \param[out] transform - the 4x4 homogeneous transformation matrix
 */
//---------------------------------------------------------------------
void CRFrame::getTransformToParent(Eigen::Matrix4d &transform) {
    Eigen::Matrix3d r = rotation;
    Eigen::Vector3d t = translation;
    transform << r(0,0), r(0,1), r(0,2), t(0,0),
    r(1,0), r(1,1), r(1,2), t(1,0),
    r(2,0), r(2,1), r(2,2), t(2,0),
    0.0, 0.0, 0.0, 1.0;
}


//=====================================================================
/*!
 This method returns a 4x4 homogeneous matrix of the frame 
 transformation to the child frame (i). \n
 
 \param[out] transform - the 4x4 homogeneous transformation matrix
 */
//---------------------------------------------------------------------
void CRFrame::getTransformToChild(Eigen::Matrix4d &transform) {
    Eigen::Matrix3d r = rotation.transpose();
    Eigen::Vector3d t = -r*translation;
    transform << r(0,0), r(0,1), r(0,2), t(0,0),
    r(1,0), r(1,1), r(1,2), t(1,0),
    r(2,0), r(2,1), r(2,2), t(2,0),
    0.0, 0.0, 0.0, 1.0;
}


//=====================================================================
/*!
 This method transforms points \f$p\f$ in the child frame to points 
 \f$y\f$ in the parent frame.\n
 
 \param[in]  p - points in the child frame
 \param[out] y - the points transformed into the parent frame
 */
//---------------------------------------------------------------------
void CRFrame::transformToParent(Eigen::Vector3d p, Eigen::Vector3d &y) {
    y = rotation*p + translation;
}


//=====================================================================
/*!
 This method transforms points \f$p\f$ in the parent frame to points 
 \f$y\f$ in the child frame.\n
 
 \param[in]  p - points in the parent frame
 \param[out] y - the points transformed into the child frame
 */
//---------------------------------------------------------------------
void CRFrame::transformToChild(Eigen::Vector3d p, Eigen::Vector3d &y) {
    y = rotation.transpose()*p - rotation.transpose()*translation;
}
    
    
//=====================================================================
/*!
 This method returns a true if the frame is driven (i.e. has a free
 variable) or a false if the frame is not driven.\n
 
 \return result - T/F indicator for the frame having a free variable.
 
 */
//---------------------------------------------------------------------
bool CRFrame::isDriven() {
    return false;
}



//=====================================================================
// End namespace
}