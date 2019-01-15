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

\project CoreRobotics Project
\url     www.corerobotics.org
\author  Parker Owan

*/
//=====================================================================

#define _USE_MATH_DEFINES
#include <cmath>
#include "Eigen/Dense"
#include "CRFrame.hpp"
#include "math/CRMatrix.hpp"


//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics  {


//=====================================================================
/*!
 The constructor sets the rotation and translation parameters upon
 construction, with defaults listed in parenthesis.\n
 
 \param[in]     i_rot       the 3x3 rotation matrix (identity)
 \param[in]     i_trans     the 3x1 translation vector (zeros)
 */
//---------------------------------------------------------------------
CRFrame::CRFrame(Eigen::Matrix3d i_rot, Eigen::Vector3d i_trans)
{
    this->m_rotation = i_rot;
    this->m_translation = i_trans;
}
CRFrame::CRFrame()
{
    this->m_rotation << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    this->m_translation << 0, 0, 0;
}
    
    
    
//=====================================================================
/*!
 The destructor frees up memory.\n
 */
//---------------------------------------------------------------------
// CRFrame::~CRFrame(){ }
    
    
//=====================================================================
/*!
 This method sets the value of the free variable.  The method returns a
 false if there is no free variable.\n
 
 \param[in]     i_q     value of the variable to be set
 \return                CRResult flag indicating if there is a free 
                        value to write
 */
//---------------------------------------------------------------------
CRResult CRFrame::setFreeValue(double i_q)
{
    return CR_RESULT_UNWRITABLE;
}



//=====================================================================
/*!
 This method gets the value of the free variable.  The method returns
 0 if there is no free variable.\n
 
 \return        value of the free variable.
 */
//---------------------------------------------------------------------
double CRFrame::getFreeValue(void)
{
    return 0;
}


//=====================================================================
/*!
 This method returns a 4x4 homogeneous matrix of the frame 
 transformation to the parent frame (i-1). \n
 
 \return        the 4x4 homogeneous transformation matrix
 */
//---------------------------------------------------------------------
Eigen::Matrix4d CRFrame::getTransformToParent(void) {
    
    // get the rotation and translations
    Eigen::Matrix3d r = m_rotation;
    Eigen::Vector3d t = m_translation;
    
    // define the transform
    Eigen::Matrix4d transform;
    transform << r(0,0), r(0,1), r(0,2), t(0,0),
    r(1,0), r(1,1), r(1,2), t(1,0),
    r(2,0), r(2,1), r(2,2), t(2,0),
    0.0, 0.0, 0.0, 1.0;
    
    return transform;
}


//=====================================================================
/*!
 This method returns a 4x4 homogeneous matrix of the frame 
 transformation to the child frame (i). \n
 
 \return        the 4x4 homogeneous transformation matrix
 */
//---------------------------------------------------------------------
Eigen::Matrix4d CRFrame::getTransformToChild(void) {
    
    // get the rotation and translations
    Eigen::Matrix3d r = m_rotation.transpose();
    Eigen::Vector3d t = -r*m_translation;
    
    // define the transform
    Eigen::Matrix4d transform;
    transform << r(0,0), r(0,1), r(0,2), t(0,0),
    r(1,0), r(1,1), r(1,2), t(1,0),
    r(2,0), r(2,1), r(2,2), t(2,0),
    0.0, 0.0, 0.0, 1.0;
    
    return transform;
}


//=====================================================================
/*!
 This method transforms points \f$p\f$ in the child frame to points 
 \f$y\f$ in the parent frame.\n
 
 \param[in]     i_point         point in the child frame (p)
 \return                        the points transformed into the parent
                                frame (y)
 */
//---------------------------------------------------------------------
Eigen::Vector3d CRFrame::transformToParent(Eigen::Vector3d i_point) {
    return m_rotation*i_point + m_translation;
}


//=====================================================================
/*!
 This method transforms points \f$p\f$ in the parent frame to points 
 \f$y\f$ in the child frame.\n
 
 \param[in]     i_point         points in the parent frame (p)
 \return                        the points transformed into the child 
                                frame (y)
 */
//---------------------------------------------------------------------
Eigen::Vector3d CRFrame::transformToChild(Eigen::Vector3d i_point) {
    return m_rotation.transpose()*i_point - m_rotation.transpose()*m_translation;
}
    
    
//=====================================================================
/*!
 This method returns a true if the frame is driven (i.e. has a free
 variable) or a false if the frame is not driven.\n
 
 \return        boolean indicator for the frame having a free variable.
 
 */
//---------------------------------------------------------------------
bool CRFrame::isDriven(void) {
    return false;
}


//=====================================================================
/*!
This method gets a vector of Euler angles from the frame.

Source: https://www.geometrictools.com/Documentation/EulerAngles.pdf

\param[in]      i_mode      the Euler mode enumerator
\return                     vector of Euler angles of the frame.
*/
//---------------------------------------------------------------------
Eigen::Vector3d CRFrame::getOrientation(CREulerMode i_mode)
{
    double a, b, g;
    Eigen::Vector3d orientation;

    double r00 = this->m_rotation(0, 0);
    double r01 = this->m_rotation(0, 1);
    double r02 = this->m_rotation(0, 2);
    double r10 = this->m_rotation(1, 0);
    double r11 = this->m_rotation(1, 1);
    double r12 = this->m_rotation(1, 2);
    double r20 = this->m_rotation(2, 0);
    double r21 = this->m_rotation(2, 1);
    double r22 = this->m_rotation(2, 2);

    switch (i_mode)
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
                b = M_PI;
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
                b = M_PI;
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
                b = M_PI;
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
                b = M_PI;
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
                b = M_PI;
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
                b = M_PI;
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
                b = -M_PI / 2;
                a = -atan2(r10, r11);
                g = 0;
            }
        } 
        else 
        {
            b = M_PI / 2;
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
                b = M_PI / 2;
                a = -atan2(r21, r22);
                g = 0;
            }
        }
        else
        {
            b = M_PI / 2;
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
                b = M_PI / 2;
                a = -atan2(r02, r00);
                g = 0;
            }
        }
        else
        {
            b = M_PI / 2;
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
                b = M_PI / 2;
                a = -atan2(-r20, r22);
                g = 0;
            }
        }
        else
        {
            b = M_PI / 2;
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
                b = M_PI / 2;
                a = -atan2(-r12, r11);
                g = 0;
            }
        }
        else
        {
            b = -M_PI / 2;
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
                b = M_PI / 2;
                a = -atan2(-r01, r00);
                g = 0;
            }
        }
        else
        {
            b = M_PI / 2;
            a = atan2(-r01, r00);
            g = 0;
        }
        break;
    default:
        break;
    }

    // push to vector
    orientation << a, b, g;
    
    // return
    return orientation;
}


//=====================================================================
/*!
This method returns a vector of the position and orientation.

\param[in]      i_mode      the Euler mode enumerator
\return                     pose vector (x, y, z, a, b, g)^T
*/
//---------------------------------------------------------------------
Eigen::Matrix<double, 6, 1> CRFrame::getPose(CREulerMode i_mode)
{
    
    Eigen::Matrix<double, 6, 1> pose;
    
	Eigen::Vector3d theta = this->getOrientation(i_mode);

	pose << this->m_translation(0),
            this->m_translation(1),
            this->m_translation(2),
            theta(0),
            theta(1),
            theta(2);

    return pose;
}

//=====================================================================
/*!
 This method returns a vector of the position and orientation.
 
 \param[in]     i_mode          the Euler mode enumerator
 \param[in]     i_poseElements  a boolean vector indiciating which
                                elements of the pose vector to return
 \return                        reduced pose vector specified by the 
                                true elements in poseElements
 */
//---------------------------------------------------------------------
Eigen::VectorXd CRFrame::getPose(CREulerMode i_mode,
                                 Eigen::Matrix<bool, 6, 1> i_poseElements)
{
    
    // TODO: this whole thing could be faster
    // use overload above to return the full pose vector
    Eigen::Matrix<double, 6, 1> v = this->getPose(i_mode);
    
    // Get the number of true elements in the pose vector & size the output
    int m = i_poseElements.cast<int>().sum();
    
    // initialize a pose vector to return
    Eigen::VectorXd pose(m);
    
    // Now push into the pose vector
    int elem = 0;
    for (int row = 0; row < 6; row++){
        if (i_poseElements(row)){
            pose(elem) = v(row);
            elem++;
        };
    }
    
    return pose;
}


//=====================================================================
// End namespace
}
