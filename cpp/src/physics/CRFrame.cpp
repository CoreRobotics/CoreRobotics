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

Source: http://www.staff.city.ac.uk/~sbbh653/publications/euler.pdf 

\param[in]  mode - the Euler mode enumerator
\param[out] pose - vector of Euler angles of the frame.
*/
//---------------------------------------------------------------------
void CRFrame::getEulerPose(CREulerMode mode, Eigen::Vector3d &pose)
{
    double a, b, g;

    if (this->rotation(2, 0) != 1 && this->rotation(2, 0) != -1)
    {
        double b1 = -asin(this->rotation(2, 0));
        double b2 = CoreRobotics::PI - b1;

        double g1 = atan2(this->rotation(2, 1) / cos(b1), this->rotation(2, 2) / cos(b1));
        double g2 = atan2(this->rotation(2, 1) / cos(b2), this->rotation(2, 2) / cos(b2));

        double a1 = atan2(this->rotation(1, 0) / cos(b1), this->rotation(0, 0) / cos(b1));
        double a2 = atan2(this->rotation(1, 0) / cos(b1), this->rotation(0, 0) / cos(b1));

        std::cout << "a1 = " << a1 << ", a2 = " << a2 << std::endl;
        std::cout << "b1 = " << b1 << ", b2 = " << b2 << std::endl;
        std::cout << "g1 = " << g1 << ", g2 = " << g2 << std::endl;


        // take the positive value if multiple solutions
        if (a1 >= 0) { a = a1; } else { a = a2; }
        if (b1 >= -CoreRobotics::PI / 2 && b1 <= CoreRobotics::PI / 2) 
        { 
            b = b1; 
        } 
        else 
        { 
            b = b2; 
        }
        if (g1 >= 0) { g = g1; } else { g = g2; }

    }
    else
    {
        a = 0;
        if (this->rotation(2, 0) == -1)
        {
            b = CoreRobotics::PI / 2.0;
            g = a + atan2(this->rotation(0, 1), this->rotation(0, 2));
        }
        else
        {
            b = -PI / 2.0;
            g = -a + atan2(-this->rotation(0, 1), -this->rotation(0, 2));
        }
    }

    pose << a, b, g;

    double a1 = CoreRobotics::CRMath::rad2deg(a);
    double b1 = CoreRobotics::CRMath::rad2deg(b);
    double g1 = CoreRobotics::CRMath::rad2deg(g);

}


void CRFrame::getEulerPose2(CREulerMode mode, Eigen::Vector3d &pose)
{

    float sy = sqrt(rotation(0, 0) * rotation(0, 0) + rotation(1, 0) * rotation(1, 0));

    bool singular = sy < 1e-6; // If

    double x, y, z;
    if (!singular)
    {
        x = atan2(rotation(2, 1), rotation(2, 2));
        y = atan2(rotation(2, 0), sy);
        z = atan2(rotation(1, 0), rotation(0, 0));
    }
    else
    {
        x = atan2(rotation(1, 2), rotation(1, 1));
        y = atan2(rotation(2, 0), sy);
        z = 0;
    }
    pose << x, y, z;
        

}

//=====================================================================
// End namespace
}


