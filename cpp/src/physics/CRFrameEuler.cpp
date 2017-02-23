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
 \version 0.0
 
 */
//=====================================================================

#include "CRFrameEuler.hpp"
#include "../../external/eigen/Eigen/Dense"


//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics {


//=====================================================================
/*!
 The constructor sets the rotation and translation parameters upon
 construction, with defaults listed in parenthesis.\n
 
 \param[in]   x      - x position of the frame (0)
 \param[in]   y      - y position of the frame (0)
 \param[in]   z      - z position of the frame (0)
 \param[in]   a      - alpha angle of the frame [rad] (0)
 \param[in]   b      - beta angle of the frame [rad] (0)
 \param[in]   g      - gamma angle of the frame [rad] (0)
 \param[in]   mode   - Euler angle convention (CR_EULER_MODE_ZXZ)
 \param[in]   free   - free variable (CR_EULER_FREE_NONE)
 */
//---------------------------------------------------------------------
CRFrameEuler::CRFrameEuler(double x, double y, double z, double a, double b, double g,
                           CREulerMode mode, CREulerFreeVariable free)
{
    pos_x = x;
    pos_y = y;
    pos_z = z;
    ang_a = a;
    ang_b = b;
    ang_g = g;
    eulerMode = mode;
    freeVar = free;
    setRotationAndTranslation();
}
CRFrameEuler::CRFrameEuler()
{
    pos_x = 0.0;
    pos_y = 0.0;
    pos_z = 0.0;
    ang_a = 0.0;
    ang_b = 0.0;
    ang_g = 0.0;
    eulerMode = CR_EULER_MODE_ZXZ;
    freeVar = CR_EULER_FREE_NONE;
    setRotationAndTranslation();
}



//=====================================================================
/*!
 This method sets the value of the free variable.  The method returns
 a true if the value was written and a false if freeVar is set to 
 CR_EULER_FREE_NONE.\n
 
 \param[in]   q   - value of the variable to be set
 */
//---------------------------------------------------------------------
bool CRFrameEuler::setFreeValue(double q)
{
    bool isWritable = true;
    switch (freeVar){
        case CR_EULER_FREE_NONE:
            isWritable = false;
            break;
        case CR_EULER_FREE_POS_X:
            pos_x = q;
            break;
        case CR_EULER_FREE_POS_Y:
            pos_y = q;
            break;
        case CR_EULER_FREE_POS_Z:
            pos_z = q;
            break;
        case CR_EULER_FREE_ANG_A:
            ang_a = q;
            break;
        case CR_EULER_FREE_ANG_B:
            ang_b = q;
            break;
        case CR_EULER_FREE_ANG_G:
            ang_g= q;
            break;
    }
    setRotationAndTranslation();
    return isWritable;
}



//=====================================================================
/*!
 This method get the value of the free variable.  The method returns 
 q = NULL if freeVar is set to CR_EULER_FREE_NONE.\n
 
 \param[out]   q   - value of the free variable.
 */
//---------------------------------------------------------------------
void CRFrameEuler::getFreeValue(double &q)
{
    switch (freeVar){
        case CR_EULER_FREE_NONE:
            q = 0.0;
            break;
        case CR_EULER_FREE_POS_X:
            q = pos_x;
            break;
        case CR_EULER_FREE_POS_Y:
            q = pos_y;
            break;
        case CR_EULER_FREE_POS_Z:
            q = pos_z;
            break;
        case CR_EULER_FREE_ANG_A:
            q = ang_a;
            break;
        case CR_EULER_FREE_ANG_B:
            q = ang_b;
            break;
        case CR_EULER_FREE_ANG_G:
            q = ang_g;
            break;
    }
}



//=====================================================================
/*!
 This method sets the value of the Euler convention.\n
 
 \param[in]   mode   - Euler convention
 */
//---------------------------------------------------------------------
void CRFrameEuler::setMode(CREulerMode mode)
{
    eulerMode = mode;
    setRotationAndTranslation();
}



//=====================================================================
/*!
 This method gets the value of the Euler convention.\n
 
 \param[out]   mode   - Euler convention.
 */
//---------------------------------------------------------------------
void CRFrameEuler::getMode(CREulerMode &mode)
{
    mode = eulerMode;
}



//=====================================================================
/*!
 This method sets the position values of the frame transformation.\n
 
 \param[in]   x   - x position of the frame
 \param[in]   y   - y position of the frame
 \param[in]   z   - z position of the frame
 */
//---------------------------------------------------------------------
void CRFrameEuler::setPosition(double x, double y, double z)
{
    pos_x = x;
    pos_y = y;
    pos_z = z;
    setRotationAndTranslation();
}


//=====================================================================
/*!
 This method gets the position values of the frame transformation.\n
 
 \param[out]   x   x position of the frame
 \param[out]   y   y position of the frame
 \param[out]   z   z position of the frame
 */
//---------------------------------------------------------------------
void CRFrameEuler::getPosition(double &x, double &y, double &z)
{
    x = pos_x;
    y = pos_y;
    z = pos_z;
}



//=====================================================================
/*!
 This method sets the orientation values of the frame transformation.\n
 
 \param[in]   a   - alpha angle of the frame [rad]
 \param[in]   b   - beta angle of the frame [rad]
 \param[in]   g   - gamma angle of the frame [rad]
 */
//---------------------------------------------------------------------
void CRFrameEuler::setOrientation(double a, double b, double g)
{
    ang_a = a;
    ang_b = b;
    ang_g = g;
    setRotationAndTranslation();
}


//=====================================================================
/*!
 This method gets the orientation values of the frame transformation.\n
 
 \param[out]   a   - alpha angle of the frame [rad]
 \param[out]   b   - beta angle of the frame [rad]
 \param[out]   g   - gamma angle of the frame [rad]
 */
//---------------------------------------------------------------------
void CRFrameEuler::getOrientation(double &a, double &b, double &g)
{
    a = ang_a;
    b = ang_b;
    g = ang_g;
}



//=====================================================================
/*!
 This method sets the position and orientation values of the frame 
 transformation.\n
 
 \param[in]   x   - x position of the frame
 \param[in]   y   - y position of the frame
 \param[in]   z   - z position of the frame
 \param[in]   a   - alpha angle of the frame [rad]
 \param[in]   b   - beta angle of the frame [rad]
 \param[in]   g   - gamma angle of the frame [rad]
 */
//---------------------------------------------------------------------
void CRFrameEuler::setPositionAndOrientation(double x, double y, double z, double a, double b, double g)
{
    setPosition(x,y,z);
    setOrientation(a,b,g);
}


//=====================================================================
/*!
 TThis method gets the position and orientation values of the frame 
 transformation.\n
 
 \param[out]   x   - x position of the frame
 \param[out]   y   - y position of the frame
 \param[out]   z   - z position of the frame
 \param[out]   a   - alpha angle of the frame [rad]
 \param[out]   b   - beta angle of the frame [rad]
 \param [out]  g   - gamma angle of the frame [rad]
 */
//---------------------------------------------------------------------
void CRFrameEuler::getPositionAndOrientation(double &x, double &y, double &z, double &a, double &b, double &g)
{
    getPosition(x,y,z);
    getOrientation(a,b,g);
}
    
    
    
//=====================================================================
/*!
 This method returns a true if the frame is driven (i.e. has a free 
 variable) or a false if the frame is not driven.\n
 
 \return - true = is driven, false = is not driven
 
 */
//---------------------------------------------------------------------
bool CRFrameEuler::isDriven() {
    if (freeVar == CR_EULER_FREE_NONE) {
        return false;
    } else {
        return true;
    }
}



//=====================================================================
// Private Methods:

//! sets the private rotation and translation members - Note that
//  anytime a parameter gets set in the frame class, this method gets
//  called to update the rotation/translation members.
void CRFrameEuler::setRotationAndTranslation()
{
    Eigen::Matrix3d r1;
    Eigen::Matrix3d r2;
    Eigen::Matrix3d r3;
    switch (eulerMode){
        case CR_EULER_MODE_ZXZ:
            rotAboutZ(ang_a, r1);
            rotAboutX(ang_b, r2);
            rotAboutZ(ang_g, r3);
            break;
        case CR_EULER_MODE_XYX:
            rotAboutX(ang_a, r1);
            rotAboutY(ang_b, r2);
            rotAboutX(ang_g, r3);
            break;
        case CR_EULER_MODE_YZY:
            rotAboutY(ang_a, r1);
            rotAboutZ(ang_b, r2);
            rotAboutY(ang_g, r3);
            break;
        case CR_EULER_MODE_ZYZ:
            rotAboutZ(ang_a, r1);
            rotAboutY(ang_b, r2);
            rotAboutZ(ang_g, r3);
            break;
        case CR_EULER_MODE_XZX:
            rotAboutX(ang_a, r1);
            rotAboutZ(ang_b, r2);
            rotAboutX(ang_g, r3);
            break;
        case CR_EULER_MODE_YXY:
            rotAboutY(ang_a, r1);
            rotAboutX(ang_b, r2);
            rotAboutY(ang_g, r3);
            break;
        case CR_EULER_MODE_XYZ:
            rotAboutX(ang_a, r1);
            rotAboutY(ang_b, r2);
            rotAboutZ(ang_g, r3);
            break;
        case CR_EULER_MODE_YZX:
            rotAboutY(ang_a, r1);
            rotAboutZ(ang_b, r2);
            rotAboutX(ang_g, r3);
            break;
        case CR_EULER_MODE_ZXY:
            rotAboutZ(ang_a, r1);
            rotAboutX(ang_b, r2);
            rotAboutY(ang_g, r3);
            break;
        case CR_EULER_MODE_XZY:
            rotAboutX(ang_a, r1);
            rotAboutZ(ang_b, r2);
            rotAboutY(ang_g, r3);
            break;
        case CR_EULER_MODE_ZYX:
            rotAboutZ(ang_a, r1);
            rotAboutY(ang_b, r2);
            rotAboutX(ang_g, r3);
            break;
        case CR_EULER_MODE_YXZ:
            rotAboutY(ang_a, r1);
            rotAboutX(ang_b, r2);
            rotAboutZ(ang_g, r3);
            break;
    }
    rotation = r1*r2*r3;
    translation << pos_x, pos_y, pos_z;
}

//! standard rotation about the x axis
void CRFrameEuler::rotAboutX(double ang, Eigen::Matrix3d &rot)
{
    rot << 1, 0, 0, 0, cos(ang), -sin(ang), 0, sin(ang),  cos(ang);
}

//! standard rotation about the y axis
void CRFrameEuler::rotAboutY(double ang, Eigen::Matrix3d &rot)
{
    rot << cos(ang), 0, sin(ang), 0, 1, 0, -sin(ang), 0, cos(ang);
}

//! standard rotation about the z axis
void CRFrameEuler::rotAboutZ(double ang, Eigen::Matrix3d &rot)
{
    rot << cos(ang), -sin(ang), 0, sin(ang), cos(ang), 0, 0, 0, 1;
}


//=====================================================================
// End namespace
}


