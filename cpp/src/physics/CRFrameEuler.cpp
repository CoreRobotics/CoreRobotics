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
#include "CRMath.hpp"
#include "Eigen/Dense"


//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics {


//=====================================================================
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
//---------------------------------------------------------------------
CRFrameEuler::CRFrameEuler(double i_x,
                           double i_y,
                           double i_z,
                           double i_a,
                           double i_b,
                           double i_g,
                           CREulerMode i_mode,
                           CREulerFreeVariable i_free)
{
    m_posX = i_x;
    m_posY = i_y;
    m_posZ = i_z;
    m_angA = i_a;
    m_angB = i_b;
    m_angG = i_g;
    m_eulerMode = i_mode;
    m_freeVar = i_free;
    setRotationAndTranslation();
}
CRFrameEuler::CRFrameEuler()
{
    m_posX = 0.0;
    m_posY = 0.0;
    m_posZ = 0.0;
    m_angA = 0.0;
    m_angB = 0.0;
    m_angG = 0.0;
    m_eulerMode = CR_EULER_MODE_ZXZ;
    m_freeVar = CR_EULER_FREE_NONE;
    setRotationAndTranslation();
}



//=====================================================================
/*!
 This method sets the value of the free variable.  The method returns
 a true if the value was written and a false if m_freeVar is set to 
 CR_EULER_FREE_NONE.\n
 
 \param[in]   i_q   - value of the variable to be set
 \return - CRResult flag indicating if the parameter is writable
 */
//---------------------------------------------------------------------
CRResult CRFrameEuler::setFreeValue(double i_q)
{
    CRResult result = CR_RESULT_SUCCESS;
    switch (m_freeVar){
        case CR_EULER_FREE_NONE:
            result = CR_RESULT_UNWRITABLE;
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
    setRotationAndTranslation();
    return result;
}



//=====================================================================
/*!
 This method get the value of the free variable.  The method returns 
 q = NULL if m_freeVar is set to CR_EULER_FREE_NONE.\n
 
 \return - value of the free variable.
 */
//---------------------------------------------------------------------
double CRFrameEuler::getFreeValue(void)
{
    switch (m_freeVar){
        case CR_EULER_FREE_NONE:
            return 0.0;
            break;
        case CR_EULER_FREE_POS_X:
            return m_posX;
            break;
        case CR_EULER_FREE_POS_Y:
            return m_posY;
            break;
        case CR_EULER_FREE_POS_Z:
            return m_posZ;
            break;
        case CR_EULER_FREE_ANG_A:
            return m_angA;
            break;
        case CR_EULER_FREE_ANG_B:
            return m_angB;
            break;
        case CR_EULER_FREE_ANG_G:
            return m_angG;
            break;
    }
}



//=====================================================================
/*!
 This method sets the value of the Euler convention.\n
 
 \param[in]   i_mode   - Euler convention
 */
//---------------------------------------------------------------------
void CRFrameEuler::setMode(CREulerMode i_mode)
{
    m_eulerMode = i_mode;
    setRotationAndTranslation();
}



//=====================================================================
/*!
 This method gets the value of the Euler convention.\n
 
 \return - Euler convention.
 */
//---------------------------------------------------------------------
CREulerMode CRFrameEuler::getMode(void)
{
    return m_eulerMode;
}



//=====================================================================
/*!
 This method sets the position values of the frame transformation.\n
 
 \param[in]   i_x   - x position of the frame
 \param[in]   i_y   - y position of the frame
 \param[in]   i_z   - z position of the frame
 */
//---------------------------------------------------------------------
void CRFrameEuler::setPosition(double i_x, double i_y, double i_z)
{
    m_posX = i_x;
    m_posY = i_y;
    m_posZ = i_z;
    setRotationAndTranslation();
}


//=====================================================================
/*!
 This method gets the position values of the frame transformation.\n
 
 \param[out]   o_x   x position of the frame
 \param[out]   o_y   y position of the frame
 \param[out]   o_z   z position of the frame
 */
//---------------------------------------------------------------------
void CRFrameEuler::getPosition(double& o_x, double& o_y, double& o_z)
{
    o_x = m_posX;
    o_y = m_posY;
    o_z = m_posZ;
}



//=====================================================================
/*!
 This method sets the orientation values of the frame transformation.\n
 
 \param[in]   i_a   - alpha angle of the frame [rad]
 \param[in]   i_b   - beta angle of the frame [rad]
 \param[in]   i_g   - gamma angle of the frame [rad]
 */
//---------------------------------------------------------------------
void CRFrameEuler::setOrientation(double i_a, double i_b, double i_g)
{
    m_angA = i_a;
    m_angB = i_b;
    m_angG = i_g;
    setRotationAndTranslation();
}


//=====================================================================
/*!
 This method gets the orientation values of the frame transformation.\n
 
 \param[out]   o_a   - alpha angle of the frame [rad]
 \param[out]   o_b   - beta angle of the frame [rad]
 \param[out]   o_g   - gamma angle of the frame [rad]
 */
//---------------------------------------------------------------------
void CRFrameEuler::getOrientation(double& o_a, double& o_b, double& o_g)
{
    o_a = m_angA;
    o_b = m_angB;
    o_g = m_angG;
}



//=====================================================================
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
//---------------------------------------------------------------------
void CRFrameEuler::setPositionAndOrientation(double i_x,
                                             double i_y,
                                             double i_z,
                                             double i_a,
                                             double i_b,
                                             double i_g){
    setPosition(i_x,i_y,i_z);
    setOrientation(i_a,i_b,i_g);
}


//=====================================================================
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
//---------------------------------------------------------------------
void CRFrameEuler::getPositionAndOrientation(double& o_x,
                                             double& o_y,
                                             double& o_z,
                                             double& o_a,
                                             double& o_b,
                                             double& o_g)
{
    getPosition(o_x,o_y,o_z);
    getOrientation(o_a,o_b,o_g);
}
    
    
    
//=====================================================================
/*!
 This method returns a true if the frame is driven (i.e. has a free 
 variable) or a false if the frame is not driven.\n
 
 \return - true = is driven, false = is not driven
 
 */
//---------------------------------------------------------------------
bool CRFrameEuler::isDriven(void) {
    if (m_freeVar == CR_EULER_FREE_NONE) {
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
    
    Eigen::Matrix3d r1 = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d r2 = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d r3 = Eigen::Matrix3d::Identity();
    
    switch (m_eulerMode){
        case CR_EULER_MODE_ZXZ:
			r1 = CRMath::rotAboutZ(m_angA);
			r2 = CRMath::rotAboutX(m_angB);
			r3 = CRMath::rotAboutZ(m_angG);
            break;
        case CR_EULER_MODE_XYX:
			r1 = CRMath::rotAboutX(m_angA);
			r2 = CRMath::rotAboutY(m_angB);
			r3 = CRMath::rotAboutX(m_angG);
            break;
        case CR_EULER_MODE_YZY:
			r1 = CRMath::rotAboutY(m_angA);
			r2 = CRMath::rotAboutZ(m_angB);
			r3 = CRMath::rotAboutY(m_angG);
            break;
        case CR_EULER_MODE_ZYZ:
			r1 = CRMath::rotAboutZ(m_angA);
			r2 = CRMath::rotAboutY(m_angB);
			r3 = CRMath::rotAboutZ(m_angG);
            break;
        case CR_EULER_MODE_XZX:
			r1 = CRMath::rotAboutX(m_angA);
			r2 = CRMath::rotAboutZ(m_angB);
			r3 = CRMath::rotAboutX(m_angG);
            break;
        case CR_EULER_MODE_YXY:
			r1 = CRMath::rotAboutY(m_angA);
			r2 = CRMath::rotAboutX(m_angB);
			r3 = CRMath::rotAboutY(m_angG);
            break;
        case CR_EULER_MODE_XYZ:
			r1 = CRMath::rotAboutX(m_angA);
			r2 = CRMath::rotAboutY(m_angB);
			r3 = CRMath::rotAboutZ(m_angG);
            break;
        case CR_EULER_MODE_YZX:
			r1 = CRMath::rotAboutY(m_angA);
			r2 = CRMath::rotAboutZ(m_angB);
			r3 = CRMath::rotAboutX(m_angG);
            break;
        case CR_EULER_MODE_ZXY:
			r1 = CRMath::rotAboutZ(m_angA);
			r2 = CRMath::rotAboutX(m_angB);
			r3 = CRMath::rotAboutY(m_angG);
            break;
        case CR_EULER_MODE_XZY:
			r1 = CRMath::rotAboutX(m_angA);
			r2 = CRMath::rotAboutZ(m_angB);
			r3 = CRMath::rotAboutY(m_angG);
            break;
        case CR_EULER_MODE_ZYX:
			r1 = CRMath::rotAboutZ(m_angA);
			r2 = CRMath::rotAboutY(m_angB);
			r3 = CRMath::rotAboutX(m_angG);
            break;
        case CR_EULER_MODE_YXZ:
			r1 = CRMath::rotAboutY(m_angA);
			r2 = CRMath::rotAboutX(m_angB);
			r3 = CRMath::rotAboutZ(m_angG);
            break;
    }
    m_rotation = r1*r2*r3;
    m_translation << m_posX, m_posY, m_posZ;
}


//=====================================================================
// End namespace
}


