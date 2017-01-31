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

#include "CRFrameDh.hpp"
#include "../../external/eigen/Eigen/Dense"


//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics {


//=====================================================================
/*!
 The constructor sets the rotation and translation parameters upon 
 construction, with defaults listed in parenthesis.\n
 
 \param   r      - x position of the frame (0)
 \param   alpha  - y position of the frame (0)
 \param   d      - z position of the frame (0)
 \param   theta  - alpha angle of the frame [rad] (0)
 \param   mode   - DH convention (CR_DH_MODE_MODIFIED)
 \param   free   - free variable (CR_DH_FREE_NONE)
 */
//=====================================================================
CRFrameDh::CRFrameDh()
{
    dh_r = 0.0;
    dh_alpha = 0.0;
    dh_d = 0.0;
    dh_theta = 0.0;
    dhMode = CR_DH_MODE_MODIFIED;
    freeVar = CR_DH_FREE_NONE;
    setRotationAndTranslation();
}
CRFrameDh::CRFrameDh(double r, double alpha, double d, double theta,
                     CRDhMode mode, CRDhFreeVariable free)
{
    dh_r = r;
    dh_alpha = alpha;
    dh_d = d;
    dh_theta = theta;
    dhMode = mode;
    freeVar = free;
    setRotationAndTranslation();
}


//=====================================================================
/*!
 The destructor frees up memory:\n
 */
//---------------------------------------------------------------------
CRFrameDh::~CRFrameDh()
{ }



//=====================================================================
/*!
 This method sets the value of the free variable.  The method returns a
 true if the value was written and a false if freeVar is set to 
 CR_DH_FREE_NONE.\n
 
 \param q - value of the variable to be set
 */
//---------------------------------------------------------------------
bool CRFrameDh::setFreeValue(double q)
{
    bool isWritable = true;
    switch (freeVar){
        case CR_DH_FREE_NONE:
            isWritable = false;
            break;
        case CR_DH_FREE_R:
            dh_r = q;
            break;
        case CR_DH_FREE_ALPHA:
            dh_alpha = q;
            break;
        case CR_DH_FREE_D:
            dh_d = q;
            break;
        case CR_DH_FREE_THETA:
            dh_theta = q;
        break;
    }
    setRotationAndTranslation();
    return isWritable;
}



//=====================================================================
/*!
 This method get the value of the free variable.  The method returns 
 q = NULL if freeVar is set to CR_DH_FREE_NONE.\n
 
 \param q - value of the free variable.
 */
//---------------------------------------------------------------------
void CRFrameDh::getFreeValue(double &q)
{
    switch (freeVar){
        case CR_DH_FREE_NONE:
            q = NULL;
            break;
        case CR_DH_FREE_R:
            q = dh_r;
            break;
        case CR_DH_FREE_ALPHA:
            q = dh_alpha;
            break;
        case CR_DH_FREE_D:
            q = dh_d;
            break;
        case CR_DH_FREE_THETA:
            q = dh_theta;
            break;
    }
}


//=====================================================================
/*!
 This method sets the the DH convention.\n
 
 \param mode - DH convention
 */
//---------------------------------------------------------------------
void CRFrameDh::setMode(CRDhMode mode)
{
    dhMode = mode;
    setRotationAndTranslation();
}



//=====================================================================
/*!
 This method gets the DH convention.\n
 
 \param mode - DH convention.
 */
//---------------------------------------------------------------------
void CRFrameDh::getMode(CRDhMode &mode)
{
    mode = dhMode;
}


//=====================================================================
/*!
 This method sets the DH parameter values.\n
 
 \param   x   - x position of the frame
 \param   y   - y position of the frame
 \param   z   - z position of the frame
 */
//---------------------------------------------------------------------
void CRFrameDh::setParameters(double r, double alpha, double d, double theta)
{
    dh_r = r;
    dh_alpha = alpha;
    dh_d = d;
    dh_theta = theta;
    setRotationAndTranslation();
}


//=====================================================================
/*!
 This method gets the DH parameter values.\n
 
 \param   x   - x position of the frame
 \param   y   - y position of the frame
 \param   z   - z position of the frame
 */
//---------------------------------------------------------------------
void CRFrameDh::getParameters(double &r, double &alpha, double &d, double &theta)
{
    r = dh_r;
    alpha = dh_alpha;
    d = dh_d;
    theta = dh_theta;
}



//=====================================================================
// Private Methods:

//! sets the private rotation and translation members - Note that
//  anytime a parameter gets set in the frame class, this method gets
//  called to update the rotation/translation members.
void CRFrameDh::setRotationAndTranslation()
{
    switch (dhMode){
        case CR_DH_MODE_CLASSIC:
            rotation << cos(dh_theta), -sin(dh_theta)*cos(dh_alpha), sin(dh_theta)*sin(dh_alpha),
            sin(dh_theta), cos(dh_theta)*cos(dh_alpha), -cos(dh_theta)*sin(dh_alpha),
            0, sin(dh_alpha), cos(dh_alpha);
            translation << dh_r*cos(dh_theta), dh_r*sin(dh_theta), dh_d;
            break;
        case CR_DH_MODE_MODIFIED:
            rotation << cos(dh_theta), -sin(dh_theta), 0,
            sin(dh_theta)*cos(dh_alpha), cos(dh_theta)*cos(dh_alpha), -sin(dh_alpha),
            sin(dh_theta)*sin(dh_alpha), cos(dh_theta)*sin(dh_alpha), cos(dh_alpha);
            translation << dh_r, -dh_d*sin(dh_alpha), dh_d*cos(dh_alpha);
            break;
    }
}


//=====================================================================
// End namespace
}


