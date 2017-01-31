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

#ifndef CRFrameEuler_hpp
#define CRFrameEuler_hpp

//=====================================================================
// Includes
#include "../../external/eigen/Eigen/Dense"
#include "CRFrame.hpp"


//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics {
    
//=====================================================================
/*!
 \file CRFrameEuler.hpp
 \brief Implements a derived frame class for handling Euler rotations
 in a 3D affine transformation.
 */
//---------------------------------------------------------------------
/*!
 \class CRFrameEuler
 \ingroup physics
 
 \brief This class implements a homogeneous transformation in the
 special Euclidean group made up of rotations and translations, where
 the rotations are defined by Euler angles.
 
 \details
 CRFrameEuler implements a homogeneous transformation
 */
//=====================================================================
//! Enumerator for handling Euler angle free variable candidates
enum CREulerFreeVariable {
    CR_EULER_FREE_NONE,
    CR_EULER_FREE_POS_X,
    CR_EULER_FREE_POS_Y,
    CR_EULER_FREE_POS_Z,
    CR_EULER_FREE_ANG_A,
    CR_EULER_FREE_ANG_B,
    CR_EULER_FREE_ANG_G,
};


//=====================================================================
class CRFrameEuler : public CRFrame  {

//---------------------------------------------------------------------
// Constructor and Destructor
public:

    //! Class constructor
    CRFrameEuler();
    CRFrameEuler(double x, double y, double z, double a, double b,
                 double g, CREulerMode mode, CREulerFreeVariable free);

    //! Class destructor
    virtual ~CRFrameEuler();


//---------------------------------------------------------------------
// Get/Set Methods
public:
    
    //! Set the value of the free variable
    bool setFreeValue(double q);
    
    //! Get the value of the free variable
    void getFreeValue(double &q);
    
    //! Set the Euler convention
    void setMode(CREulerMode mode);
    
    //! Get the Euler convention
    void getMode(CREulerMode &mode);
    
    //! Set the position values
    void setPosition(double x, double y, double z);
    
    //! Get the x position
    void getPosition(double &x, double &y, double &z);
    
    //! Set the angle values
    void setOrientation(double a, double b, double g);
    
    //! Get the angle values
    void getOrientation(double &a, double &b, double &g);
    
    //! Set the position and angle values
    void setPositionAndOrientation(double x, double y, double z, double a, double b, double g);
    
    //! Get the position and angle values
    void getPositionAndOrientation(double &x, double &y, double &z, double &a, double &b, double &g);
    

    
//---------------------------------------------------------------------
// Public Members
public:
    
    //! free variable indicator
    CREulerFreeVariable freeVar;
    
//---------------------------------------------------------------------
// Private Members
private:
    
    //! Euler convention
    CREulerMode eulerMode;

    //! x position
    double pos_x;
    
    //! y position
    double pos_y;
    
    //! z position
    double pos_z;
    
    //! alpha angle [rad]
    double ang_a;
    
    //! beta angle [rad]
    double ang_b;
    
    //! gamma angle [rad]
    double ang_g;
    
    
//---------------------------------------------------------------------
// Private Methods
private:
    
    //! Overload the inhereted method to set the rotation and
    //  translation explicitly for Euler angles
    void setRotationAndTranslation();
    
    //! standard rotation about the x axis
    void rotAboutX(double ang, Eigen::Matrix3d &rot);
    
    //! standard rotation about the y axis
    void rotAboutY(double ang, Eigen::Matrix3d &rot);
    
    //! standard rotation about the z axis
    void rotAboutZ(double ang, Eigen::Matrix3d &rot);


};

//=====================================================================
// End namespace
}

#endif
