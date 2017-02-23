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
 \section Description
 CRFrameEuler implements an transformation where the rotation is 
 specified by three Euler angles (<a href="wikipedia.org/wiki/Euler_angles">
 see Wikipedia article).</a>
 
 Properties of the transform can be set with methods
 - CRFrameEuler::setPosition, CRFrameEuler::setOrientation and
 CRFrameEuler::setPositionAndOrientation set transformation values.
 - CRFrameEuler::setMode sets the Euler angle convention (options are in
 CoreRobotics::CREulerMode).
 
 The free variable can be specified by the CRFrameEuler::freeVar member
 (options are in CoreRobotics::CREulerFreeVariable).
 
 \section Example
 This example creates an Euler frame class.
 \code
 
 #include "CoreRobotics.hpp"
 #include <stdio>
 
 main() {
    CoreRobotics::CRFrameEuler Frame;

    Frame.setPositionAndOrientation(0, 0, 1, 0, 0, 0.7);
    Frame.setMode(CoreRobotics::CR_EULER_MODE_XYZ);

    Eigen::Matrix4d T;
    Frame.getTransformToParent(T);
    std::cout << "Transformation to parent\n" << T << std::endl;

    Eigen::Vector3d p, y;
    p << 5, 6, 7;
    Frame.transformToParent(p, y);
    std::cout << "Point " << p.transpose() << " transformed to "
    << y.transpose() << std::endl;
 }
 
 \endcode
 
 \section References
 [1] J. Craig, "Introduction to Robotics: Mechanics and Control", Ed. 3,
 Pearson, 2004.
 
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
    CRFrameEuler(double x, double y, double z, double a, double b,
                 double g, CREulerMode mode, CREulerFreeVariable free);
    CRFrameEuler();

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
// Public Methods
public:
    
    //! Query if the frame is driven, i.e. has a free variable
    bool isDriven();

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
    virtual void setRotationAndTranslation();
    
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
