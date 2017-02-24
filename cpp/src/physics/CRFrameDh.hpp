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

#ifndef CRFrameDh_hpp
#define CRFrameDh_hpp

//=====================================================================
// Includes
#include "Eigen/Dense"
#include "CRFrame.hpp"


//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics {
    
//=====================================================================
/*!
 \file CRFrameDh.hpp
 \brief Implements a derived frame class for handling DH parameter
 representations of a 3D affine transformation.
 */
//---------------------------------------------------------------------
/*!
 \class CRFrameDh
 \ingroup physics
 
 \brief This class implements a homogeneous transformation in the
 special Euclidean group made up of rotations and translations, where
 the rotations and translations are defined by DH parameters.
 
 \details
 \section Description
 CRFrameDh implements a DH parameter transformation specified by 4 
 parameters (<a href="wikipedia.org/wiki/Denavit–Hartenberg_parameters">
 see Wikipedia article).</a>
 
 Properties of the transform can be set with methods
 - CRFrameDh::setParameters sets the DH parameter values.
 - CRFrameDh::setMode sets the DH convention (options are in
 CoreRobotics::CRDhMode).
 
 The free variable can be specified by the CRFrameDh::freeVar member
 (options are in CoreRobotics::CRDhFreeVariable).
 
 \section Example
 This example creates a DH parameter frame class.
 \code
 
 #include "CoreRobotics.hpp"
 #include <stdio>
 
 main() {
    CoreRobotics::CRFrameDh Frame;

    Frame.setParameters(0.1, 0.0, 2.1, 0.7);

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
 [1] J. Denavit and R. Hartenberg, "A kinematic notation for lower-pair
 mechanisms based on matrices". Trans ASME J. Appl. Mech. 23, pp. 215-221, 1955.
 
 [2] J. Craig, "Introduction to Robotics: Mechanics and Control", Ed. 3,
 Pearson, 2004.
 
 */
//=====================================================================
//! Enumerator for handling DH parameter free variable candidates
enum CRDhFreeVariable {
    CR_DH_FREE_NONE,
    CR_DH_FREE_R,
    CR_DH_FREE_ALPHA,
    CR_DH_FREE_D,
    CR_DH_FREE_THETA,
};


//! Enumerator for handling DH conventions
enum CRDhMode {
    CR_DH_MODE_CLASSIC,
    CR_DH_MODE_MODIFIED,
};


//=====================================================================
class CRFrameDh : public CRFrame  {
    

//---------------------------------------------------------------------
// Constructor and Destructor
public:

    //! Class constructor
    CRFrameDh(double r, double alpha, double d, double theta,
              CRDhMode mode, CRDhFreeVariable free);
    CRFrameDh();

//---------------------------------------------------------------------
// Get/Set Methods
public:
    
    //! Set the value of the free variable
    bool setFreeValue(double q);
    
    //! Get the value of the free variable
    void getFreeValue(double &q);
    
    //! Set the DH convention
    void setMode(CRDhMode mode);
    
    //! Get the DH convention
    void getMode(CRDhMode &mode);
    
    //! Set the DH parameter values
    void setParameters(double r, double alpha, double d, double theta);
    
    //! Get the x position
    void getParameters(double &r, double &alpha, double &d, double &theta);
    
//---------------------------------------------------------------------
// Public Methods
public:
    
    //! Query if the frame is driven, i.e. has a free variable
    bool isDriven();
    
//---------------------------------------------------------------------
// Public Members
public:
    
    //! free variable indicator
    CRDhFreeVariable freeVar;
    
//---------------------------------------------------------------------
// Private Members
private:
    
    //! DH convention mode
    CRDhMode dhMode;

    //! r parameter
    double dh_r;
    
    //! d parameter
    double dh_d;
    
    //! alpha angle parameter [rad]
    double dh_alpha;
    
    //! theta angle parameter [rad]
    double dh_theta;
    
//---------------------------------------------------------------------
// Private Methods
private:
    
    //! Overload the inhereted method to set the rotation and
    //  translation explicitly for DH parameters
    using CRFrame::setRotationAndTranslation;
    void setRotationAndTranslation();

};

//=====================================================================
// End namespace
}

#endif
