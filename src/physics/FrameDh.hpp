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
\author  Parker Owan, Tony Piaskowy

*/
//=====================================================================

#ifndef FrameDh_hpp
#define FrameDh_hpp

//=====================================================================
// Includes
#include "Eigen/Dense"
#include "Frame.hpp"


//=====================================================================
// CoreRobotics namespace
namespace cr {
    
//=====================================================================
/*!
 \file FrameDh.hpp
 \brief Implements a derived frame class for handling DH parameter
 representations of a 3D affine transformation.
 */
//---------------------------------------------------------------------
/*!
 \class FrameDh
 \ingroup physics
 
 \brief This class implements a homogeneous transformation in the
 special Euclidean group made up of rotations and translations, where
 the rotations and translations are defined by DH parameters.
 
 \details
 ## Description
 FrameDh implements a DH parameter transformation specified by 4 
 parameters (<a href="wikipedia.org/wiki/Denavit–Hartenberg_parameters">
 see Wikipedia article).</a> With an additional 5th paramter for constant 
 offsets for the driven variable.
 
 Properties of the transform can be set with methods
 - FrameDh::setParameters sets the DH parameter values.
 - FrameDh::setMode sets the DH convention (options are in
 cr::DhMode).
 
 The free variable can be specified by the FrameDh::m_freeVar member
 (options are in cr::DhFreeVariable).
 
 ## Example
 This example creates a DH parameter frame class.
 \include example_FrameDh.cpp
 
 ## References
 [1] J. Denavit and R. Hartenberg, "A kinematic notation for lower-pair
 mechanisms based on matrices". Trans ASME J. Appl. Mech. 23, pp. 215-221, 1955.
 
 [2] J. Craig, "Introduction to Robotics: Mechanics and Control", Ed. 3,
 Pearson, 2004.
 
 */
//=====================================================================
//! Enumerator for handling DH parameter free variable candidates
enum DhFreeVariable {
    CR_DH_FREE_NONE,
    CR_DH_FREE_R,
    CR_DH_FREE_ALPHA,
    CR_DH_FREE_D,
    CR_DH_FREE_THETA,
};


//! Enumerator for handling DH conventions
enum DhMode {
    CR_DH_MODE_CLASSIC,
    CR_DH_MODE_MODIFIED,
};


//=====================================================================
class FrameDh : public Frame  {
    

//---------------------------------------------------------------------
// Constructor and Destructor
public:

    //! Class constructor
    FrameDh(double i_r,
              double i_alpha,
              double i_d,
              double i_theta,
              DhMode i_mode,
              DhFreeVariable i_free);
    FrameDh();

//---------------------------------------------------------------------
// Get/Set Methods
public:
    
    //! Set the value of the free variable
    Result setFreeValue(double i_q);
    
    //! Get the value of the free variable
    double getFreeValue(void);
    
    //! Set the free variable
    void setFreeVariable(DhFreeVariable i_free) {m_freeVar = i_free;}
    
    //! Get the free variable
    DhFreeVariable getFreeVariable(void) {return m_freeVar;}
    
    //! Set the DH convention
    void setMode(DhMode i_mode);
    
    //! Get the DH convention
    DhMode getMode(void);
    
    //! Set the DH parameter values
    void setParameters(double i_r,
                       double i_alpha,
                       double i_d,
                       double i_theta);
    
    //! Get the DH parameter values
    void getParameters(double& o_r,
                       double& o_alpha,
                       double& o_d,
                       double& o_theta);
    
//---------------------------------------------------------------------
// Public Methods
public:
    
    //! Query if the frame is driven, i.e. has a free variable
    bool isDriven(void);

    
//---------------------------------------------------------------------
// Private Members
private:
    
    //! free variable indicator
    DhFreeVariable m_freeVar;
    
    //! DH convention mode
    DhMode m_dhMode;

    //! r parameter
    double m_dhR;
    
    //! d parameter
    double m_dhD;
    
    //! alpha angle parameter [rad]
    double m_dhAlpha;
    
    //! theta angle parameter [rad]
    double m_dhTheta;

	//! free variable offset parameter [m] or [rad]
    // double m_freeVarOffset;
    
//---------------------------------------------------------------------
// Private Methods
private:
    
    //! Update the rotation and translation matrices from data
    using Frame::setRotationAndTranslation;
    void setRotationAndTranslation();

};

//=====================================================================
// End namespace
}

#endif
