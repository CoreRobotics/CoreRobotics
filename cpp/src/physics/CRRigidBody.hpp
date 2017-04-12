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

#ifndef CRRigidBody_hpp
#define CRRigidBody_hpp

//=====================================================================
// Includes
#include "CRFrame.hpp"
#include "CRFrameDh.hpp"
#include "CRFrameEuler.hpp"


//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics {
    
//=====================================================================
/*!
 \file CRRigidBody.hpp
 \brief Implements a container class for rigid body transformations and
 dynamics properties.
 */
//---------------------------------------------------------------------
/*!
 \class CRRigidBody
 \ingroup physics
 
 \brief This class is a container for rigid body transformations and
 dynamic properties necessary for multibody representations.
 
 \details
 \section Description
 CRRigidBody implements a container for representing rigid body
 dynamics.  At a minimum it points to a CoreRobotics::CRFrame class or
 derived subclass.
 
 \section Example
 This example creates an Rigid body object.
 \code
 
 #include "CoreRobotics.hpp"
 #include <stdio>
 
 main() {
 
    CoreRobotics::CRRigidBody Link;
    CoreRobotics::CRFrame* Frame = new CoreRobotics::CRFrame();

    Link.setFrame(Frame);
 }
 
 \endcode
 
 \section References
 [1] J. Craig, "Introduction to Robotics: Mechanics and Control", Ed. 3,
 Pearson, 2004.
 */
//=====================================================================
class CRRigidBody {
    
//---------------------------------------------------------------------
// Constructor and Destructor
public:
    
    //! Class constructor
    CRRigidBody(CRFrame* i_frame);
    CRRigidBody();
    
    //! Class destructor
    // virtual ~CRRigidBody() = 0;
    
//---------------------------------------------------------------------
// Get/Set Methods
public:
    
    //! Sets the pointer to the frame class for the transformation
    void setFrame(CRFrame* i_frame) {this->m_frame = i_frame;}
    
//---------------------------------------------------------------------
// Public Members
public:
    
    //! Pointer to the rigid body frame transformation
    CoreRobotics::CRFrame* m_frame;
    
};

//=====================================================================
// End namespace
}

#endif
