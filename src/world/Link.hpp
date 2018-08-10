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
//---------------------------------------------------------------------
// Begin header definition

#ifndef CR_LINK_HPP_
#define CR_LINK_HPP_

#include "FrameEuler.hpp"
#include "RigidBody.hpp"
#include "Node.hpp"
#include <vector>
#include <memory>

//---------------------------------------------------------------------
// Begin namespace
namespace cr {
namespace world {
    
//! Link shared pointer
class Link;
typedef std::shared_ptr<Link> LinkPtr;
    
    
//---------------------------------------------------------------------
/*!
 \class Link
 \ingroup world
 
 \brief
 This class implements a link item for capturing driven manipulator
 kinematics and dynamics.

 \details
 
 */
//---------------------------------------------------------------------
class Link : public Node
{
    
    // Constructor and Destructor
    public:
    
        //! Class constructor
        Link();
    
        //! Class destructor
        ~Link();
    
        //! Create a pointer
        static LinkPtr create();
    
    
    // Link controls
    public:
    
        //! get the pointer to the center of mass node
        NodePtr getCenterOfMass() { return m_comItem; }
    
        //! set the rigid body parameters
        void setRigidBody(const RigidBody& i_body);
    
        //! return the rigid body parameters
        RigidBody getRigidBody() { return m_body; }
    
        //! set the degree of freedom
        void setDegreeOfFreedom(EulerFreeVariable i_dof) { m_frame.setFreeVariable(i_dof); }
    
        //! return the degree of freedom
        EulerFreeVariable getDegreeOfFreedom() { return m_frame.getFreeVariable(); }
    
        //! set the driven value
        void setFreeValue(const double& i_value) { m_frame.setFreeValue(i_value); }
    
        //! return the driven value
        double getFreeValue() { return m_frame.getFreeValue(); }

		//! set the driven velocity
		void setFreeVelocity(const double& i_velocity) { m_velocity = i_velocity; }

		//! return the driven velocity
		double getFreeVelocity() { return m_velocity; }
    
        //! set euler mode
        void setEulerMode(const EulerMode& i_mode) { m_frame.setMode(i_mode); }
    
        //! get euler mode
        EulerMode getEulerMode() { return m_frame.getMode(); }
    
    
    // Frame transformation controls
    public:
    
        //! set the local frame transformation
        void setLocalTransform(const FrameEuler& i_frame) { m_frame = i_frame; }
    
        //! return the local frame transformation
        virtual Frame getLocalTransform();
    
        //! return the global frame transformation
        virtual Frame getGlobalTransform();
    
        //! return the relative frame transformation
        virtual Frame getRelativeTransform(NodePtr i_item);
    
    
    // Print details
    public:
    
        //! print the scene
        virtual void print(std::ostream& i_stream);
    
    
    // protected member data
    protected:
    
        //! Rigid body
        RigidBody m_body;
    
        //! frame - overloaded from Node::m_frame
        FrameEuler m_frame;

		//! free value velocity
		double m_velocity = 0;
    
        //! center of mass node
        NodePtr m_comItem;
    
        //! type (read only)
        std::string m_type = "Link";
    
};

}
}
// end namespace
//---------------------------------------------------------------------

#endif
