//=====================================================================
/*
 Software License Agreement (BSD-3-Clause License)
 Copyright (c) 2019, CoreRobotics.
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

#ifndef CR_RIGIDBODY_HPP_
#define CR_RIGIDBODY_HPP_


#include "Frame.hpp"


//---------------------------------------------------------------------
// Begin namespace
namespace cr {
namespace physics {


//---------------------------------------------------------------------
/*!
 \class RigidBody
 \ingroup physics
 
 \brief This class is a container for rigid body transformations and
 dynamic properties necessary for multibody representations.
 
 \details
 ## Description
 RigidBody implements a container for representing rigid body
 dynamics.  At a minimum it points to a cr::Frame class or
 derived subclass.
 
 ## Example
 This example creates an Rigid body object.
 \include example_Manipulator.cpp
 
 ## References
 [1] J. Craig, "Introduction to Robotics: Mechanics and Control", Ed. 3,
 Pearson, 2004.
 */
//---------------------------------------------------------------------
class RigidBody {
    

    // Constructor and Destructor
    public:
    
        //! Class constructor
        RigidBody();
    
        //! Class destructor
        virtual ~RigidBody();
    

    // Get/Set Methods
    public:
    
        //! Set the center of mass
        void setCenterOfMass(const Eigen::Vector3d& i_com) { m_com = i_com; }
    
        //! Get the center of mass
        Eigen::Vector3d getCenterOfMass() { return m_com; }
    
        //! Set the inertia tensor
        void setInertiaTensor(const Eigen::Matrix3d& i_inertia) { m_inertia = i_inertia; }
    
        //! Get the inertia tensor
        Eigen::Matrix3d getInertiaTensor() { return m_inertia; }
    
        //! Set the mass value
        void setMass(const double& i_mass) { m_mass = i_mass; }
    
        //! Get the mass value
        double getMass() { return m_mass; }
    
        //! get the mass matrix
        Eigen::Matrix<double, 6, 6> getMassMatrix();
    
    
    // Private Members
    private:
    
        //! center of mass
        Eigen::Vector3d m_com;
    
        //! inertia tensor
        Eigen::Matrix3d m_inertia;
    
        //! mass value
        double m_mass;
    
    };

}
}
// end namespace
//---------------------------------------------------------------------


#endif
