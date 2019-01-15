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

#include "Link.hpp"

//---------------------------------------------------------------------
// Begin namespace
namespace cr {
namespace world {
    
    
//---------------------------------------------------------------------
/*!
 Constructor to the world item.\n
 */
//---------------------------------------------------------------------
Link::Link() : m_comItem( std::make_shared<Node>() ) {
    m_comItem->setName("COM");
}
    
    
//---------------------------------------------------------------------
/*!
 Destructor to the world item.\n
 */
//---------------------------------------------------------------------
Link::~Link(){
    
}
    
    
//---------------------------------------------------------------------
/*!
 Create a new world item.\n
 */
//---------------------------------------------------------------------
LinkPtr Link::create(){
    return std::make_shared<Link>();
}
    
    
//---------------------------------------------------------------------
/*!
 This function sets the rigid body parameters and updates internal data.
 
 \param[in]        i_body - the RigidBody data
 */
//---------------------------------------------------------------------
void Link::setRigidBody(const physics::RigidBody& i_body)
{
    addChild(m_comItem);
    m_body = i_body;
    physics::Frame f;
    f.setTranslation( m_body.getCenterOfMass() );
    m_comItem->setLocalTransform(f);
}
    
    
//---------------------------------------------------------------------
/*!
 This function gets the local frame transformation (relative to
 the parent frame).
 
 \return        the local frame transformation
 */
//---------------------------------------------------------------------
physics::Frame Link::getLocalTransform()
{
    physics::Frame f;
    Eigen::Matrix3d r = m_frame.getRotation();
    Eigen::Vector3d t = m_frame.getTranslation();
    f.setRotationAndTranslation(r, t);
    return f;
}
    
    
//---------------------------------------------------------------------
/*!
 This function gets the global frame transformation (relative to
 the world frame).
 
 \return        the local frame transformation
 */
//---------------------------------------------------------------------
physics::Frame Link::getGlobalTransform()
{
    Eigen::Matrix4d T = m_frame.getTransformToParent();
    NodePtr parent = m_parent;
    while (parent != NULL){
        T = parent->getLocalTransform().getTransformToParent() * T;
        parent = parent->getParent();
    }
    physics::Frame f;
    f.setRotationAndTranslation(T.topLeftCorner(3, 3), T.topRightCorner(3, 1));
    return f;
}


//---------------------------------------------------------------------
/*!
 This function gets a relative frame transformation (relative to
 the frame of the intput item).
 
 \return        the relative frame transformation
 */
//---------------------------------------------------------------------
physics::Frame Link::getRelativeTransform(NodePtr i_item)
{
    Eigen::Matrix4d T0 = this->getGlobalTransform().getTransformToParent();
    Eigen::Matrix4d T = i_item->getGlobalTransform().getTransformToChild() * T0;
    physics::Frame f;
    f.setRotationAndTranslation(T.topLeftCorner(3, 3), T.topRightCorner(3, 1));
    return f;
}
    
    
//---------------------------------------------------------------------
/*!
 Print the children.\n
 */
//---------------------------------------------------------------------
void Link::print(std::ostream& i_stream)
{
    unsigned d = getDepth();
    for (unsigned i = 0; i < d; i++){
        i_stream << "  ";
    }
    std::string id = "N";
    if (isLeaf()) {
        id = "L";
    } else if (isRoot()) {
        id = "R";
    }
    std::string dof = "-";
    if (getDegreeOfFreedom() == physics::CR_EULER_FREE_ANG_A) {
        dof = "A";
    } else if (getDegreeOfFreedom() == physics::CR_EULER_FREE_ANG_B) {
        dof = "B";
    } else if (getDegreeOfFreedom() == physics::CR_EULER_FREE_ANG_G) {
        dof = "G";
    } else if (getDegreeOfFreedom() == physics::CR_EULER_FREE_POS_X) {
        dof = "X";
    } else if (getDegreeOfFreedom() == physics::CR_EULER_FREE_POS_Y) {
        dof = "Y";
    } else if (getDegreeOfFreedom() == physics::CR_EULER_FREE_POS_Z) {
        dof = "Z";
    }
    i_stream << "+ [" << id << "] world::Link DOF = " << dof << " '" << getName() << "'\n";
    for (unsigned i = 0; i < m_children.size(); i++)
    {
        m_children.at(i)->print(i_stream);
    }
}


}
}
// end namespace
//---------------------------------------------------------------------


