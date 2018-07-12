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
Link::Link(){
    
}
    
    
//---------------------------------------------------------------------
/*!
 Destructor to the world item.\n
 */
//---------------------------------------------------------------------
Link::~Link(){
    // delete m_parent;
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
 This function gets the local frame transformation (relative to
 the parent frame).
 
 \return        the local frame transformation
 */
//---------------------------------------------------------------------
Frame Link::getLocalTransform()
{
    Frame f;
    Eigen::Matrix3d r = m_frame.getRotation();
    Eigen::Vector3d t = m_frame.getTranslation();
    f.setRotationAndTranslation(r, t);
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
    for (int i = 0; i < d; i++){
        i_stream << "  ";
    }
    std::string id = "N";
    if (isLeaf()) {
        id = "L";
    } else if (isRoot()) {
        id = "R";
    }
    std::string dof = "-";
    if (getDegreeOfFreedom() == CR_EULER_FREE_ANG_A) {
        dof = "A";
    } else if (getDegreeOfFreedom() == CR_EULER_FREE_ANG_B) {
        dof = "B";
    } else if (getDegreeOfFreedom() == CR_EULER_FREE_ANG_G) {
        dof = "G";
    } else if (getDegreeOfFreedom() == CR_EULER_FREE_POS_X) {
        dof = "X";
    } else if (getDegreeOfFreedom() == CR_EULER_FREE_POS_Y) {
        dof = "Y";
    } else if (getDegreeOfFreedom() == CR_EULER_FREE_POS_Z) {
        dof = "Z";
    }
    i_stream << "+ [" << id << "] world::Link DOF = " << dof << " '" << getName() << "'\n";
    for (int i = 0; i < m_children.size(); i++)
    {
        m_children.at(i)->print(i_stream);
    }
}


}
}
// end namespace
//---------------------------------------------------------------------


