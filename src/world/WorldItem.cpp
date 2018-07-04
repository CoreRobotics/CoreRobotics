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

#include "WorldItem.hpp"

//---------------------------------------------------------------------
// Begin namespace
namespace cr {
    
    
//---------------------------------------------------------------------
/*!
 Constructor to the world item.\n
 */
//---------------------------------------------------------------------
WorldItem::WorldItem(){
    
}
    
    
//---------------------------------------------------------------------
/*!
 Destructor to the world item.\n
 */
//---------------------------------------------------------------------
WorldItem::~WorldItem(){
    // delete m_parent;
}
    
    
//---------------------------------------------------------------------
/*!
 Create a new world item.\n
 */
//---------------------------------------------------------------------
WorldItemPtr WorldItem::create(){
    return std::make_shared<WorldItem>();
}
    
    
//---------------------------------------------------------------------
/*!
 This function adds an item to the list of children
 
 \param[in]     i_item - the child to add
 */
//---------------------------------------------------------------------
void WorldItem::addChild(WorldItemPtr i_item)
{
    i_item->setParent(shared_from_this());
    m_children.push_back(i_item);
}


//---------------------------------------------------------------------
/*!
 This function removes an item from the list of children
 
 \param[in]     i_item - the child to remove
 \return        result of the remove operation
                (CR_RESULT_SUCCESS or CR_RESULT_NOT_FOUND)
 */
//---------------------------------------------------------------------
Result WorldItem::removeChild(WorldItemPtr i_item)
{
    for (int k = 0; k < m_children.size(); k++){
        if (m_children.at(k) == i_item)
        {
            m_children.erase(m_children.begin()+k);
            i_item->setParent(NULL);
            return CR_RESULT_SUCCESS;
        }
    }
    return CR_RESULT_NOT_FOUND;
}
    
    
//---------------------------------------------------------------------
/*!
 This function sets the parent of the item.
 
 \param[in]     i_item - the parent item
 */
//---------------------------------------------------------------------
void WorldItem::setParent(WorldItemPtr i_item)
{
    m_parent = i_item;
}
    
    
//---------------------------------------------------------------------
/*!
 This function return the parent of the item.
 
 \return        the parent item
 */
//---------------------------------------------------------------------
WorldItemPtr WorldItem::getParent()
{
    return m_parent;
}


//---------------------------------------------------------------------
/*!
 This function returns a flag indicating if it is a leaf (i.e. no children).
 
 \return        flag (true = if there are no children)
 */
//---------------------------------------------------------------------
bool WorldItem::isLeaf()
{
    bool isLeaf = true;
    if (m_children.size() > 0)
        isLeaf = false;
    return isLeaf;
}
    
    
//---------------------------------------------------------------------
/*!
 This function returns the depth of the node in the tree
 
 \return        integer depth (0 = root)
 */
//---------------------------------------------------------------------
unsigned WorldItem::getDepth()
{
    unsigned d = 0;
    WorldItemPtr parent = getParent();
    while ( parent != NULL) {
        d++;
        parent = parent->getParent();
    }
    return d;
}
    
    
//---------------------------------------------------------------------
/*!
 This function sets the local frame transformation (relative to
 the parent frame).
 
 \param[in]     i_frame - the local frame transformation
 */
//---------------------------------------------------------------------
void WorldItem::setLocalTransform(const Frame& i_frame)
{
    m_frame = i_frame;
}
    
    
//---------------------------------------------------------------------
/*!
 This function gets the local frame transformation (relative to
 the parent frame).
 
 \return        the local frame transformation
 */
//---------------------------------------------------------------------
Frame WorldItem::getLocalTransform()
{
    return m_frame;
}
    
    
//---------------------------------------------------------------------
/*!
 This function gets the global frame transformation (relative to
 the world frame).
 
 \return        the local frame transformation
 */
//---------------------------------------------------------------------
Frame WorldItem::getGlobalTransform()
{
    Eigen::Matrix4d T = m_frame.getTransformToParent();
    WorldItemPtr parent = m_parent;
    while (parent != NULL){
        T = parent->getLocalTransform().getTransformToParent() * T;
        parent = parent->getParent();
    }
    Frame f;
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
Frame WorldItem::getRelativeTransform(WorldItemPtr i_item)
{
    Eigen::Matrix4d T0 = this->getGlobalTransform().getTransformToParent();
    Eigen::Matrix4d T = i_item->getGlobalTransform().getTransformToChild() * T0;
    Frame f;
    f.setRotationAndTranslation(T.topLeftCorner(3, 3), T.topRightCorner(3, 1));
    return f;
}
    
    
//---------------------------------------------------------------------
/*!
 Print the children.\n
 */
//---------------------------------------------------------------------
void WorldItem::print(std::ostream& i_stream)
{
    unsigned d = getDepth();
    for (int i = 0; i < d; i++){
        i_stream << "  ";
    }
    i_stream << "+ cr::WorldItem '" << getName() << "'\n";
    for (int i = 0; i < m_children.size(); i++)
    {
        m_children.at(i)->print(i_stream);
    }
}


}
// end namespace
//---------------------------------------------------------------------


