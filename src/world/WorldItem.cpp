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
namespace CoreRobotics {
    
    
//---------------------------------------------------------------------
/*!
 The constructor sets up the world item.\n
 */
//---------------------------------------------------------------------
WorldItem::WorldItem()
{
    
}


//---------------------------------------------------------------------
/*!
 The destructor deletes the world item.\n
 */
//---------------------------------------------------------------------
WorldItem::~WorldItem()
{
	delete m_parent;
}


//---------------------------------------------------------------------
/*!
Add a child to the list of this element's children

\param[in]		i_child - pointer to the child being added
*/
//---------------------------------------------------------------------
void WorldItem::addChild(WorldItem* i_child)
{
	m_children.push_back(i_child);
	i_child->setParent(this);
}


//---------------------------------------------------------------------
/*!
Sets the parent to this element

\param[in]		i_parent - pointer to the parent
*/
//---------------------------------------------------------------------
void WorldItem::setParent(WorldItem* i_parent)
{
	m_parent = i_parent;
}


}
// end namespace
//---------------------------------------------------------------------

