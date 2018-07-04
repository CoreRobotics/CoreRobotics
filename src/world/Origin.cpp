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

#include "Origin.hpp"

//---------------------------------------------------------------------
// Begin namespace
namespace cr {
namespace world {
    
    
//---------------------------------------------------------------------
/*!
 The constructor sets up the world.\n
 */
//---------------------------------------------------------------------
Origin::Origin()
{
    m_rootItem = std::make_shared<Node>();
}


//---------------------------------------------------------------------
/*!
 The destructor deletes the world.\n
 */
//---------------------------------------------------------------------
Origin::~Origin()
{

}
    
    
//---------------------------------------------------------------------
/*!
 Create a whole new world.\n
 */
//---------------------------------------------------------------------
OriginPtr Origin::create()
{
    return std::make_shared<Origin>();
}
    
    
//---------------------------------------------------------------------
/*!
 Set the name.\n
 
 \param[in]     i_name - the name to set
 */
//---------------------------------------------------------------------
void Origin::setName(std::string i_name)
{
    Item::setName(i_name);
    m_rootItem->setName(i_name.append(" root"));
}
    
    
//---------------------------------------------------------------------
/*!
 Print the output scene.\n
 */
//---------------------------------------------------------------------
void Origin::print(std::ostream& i_stream)
{
    i_stream << "\nTree graph structure from world::Origin:\n\n";
    m_rootItem->print(i_stream);
    i_stream << "\n";
}
    
    

}
}
// end namespace
//---------------------------------------------------------------------



//---------------------------------------------------------------------
/*!
 Display operator details to an output stream.\n
 */
//---------------------------------------------------------------------
/*
std::ostream& operator<<(std::ostream& os, cr::OriginPtr obj)
{
    os << "obj->getName()" << "\n";
    return os;
}
 */

