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

#ifndef CR_ORIGIN_HPP_
#define CR_ORIGIN_HPP_

#include "Node.hpp"
#include "core/Item.hpp"
#include <memory>

//---------------------------------------------------------------------
// Begin namespace
namespace cr {
namespace world {
    
//! Origin shared pointer
class Origin;
typedef std::shared_ptr<Origin> OriginPtr;
    
    
//---------------------------------------------------------------------
/*!
 \class Origin
 \ingroup world
 
 \brief
 This class implements a scene graph node that represents the world
 origin, and thus cannot have a parent.
 
 \details
 
 */
//---------------------------------------------------------------------
class Origin : public core::Item
{
    
    // Constructor and Destructor
    public:
    
        //! Class constructor
		Origin();
    
        //! Class destructor
        ~Origin();
    
        //! Create a pointer
        static OriginPtr create();
    
    
    // naming
    public:
    
        //! set the name
        virtual void setName(std::string i_name);


	// Tree graph behaviors
	public:

		//! add a child to the list of children
        void addChild(NodePtr i_item) { m_rootItem->addChild(i_item); }
    
        //! remove a child from the list of children
        void removeChild(NodePtr i_item) { m_rootItem->removeChild(i_item); }
    
    
    // Print details
    public:
    
        //! print out the scene
        virtual void print(std::ostream& i_stream);

    
    // Private members
    private:
    
        //! add a world item
        NodePtr m_rootItem;
    
        //! type (read only)
        std::string m_type = "Origin";
    
};

}
}
// end namespace
//---------------------------------------------------------------------


//! Origin display operator overload
// https://stackoverflow.com/questions/476272/how-to-properly-overload-the-operator-for-an-ostream
// std::ostream& operator<<(std::ostream&, const cr::Origin&);



#endif
