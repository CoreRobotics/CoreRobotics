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

#ifndef CR_WORLDITEM_HPP_
#define CR_WORLDITEM_HPP_

#include <vector>
#include "Frame.hpp"

//---------------------------------------------------------------------
// Begin namespace
namespace cr {
    
//! WorldItem shared pointer
class WorldItem;
typedef std::shared_ptr<WorldItem> WorldItemPtr;
    
    
//---------------------------------------------------------------------
/*!
 \class WorldItem
 \ingroup core
 
 \brief
 This class implements a basic world scene graph item for attaching to
 a World thread element or other WorldItems.

 \details
 
 */
//---------------------------------------------------------------------
class WorldItem {
    
    // Constructor and Destructor
    public:
    
        //! Class constructor
        WorldItem();
    
        //! Class destructor
        ~WorldItem();
    
        //! Create a pointer
        static WorldItemPtr create();
    
    
    // Scene graph controls
    public:
    
        //! add a child to the list of children
        void addChild(WorldItem* i_item);
    
        //! remove a child from the list of children
        Result removeChild(WorldItem* i_item);
    
        //! set the parent of the item
        void setParent(WorldItem* i_item);
    
        //! get the parent of the child
        WorldItem* getParent();
    
    
    // Frame transformation controls
    public:
    
        //! set the local frame transformation
        void setLocalTransform(const Frame& i_frame);
    
        //! return the local frame transformation
        Frame getLocalTransform();
    
        //! return the global frame transformation
        Frame getGlobalTransform();
    
        //! return the relative frame transformation
        Frame getRelativeTransform(WorldItem* i_item);
    
    
    // protected member data
    protected:
    
        //! Transformation
        Frame m_frame;
    
    
    // private member data
    private:
    
        //! parent item
        WorldItem* m_parent = NULL;
    
        //! list of children
        std::vector<WorldItem*> m_children;
    
    
    
};

}
// end namespace
//---------------------------------------------------------------------

#endif
