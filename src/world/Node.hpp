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

#ifndef CR_NODE_HPP_
#define CR_NODE_HPP_

#include "Frame.hpp"
#include "Item.hpp"
#include <vector>
#include <memory>

//---------------------------------------------------------------------
// Begin namespace
namespace cr {
namespace world {
    
//! Node shared pointer
class Node;
typedef std::shared_ptr<Node> NodePtr;
    
    
//---------------------------------------------------------------------
/*!
 \class Node
 \ingroup world
 
 \brief
 This class implements a basic world scene graph item for attaching to
 a Origin thread element or other Nodes.

 \details
 
 */
//---------------------------------------------------------------------
class Node
    : public std::enable_shared_from_this<Node>, public Item
{
    
    // Constructor and Destructor
    public:
    
        //! Class constructor
        Node();
    
        //! Class destructor
        ~Node();
    
        //! Create a pointer
        static NodePtr create();
    
    
    // Scene graph controls
    public:
    
        //! add a child to the list of children
        void addChild(NodePtr i_item);
    
        //! remove a child from the list of children
        Result removeChild(NodePtr i_item);
    
        //! set the parent of the item
        void setParent(NodePtr i_item);
    
        //! get the parent of the child
        NodePtr getParent();
    
        //! return if the item is a leaf (i.e. no children)
        bool isLeaf();
    
        //! return the depth of the node (0 = root)
        unsigned getDepth();
    
    
    // Frame transformation controls
    public:
    
        //! set the local frame transformation
        void setLocalTransform(const Frame& i_frame);
    
        //! return the local frame transformation
        Frame getLocalTransform();
    
        //! return the global frame transformation
        Frame getGlobalTransform();
    
        //! return the relative frame transformation
        Frame getRelativeTransform(NodePtr i_item);
    
    
    // Print details
    public:
    
        //! print the scene
        void print(std::ostream& i_stream);
    
    
    // protected member data
    protected:
    
        //! Transformation
        Frame m_frame;
    
    
    // private member data
    private:
    
        //! parent item
        NodePtr m_parent = NULL;
    
        //! list of children
        std::vector<NodePtr> m_children;
    
    
    
};

}
}
// end namespace
//---------------------------------------------------------------------

#endif
