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
// Begin header definition

#ifndef CR_NODE_HPP_
#define CR_NODE_HPP_

#include "physics/Frame.hpp"
#include "core/Item.hpp"
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
    : public std::enable_shared_from_this<Node>, public core::Item
{
    
    // Constructor and Destructor
    public:
    
        //! Class constructor
        Node();
    
        //! Class destructor
        ~Node();
    
        //! Create a pointer
        static NodePtr create();
    
    
    // Tree graph controls
    public:
    
        //! add a child to the list of children
        void addChild(NodePtr i_item);
    
        //! remove a child from the list of children
        core::Result removeChild(NodePtr i_item);
    
        //! set the parent of the item
        void setParent(NodePtr i_item);
    
        //! get the parent of the child
        NodePtr getParent();
    
        //! check if a node is an ancestor (parent at any level) of this node
        bool isAncestor(NodePtr i_node);
    
        //! check if a node is an descendant (child at any level) of this node
        bool isDescendent(NodePtr i_node);
    
        //! return if the item is a leaf (i.e. no children)
        bool isLeaf();
    
        //! return if the item is a root (i.e. no parent)
        bool isRoot();
    
        //! return the depth of the node (0 = root)
        unsigned getDepth();
    
    
    // Frame transformation controls
    public:
    
        //! set the local frame transformation
        void setLocalTransform(const physics::Frame& i_frame);
    
        //! return the local frame transformation
        virtual physics::Frame getLocalTransform();
    
        //! return the global frame transformation
        virtual physics::Frame getGlobalTransform();
    
        //! return the relative frame transformation
        virtual physics::Frame getRelativeTransform(NodePtr i_item);
    
    
    // Print details
    public:
    
        //! print the scene
        virtual void print(std::ostream& i_stream);
    
    
    // protected member data
    protected:
    
        //! Transformation
        physics::Frame m_frame;
    
        //! parent item
        NodePtr m_parent = NULL;
    
        //! list of children
        std::vector<NodePtr> m_children;
    
        //! type (read only)
        std::string m_type = "Node";
    
    
};

}
}
// end namespace
//---------------------------------------------------------------------

#endif
