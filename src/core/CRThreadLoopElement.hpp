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

#ifndef CRThreadLoopElement_hpp
#define CRThreadLoopElement_hpp

#include <vector>


//---------------------------------------------------------------------
// Begin namespace
namespace CoreRobotics {
    
//---------------------------------------------------------------------
/*!
 \class CRThreadLoopElement
 \ingroup core
 
 \brief This abstract class defines the methods needed to derive a
 call for a CRThreadLoop.
 
 \details
 ## Description
 This abstract class defines the methods needed to derive a
 call for a CRThreadLoop.  The primary methods to be implemented are:
 
 - CRThreadLoopElement::step() is called on each iteration of the thread
 while the thread is running.
 - CRThreadLoopElement::setPriority sets the priority of the thread.
 */
//---------------------------------------------------------------------
class CRThreadLoop;
class CRThreadLoopElement {
    
    // Constructor and destructor
    public:
    
        //! constructor
        CRThreadLoopElement() {}
    
        //! destructor
        ~CRThreadLoopElement() {
            delete m_caller;
            // delete m_parent;
        }
    
    
    // Primary CRThreadLoopElement functions that must be implemented
    public:
    
        //! Step the element - called on each iteration of the thread while running
        virtual void step() = 0;
    
        //! Reset the element - called on ThreadLoop::start())
        virtual void reset() = 0;
    
    
    // ther methods called by the caller thread
    public:
    
        //! set the pointer to the caller thread - called by the thread on construction
        void setCaller(CRThreadLoop* i_caller) {m_caller = i_caller;}
    
        //! return the pointer to the caller (NULL value indicates no caller)
        CRThreadLoop* getCaller() {return m_caller; }
    
        //! set the pointer to a parent
        // void setParent(CRThreadLoopElement* i_parent) {m_parent = i_parent;}
    
        //! return the pointer to a parent
        // CRThreadLoopElement* getParent() {return m_parent; }
    
        //! add a child to the list of children
        // void addChild(CRThreadLoopElement* i_child) { m_children.push_back(i_child); }
    
    
    // Protected members
    protected:
    
        //! thread caller
        CRThreadLoop* m_caller = NULL;
    
        //! parent
        // CRThreadLoopElement* m_parent = NULL;
            
        //! list of children
        // std::vector<CRThreadLoopElement*> m_children;
};
    
}
// end namespace
//---------------------------------------------------------------------

#endif
