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

#ifndef CR_LOOP_ELEMENT_HPP_
#define CR_LOOP_ELEMENT_HPP_

#include <vector>
#include "Loop.hpp"

//---------------------------------------------------------------------
// Begin namespace
namespace cr {

    
//---------------------------------------------------------------------
/*!
 \class LoopElement
 \ingroup core
 
 \brief This abstract class defines the methods needed to derive a
 root call element for a Loop.
 
 \details
 ## Description
 This abstract class defines the methods needed to derive a
 call for the root element of a LoopElement.  The primary methods
 to be implemented are:
 
 - LoopElement::step() is called on each iteration of the Loop
 while the thread is running.
 */
//---------------------------------------------------------------------
class LoopElement {
    
    // Constructor and destructor
    public:
    
        //! constructor
		LoopElement() {}
    
        //! destructor
        ~LoopElement() {
            delete m_caller;
        }
    
    
    // Primary LoopRoot functions that must be implemented
    public:
    
        //! Step the element - called on each iteration of the thread while running
        virtual void step() = 0;
    
        //! Reset the element - called on ThreadLoop::start())
        virtual void reset() = 0;
    
    
    // LoopElement graph methods
    public:
    
        //! set the pointer to the LoopElement parent caller
        void setCaller(Loop* i_caller) { m_caller = i_caller; }
    
        //! return the pointer to the parent caller (NULL value indicates no caller)
        Loop* getCaller() { return m_caller; }
    
    
    // Protected members
    protected:
    
        //! thread caller
        Loop* m_caller = NULL;
};
    
}
// end namespace
//---------------------------------------------------------------------

#endif
