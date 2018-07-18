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

#ifndef CR_STEP_HPP_
#define CR_STEP_HPP_

#include <memory>

//---------------------------------------------------------------------
// Begin namespace
namespace cr {

    
//---------------------------------------------------------------------
/*!
 \class Step
 \ingroup core
 
 \brief This abstract class defines a ::step() method function call
 that is used by loop functions.
 
 \details
 ## Description
 This abstract class defines the base ::step() method needed to derive
 a call for a Loop class:
 
 - Step::step() is called on each iteration of the Loop while the
 thread is running.
 */
//---------------------------------------------------------------------
class Step
    : public std::enable_shared_from_this<Step>
{
    
    // Constructor and destructor
    public:
    
        //! constructor
		Step() {}
    
        //! destructor
        ~Step() {}
    
    
    // Functions to be implemented
    public:
    
        //! The step function must be implemented in derived classes
        virtual void step() = 0;
};
    
    
    
//! Step shared pointer
typedef std::shared_ptr<Step> StepPtr;
    
    
}
// end namespace
//---------------------------------------------------------------------

#endif
