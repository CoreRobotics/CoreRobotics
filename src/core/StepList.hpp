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

#ifndef CR_STEP_LIST_HPP_
#define CR_STEP_LIST_HPP_

#include "Loop.hpp"
#include "Step.hpp"
#include <vector>

//---------------------------------------------------------------------
// Begin namespace
namespace cr {
namespace core {

    
//! Manipulator shared pointer
class StepList;
typedef std::shared_ptr<StepList> StepListPtr;
    
    
//---------------------------------------------------------------------
/*!
 \class StepList
 \ingroup core
 
 \brief
 This class implements a list of individual step() classes (i.e. a
 a directed graph when signals are included)
 
 \details
 This class implements a step list.  Individual Step() classes are
 executed in the order in which they have been added to the list.
 */
//---------------------------------------------------------------------
class StepList : public Step {
    
    // Constructor and Destructor
    public:
    
        //! Class constructor
        StepList();
    
        //! Class destructor
        virtual ~StepList();
    
        //! Create
        static StepListPtr create();
    
    
    // Step member control
    public:
    
        //! Attach a step item to the list of vertices
        void attach(StepPtr i_vertex);
    
    
    // Derived step function
    public:
    
        //! step the graph
        void step();
    

    // Private members
    private:
    
        //! list of step elements
        std::vector<StepPtr> m_vertices;
    
};
    

}
}
// end namespace
//---------------------------------------------------------------------

#endif
