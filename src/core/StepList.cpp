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

#include "StepList.hpp"

//---------------------------------------------------------------------
// Begin namespace
namespace cr {
namespace core {
    
    
//---------------------------------------------------------------------
/*!
 The constructor sets up the StepList.\n
 */
//---------------------------------------------------------------------
StepList::StepList(){
    
}


//---------------------------------------------------------------------
/*!
 The destructor closes the StepList.\n
 */
//---------------------------------------------------------------------
StepList::~StepList(){
    
    m_vertices.clear();
}
    
    
//---------------------------------------------------------------------
/*!
 Create a new StepList.\n
 */
//---------------------------------------------------------------------
StepListPtr StepList::create(){
    return std::make_shared<StepList>();
}


//---------------------------------------------------------------------
/*!
 This function adds a step element to the list.\n
 */
//---------------------------------------------------------------------
void StepList::attach(StepPtr i_element)
{
    m_vertices.push_back(i_element);
}
    

//---------------------------------------------------------------------
/*!
 This function steps the list of elements.\n
 */
//---------------------------------------------------------------------
void StepList::step(){
    
    // step each of the vertices
    for (unsigned i = 0; i < m_vertices.size(); i++){
        m_vertices.at(i)->step();
    }
}


}
}
// end namespace
//---------------------------------------------------------------------


