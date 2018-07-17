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
//=====================================================================

#include "DirectedGraph.hpp"

//=====================================================================
// CoreRobotics namespace
namespace cr {

    
    
//---------------------------------------------------------------------
/*!
 The constructor sets up the thread loop without a specific update 
 rate.  The loop will attempt to execute each step as quickly as 
 possible.\n
 */
//---------------------------------------------------------------------
DirectedGraph::DirectedGraph(){
    
}


//---------------------------------------------------------------------
/*!
The constructor sets up the thread loop with a specific update 
 rate.  The loop will attempt to execute each step at the rate
 specified by i_updateRate.\n
 
\param[in]	   i_updateRate	 this sets a constant update rate (s)
*/
//---------------------------------------------------------------------
DirectedGraph::DirectedGraph(double i_updateRate)
    : cr::Loop(i_updateRate)
{
    
}


//---------------------------------------------------------------------
/*!
 The destructor closes the thread loop.\n
 */
//---------------------------------------------------------------------
DirectedGraph::~DirectedGraph(){
    
    m_vertices.clear();
}
    
    
//---------------------------------------------------------------------
/*!
 Create a new directed graph.\n
 
 \param[in]       i_updateRate     this sets a constant update rate (s)
 */
//---------------------------------------------------------------------
DirectedGraphPtr DirectedGraph::create(){
    return std::make_shared<DirectedGraph>();
}
DirectedGraphPtr DirectedGraph::create(double i_updateRate){
    return std::make_shared<DirectedGraph>(i_updateRate);
}


//---------------------------------------------------------------------
/*!
 This function adds a step pointer to the list of vertices.\n
 */
//---------------------------------------------------------------------
void DirectedGraph::add(Step* i_vertex)
{
    m_vertices.push_back(i_vertex);
}
    

//---------------------------------------------------------------------
/*!
 This function executes the .\n
 */
//---------------------------------------------------------------------
void DirectedGraph::step(){
    
    // step each of the vertices
    for (int i = 0; i < m_vertices.size(); i++){
        m_vertices.at(i)->step();
    }
}


//=====================================================================
// End namespace
}


