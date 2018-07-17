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

#ifndef CR_DIRECTED_GRAPH_HPP_
#define CR_DIRECTED_GRAPH_HPP_

#include "Loop.hpp"
#include "Step.hpp"
#include <vector>

//---------------------------------------------------------------------
// Begin namespace
namespace cr {

    
//! Manipulator shared pointer
class DirectedGraph;
typedef std::shared_ptr<DirectedGraph> DirectedGraphPtr;
    
    
//---------------------------------------------------------------------
/*!
 \class DirectedGraph
 \ingroup core
 
 \brief
 This class implements a directed graph thread loop.
 
 \details
 ## Description
 This class implements a directed graph thread loop.  One step
 of the entire graph is performed for each loop iteration.  The step
 classes are executed in the order in which they have been added to the
 list of Steps();
 
 - Loop::start starts the thread loop.
 - Loop::pause pauses the loop execution, but does not exit the
 thread of execution.
 - Loop::stop exits the thread of execution.
 - Loop::stop stops the thread execution.
 */
//---------------------------------------------------------------------
class DirectedGraph : public Loop {
    
    // Constructor and Destructor
    public:
    
        //! Class constructor
        DirectedGraph();
        DirectedGraph(double i_updateRate);
    
        //! Class destructor
        virtual ~DirectedGraph();
    
        //! Create
        static DirectedGraphPtr create();
        static DirectedGraphPtr create(double i_updateRate);
    
    
    // Step member control
    public:
    
        //! Add a step item to the list of vertices
        void add(Step* i_vertex);
    
    
    // Derived step function
    public:
    
        //! step the graph
        void step();
    

    // Private members
    private:
    
        //! list of step vertices
        std::vector<Step*> m_vertices;
    
};
    
    
//! Smart pointer to DirectedGraph
typedef std::shared_ptr<DirectedGraph> DirectedGraphPtr;
    

}
// end namespace
//---------------------------------------------------------------------

#endif
