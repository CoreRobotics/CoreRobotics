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

#ifndef TrajectoryGenerator_hpp
#define TrajectoryGenerator_hpp

//=====================================================================
// Includes
#include "Eigen/Dense"
#include "Types.hpp"
#include "Clock.hpp"

//=====================================================================
// CoreRobotics namespace
namespace cr {
    
//=====================================================================
/*!
 \file TrajectoryGenerator.hpp
 \brief Implements a class to generate minimum jerk trajectories.
 */
//---------------------------------------------------------------------
/*!
 \class TrajectoryGenerator
 \ingroup controllers
 
 \brief This class provides methods for generating minimum jerk
 trajectories from initial and final conditions.
 
 \details
 ## Description
 TrajectoryGenerator implements the minimum-jerk trajectory generation
 technique from a set of initial condititions, final conditions, and their
 1st and 2nd derivatives.
 
 
 These methods are available with the trajectory generator:
 - TrajectoryGenerator::solve computes the minimum jerk trajectory for
 the specified initial and final conditions and stores the representation
 of the trajectory internally.
 - TrajectoryGenerator::step computes the values of the trajectory for
 a specified time (if specified) or for the time elapsed since the solve
 method was called (if time is not specified).
 
 ## Example
 This example demonstrates use of the TrajectoryGenerator class.
 \include example_TrajectoryGenerator.cpp
 
 ## References
 [1] N. Hogan, "Adaptive control of mechanical impedance by coactivation of
     antagonist muscles," IEEE Trans. on Automatic Control AC-29: 681-690,
     1984. \n\n
 
 */
//=====================================================================
//! Structure defining a waypoint (i.e. the output)
struct CRWaypoint {
    double time;
    Eigen::VectorXd position;
    Eigen::VectorXd velocity;
    Eigen::VectorXd acceleration;
    Eigen::VectorXd jerk;
};

//=====================================================================
class TrajectoryGenerator {
    
//---------------------------------------------------------------------
// Constructor and Destructor
public:
    
    //! Class constructor
    TrajectoryGenerator();
    
    
//---------------------------------------------------------------------
// Public Methods
public:
    
    //! Solve for the coefficients needed to achieve the trajectory
    core::Result solve(Eigen::VectorXd i_x0,
                       Eigen::VectorXd i_v0,
                       Eigen::VectorXd m_a0,
                       Eigen::VectorXd m_xf,
                       Eigen::VectorXd m_vf,
                       Eigen::VectorXd m_af,
                       double i_tf);
    
    //! Solve for the coefficients needed to achieve the trajectory
    core::Result solve(CRWaypoint i_wp0,
                       CRWaypoint i_wpf);
    
    //! Get the trajectory at time t
    CRWaypoint step(double i_t);
    
    //! Step the next trajectory reference
    CRWaypoint step(void);
    
//---------------------------------------------------------------------
// Protected Members
protected:
    
    //! Final time
    double m_tf = 1.0;
    
    //! Polynomial coefficient matrix
    Eigen::Matrix<double, 6, Eigen::Dynamic> m_X;
    
    //! An internal clock for keeping track of time
    core::Clock m_timer;
    
};

//=====================================================================
// End namespace
}


#endif
