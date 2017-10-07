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
\version 0.0

*/
//=====================================================================
#include <iostream>
#include "CoreRobotics.hpp"

// Use the CoreRobotics namespace
using namespace CoreRobotics;


// -------------------------------------------------------------
void test_CRTrajectoryGenerator(void) {

	std::cout << "*************************************\n";
	std::cout << "Demonstration of CRTrajectoryGenerator.\n";
    std::cout << std::fixed; std::cout.precision(4);
    
    // ------------------------------------------
    // Define a trajectory generator
    CRTrajectoryGenerator trajGen;
    
    
    // ------------------------------------------
    // Initialize a clock to time the solver
    CRClock timer = CRClock();
    

	// ------------------------------------------
	// Initial and final conditions
    double tf = 1.2;
    Eigen::Vector2d x0;
    x0 << 0, 0;
    Eigen::Vector2d v0;
    v0 << 0, -2;
    Eigen::Vector2d a0;
    a0 << 0, 0;
    Eigen::Vector2d xf;
    xf << -0.4, 0.5;
    Eigen::Vector2d vf;
    vf << 1.2, 1;
    Eigen::Vector2d af;
    af << 0, 0;
    
    // ------------------------------------------
    // Compute the trajectory
    trajGen.solve(x0, v0, a0, xf, vf, af, tf);
    timer.startTimer();
    // ------------------------------------------
    // Solve for several times
    Eigen::VectorXd x;
    Eigen::VectorXd v;
    Eigen::VectorXd a;
    Eigen::VectorXd j;
    
    // loop
    printf("t (s) | Position | Velocity | Acceleration\n");
    double t = 0;
    while(t <= 1.2) {
        
        trajGen.step(t, x, v, a, j);
        
        // output the time and state
        printf("%.1f | %+.3f, %+.3f | %+.3f, %+.3f | %+.3f, %+.3f \n", t,
              x(0), x(1), v(0), v(1), a(0), a(1));
        
        // timer.sleep(0.1);
        
        t += 0.1;
    }
    

}
// -------------------------------------------------------------
