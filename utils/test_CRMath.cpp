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

#include <iostream>
#include "CoreRobotics.hpp"


Eigen::VectorXd linearSystem(double t, Eigen::VectorXd x, Eigen::VectorXd u);


// Use the CoreRobotics namespace
using namespace CoreRobotics;

void test_CRMath(void){
    
    std::cout << "**********************\n";
    std::cout << "Running the test_CRMath\n";
    
    
    // Define a state vector
    Eigen::VectorXd x(2);
    x << 0, 0; // IC
    
    // Define an input
    Eigen::VectorXd u(1);
    u << 1;  // IC
    
    // Time settings
    double t = 0;
    double dt = 0.1;
    
    // Forward euler
    std::cout << "\nForward Euler integrator:\n";
    printf("t = %3.1f, x = (%+6.4f, %+6.4f)\n",t,x(0),x(1));
    
    // loop for 2 seconds
    while (t < 2){
        x = CRMath::forwardEulerStep(*linearSystem, t, x, u, dt);
        printf("t = %3.1f, x = (%+6.4f, %+6.4f)\n",t,x(0),x(1));
        t = t+dt;
    }
    
    
    // Runga Kutta
    x << 0, 0;
    t = 0;
    std::cout << "\nRunga Kutta integrator:\n";
    printf("t = %3.1f, x = (%+6.4f, %+6.4f)\n",t,x(0),x(1));
    
    // loop for 2 seconds
    while (t < 2){
        x = CRMath::rungeKuttaStep(*linearSystem, t, x, u, dt);
        printf("t = %3.1f, x = (%+6.4f, %+6.4f)\n",t,x(0),x(1));
        t = t+dt;
    }


	// Test the wrap to pi functionality
	std::cout << "\nWrap to Pi Test:\n";
	printf("%+6.3f -> %+6.3f\n", CR_PI / 2, CRMath::wrapToPi(CR_PI / 2));
	printf("%+6.3f -> %+6.3f\n", 3 * CR_PI / 2, CRMath::wrapToPi(3 * CR_PI / 2));
	printf("%+6.3f -> %+6.3f\n", -CR_PI / 2, CRMath::wrapToPi(-CR_PI / 2));
	printf("%+6.3f -> %+6.3f\n", -3 * CR_PI / 2, CRMath::wrapToPi(-3 * CR_PI / 2));
	printf("%+6.3f -> %+6.3f\n", 2 * CR_PI, CRMath::wrapToPi(2 * CR_PI));
	printf("%+6.3f -> %+6.3f\n\n", 3 * CR_PI, CRMath::wrapToPi(3 * CR_PI));
}



// Callback for the math operation
Eigen::VectorXd linearSystem(double t, Eigen::VectorXd x, Eigen::VectorXd u){
    
    Eigen::Matrix2d A;
    Eigen::Vector2d B;
    
    A << 0.0, 1.0, -10.0, -6.0;
    B << 0.0, 1.0;
    
    return A*x + B*u;
}


