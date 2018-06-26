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

// Use the CoreRobotics namespace
using namespace CoreRobotics;


// -------------------------------------------------------------
// Declare a continuous motion model - xdot = fcn(t,x,u)
Eigen::VectorXd detDynFcn(double t, Eigen::VectorXd x, Eigen::VectorXd u){
    return -x + u;  // motion
}


// -------------------------------------------------------------
int main(void){
    
    std::cout << "*************************************\n";
    std::cout << "Demonstration of CRMotionModel.\n";
    
    
    // initialize a state vector
    Eigen::VectorXd x(1);
    x << 10;
    
    
    // initialize a deterministic sensor model
    CRMotionModel model = CRMotionModel(*detDynFcn,CRBX_MOTION_CONTINUOUS,x,0.2);
    
    
    // initialize an input and set it to zero
    Eigen::VectorXd u(1);
    u << 0;
    
    // Initialize a time t
    double t = 0;
    
    // loop
    printf("Time (s) | State\n");
    while(t <= 5) {
        
        // output the time and state
        printf("%5.1f    | %5.1f | %5.2f\n",t,u(0),x(0));
        
        // step at t = 2.5
        if (t >= 2.5){
            u << 10;
        }
        
        // get next state & time
        x = model.motion(u);
        t = model.getTime();
    }
    printf("%5.1f    | %5.1f | %5.2f\n",t,u(0),x(0));
    
}
// -------------------------------------------------------------
