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


void callback1(void);
void callback2(void);


// Use the CoreRobotics namespace
using namespace CoreRobotics;

int main(void){
    
    std::cout << "**********************\n";
    std::cout << "Demonstration of CRCore\n";
    
    double t;
    
    // Create a clock
    CRClock MyClock;
    
    
    MyClock.startTimer();
    MyClock.sleep(0.1);
    t = MyClock.getElapsedTime();
    
    std::cout << "t = " << t << std::endl;
    
    
    // Create a thread
    CRThread myThread1 = CRThread(CR_PRIORITY_HIGH);
    myThread1.setCallback(*callback1);
    
    // Create another thread
    CRThread myThread2;
    myThread2.setCallback(*callback2);
    
    
    // start the threads
    myThread1.start();
    myThread2.start();
    
}



// Callback for the first thread
void callback1(void){
    
    CRClock c;
    
    int i = 0;
    
    double dt = 0.1;
    double t  = 0.0;
    
    while(i<10){
        c.startTimer();
        i++;
        printf("Thread 1: i = %i \n",i);
        t = c.getElapsedTime();
        c.sleep(dt-t);
    }
}



// Callback for the second thread
void callback2(void){
    
    CRClock c;
    
    int i = 0;
    
    double dt = 0.25;
    double t  = 0.0;
    
    while(i<4){
        c.startTimer();
        i++;
        printf("Thread 2: i = %i \n",i);
        t = c.getElapsedTime();
        c.sleep(dt-t);
    }
}
