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
//=====================================================================

#include <iostream>
#include <cr/core>
#include "gtest/gtest.h"


// Use the CoreRobotics namespace
using namespace cr;
using namespace cr::core;


// Callback for the thread
void myCallback1(void){
    const char* memoryName = "MyMemory1";
    // Open some shared memory as client
    SharedMemory mem(memoryName, CR_MANAGER_CLIENT);
    
    // create a vector of data
    Eigen::VectorXd v(1);
    v << 0;
    
    int i = 0;
    while(i<10){
        v(0) = double(i);
        mem.set("signal_3", v);
        i++;
    }
}


// Callback for the thread
void myCallback2(void* arg){
    const char* memoryName = "MyMemory2";
    // Open some shared memory as client
    SharedMemory mem(memoryName, CR_MANAGER_CLIENT);
    
    // create a vector of data
    Eigen::VectorXd v(1);
    v << 0;
    
    int i = 0;
    while(i<10){
        v(0) = double(i);
        mem.set("signal_4", v);
        i++;
    }
}



//
// Test thread start function
//
TEST(Thread, Start){
    const char* memoryName = "MyMemory1";
    SharedMemory server(memoryName, CR_MANAGER_SERVER);
    Eigen::VectorXd v(1);
    v << 1.0;
    server.addSignal("signal_3", v);
    
    // Create a thread
    Thread myThread;
    myThread.setCallback(*myCallback1);
    
    // wait for the thread to finish
    myThread.join();
    
    Eigen::VectorXd v2 = server.get("signal_3");
    EXPECT_EQ(1, v2.size());
    EXPECT_DOUBLE_EQ(9, v2(0));
}




//
// Test thread start function - with argumnet
//
TEST(Thread, StartArgument){
    const char* memoryName = "MyMemory2";
    SharedMemory server(memoryName, CR_MANAGER_SERVER);
    Eigen::VectorXd v(1);
    v << 1.0;
    server.addSignal("signal_4", v);
    
    // Create a thread
    void* arg;
    Thread myThread;
    myThread.setCallback(*myCallback2, arg);
    
    // wait for the thread to finish
    myThread.join();
    
    Eigen::VectorXd v2 = server.get("signal_4");
    EXPECT_EQ(1, v2.size());
    EXPECT_DOUBLE_EQ(9, v2(0));
}

