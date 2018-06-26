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
#include "gtest/gtest.h"


// Use the CoreRobotics namespace
using namespace CoreRobotics;


//
// Test memory server set/get function
//
TEST(CRSharedMemory, Server){
    const char* memoryName = "MyMemory";
    CRSharedMemory server(memoryName, CR_MANAGER_SERVER);
    Eigen::VectorXd v(2);
    v << 0.0, 0.8;
    server.addSignal("signal_1", v);
    Eigen::VectorXd v2 = server.get("signal_1");
    EXPECT_EQ(2, v2.size());
    EXPECT_DOUBLE_EQ(0, v2(0));
    EXPECT_DOUBLE_EQ(0.8, v2(1));
    
    v << 1.0, 1.2;
    server.set("signal_1", v);
    v2 = server.get("signal_1");
    EXPECT_EQ(2, v2.size());
    EXPECT_DOUBLE_EQ(1.0, v2(0));
    EXPECT_DOUBLE_EQ(1.2, v2(1));
    server.removeSignal("signal_1");
}

//
// Test memory client set/get function
//
TEST(CRSharedMemory, Client){
    const char* memoryName = "MyMemory";
    CRSharedMemory server(memoryName, CR_MANAGER_SERVER);
    Eigen::VectorXd v(1);
    v << 1.4;
    server.addSignal("signal_2", v);
    
    CRSharedMemory client(memoryName, CR_MANAGER_CLIENT);
    Eigen::VectorXd v2 = client.get("signal_2");
    EXPECT_EQ(1, v2.size());
    EXPECT_DOUBLE_EQ(1.4, v2(0));
    
    v << 2.4;
    client.set("signal_2", v);
    v2 = server.get("signal_2");
    EXPECT_EQ(1, v2.size());
    EXPECT_DOUBLE_EQ(2.4, v2(0));
    
    server.removeSignal("signal_2");
}

