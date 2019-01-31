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
#include <cr/signal>
#include <cr/world>
#include <cr/physics>
#include <cr/core>
#include "gtest/gtest.h"


// Use the CoreRobotics namespace
using namespace cr::core;
using namespace cr::world;
using namespace cr::signal;
using namespace cr::physics;

//
// Test Slot connection
//
TEST(Log, Step){
    
    // set up 2 Frames
    std::shared_ptr<Frame> myFrame1 = std::shared_ptr<Frame>(new Frame);
    std::shared_ptr<Frame> myFrame2 = std::shared_ptr<Frame>(new Frame);
    
    Eigen::Vector3d p = {0.0, 0.1, 1.0};
    myFrame1->setTranslation(p);
    
    Eigen::Vector3d v = myFrame1->getTranslation();
    EXPECT_DOUBLE_EQ(v(0), p(0));
    EXPECT_DOUBLE_EQ(v(1), p(1));
    EXPECT_DOUBLE_EQ(v(2), p(2));
    v = myFrame2->getTranslation();
    EXPECT_DOUBLE_EQ(v(0), 0);
    EXPECT_DOUBLE_EQ(v(1), 0);
    EXPECT_DOUBLE_EQ(v(2), 0);
    
    // create the signal(s)
    std::shared_ptr<Signal<Eigen::Vector3d, Frame>> mySignal1 =
      Signal<Eigen::Vector3d, Frame>::create(myFrame1, &Frame::getTranslation);
    std::shared_ptr<Signal<Eigen::Vector3d, Frame>> mySignal2 =
      Signal<Eigen::Vector3d, Frame>::create(myFrame2, &Frame::getTranslation);
    mySignal1->setName("Signal 1");
    mySignal2->setName("Signal 2");
    
    // open a log and attach the signals
    LogPtr myLog = Log::create();
    myLog->add(mySignal1);
    myLog->add(mySignal2);
    
    // Start the log
    Clock c;
    c.startTimer();
    myLog->setName("my_log");
    myLog->onStart();
    myLog->step();
    c.sleep(0.001);
    myLog->step();
    myLog->step();
    c.sleep(0.001);
    myLog->step();
    myLog->step();
    myLog->onStop();
    
    // Now we should load the log file
    // and make sure it's what we expect
    std::ifstream t;
    t.open("my_log");
    std::string buffer;
    std::string line;
    std::string str = "time,Signal 1[0],Signal 1[1],Signal 1[2],Signal 2[0],Signal 2[1],Signal 2[2],";
    std::getline(t, line);
    EXPECT_EQ(str, line);
    std::cout << line << "\n";
    while(t){
        std::getline(t, line);
        std::cout << line << "\n";
    }
    t.close();
}



