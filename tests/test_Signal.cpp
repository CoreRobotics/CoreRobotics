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
#include <cr/physics>
#include <cr/core>
#include "gtest/gtest.h"


// Use the CoreRobotics namespace
using namespace cr::physics;
using namespace cr::core;
using namespace cr::signal;


//
// Test signal connection
//
TEST(Signal, Request){
    
    // set up a Frame
    std::shared_ptr<FrameEuler> myClass = std::make_shared<FrameEuler>();
    myClass->setFreeVariable(CR_EULER_FREE_POS_X);
    myClass->setFreeValue(0.71);
    
    // create the signal
    std::shared_ptr<Signal<double, FrameEuler>> mySignalFreeValue =
        Signal<double, FrameEuler>::create(myClass, &FrameEuler::getFreeValue);
    
    // check values
    EXPECT_EQ(myClass, mySignalFreeValue->getEmitter());
    EXPECT_DOUBLE_EQ(0.71, mySignalFreeValue->request());
    
    // let's create another signal of different type
    std::shared_ptr<Signal<Eigen::Vector3d, FrameEuler>> mySignalPose =
        Signal<Eigen::Vector3d, FrameEuler>::create(myClass, &FrameEuler::getTranslation);
    
    // check values
    Eigen::Vector3d v = mySignalPose->request();
    EXPECT_EQ(myClass, mySignalPose->getEmitter());
    EXPECT_DOUBLE_EQ(0.71, v(0));
    EXPECT_DOUBLE_EQ(0.0, v(1));
    EXPECT_DOUBLE_EQ(0.0, v(2));
}



