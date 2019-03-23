/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

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



