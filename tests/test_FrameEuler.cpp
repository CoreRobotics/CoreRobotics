/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#include <iostream>
#include <cr/physics>
#include "gtest/gtest.h"


// Use the CoreRobotics namespace
using namespace cr::physics;


//
// constructor
//
TEST(FrameEuler, Construct){
    FrameEuler frame;
    Eigen::VectorXd p = frame.getPose(CR_EULER_MODE_XYZ);
    EXPECT_DOUBLE_EQ(0, p(0));
    EXPECT_DOUBLE_EQ(0, p(1));
    EXPECT_DOUBLE_EQ(0, p(2));
    EXPECT_DOUBLE_EQ(0, p(3));
    EXPECT_DOUBLE_EQ(0, p(4));
    EXPECT_DOUBLE_EQ(0, p(5));
}


//
// SetFreeValue/GetFreeValue
//
TEST(FrameEuler, GetFreeValue){
    FrameEuler frame(0, 0, 0, M_PI / 2, -M_PI / 2, M_PI / 4, CR_EULER_MODE_XYZ, CR_EULER_FREE_NONE);
    EXPECT_EQ(NULL, frame.getFreeValue());
    
    frame.setFreeVariable(CR_EULER_FREE_ANG_A);
    EXPECT_DOUBLE_EQ(M_PI / 2, frame.getFreeValue());
    
    frame.setFreeValue(0);
    EXPECT_DOUBLE_EQ(0, frame.getFreeValue());
}


//
// SetFreeVariable/GetFreeVariable
//
TEST(FrameEuler, GetFreeVariable){
    FrameEuler frame(0, 0, 0, M_PI / 2, -M_PI / 2, M_PI / 4, CR_EULER_MODE_XYZ, CR_EULER_FREE_NONE);
    EXPECT_EQ(CR_EULER_FREE_NONE, frame.getFreeVariable());
    EXPECT_FALSE(frame.isDriven());
    
    frame.setFreeVariable(CR_EULER_FREE_ANG_A);
    EXPECT_EQ(CR_EULER_FREE_ANG_A, frame.getFreeVariable());
    EXPECT_TRUE(frame.isDriven());
}


//
// SetMode/GetMode
//
TEST(FrameEuler, GetMode){
    FrameEuler frame(0, 0, 0, M_PI / 2, -M_PI / 2, M_PI / 4, CR_EULER_MODE_XYZ, CR_EULER_FREE_NONE);
    EXPECT_EQ(CR_EULER_MODE_XYZ, frame.getMode());
    
    frame.setMode(CR_EULER_MODE_ZYX);
    EXPECT_EQ(CR_EULER_MODE_ZYX, frame.getMode());
}


//
// SetPositionAndOrientation/GetPositionAndOrientation
//
TEST(FrameEuler, GetPositionAndOrientation){
    FrameEuler frame(0, 0, 0, M_PI / 2, -M_PI / 2, M_PI / 4, CR_EULER_MODE_XYZ, CR_EULER_FREE_NONE);
    double x, y, z, a, b, g;
    frame.getPosition(x, y, z);
    EXPECT_DOUBLE_EQ(0, x);
    EXPECT_DOUBLE_EQ(0, y);
    EXPECT_DOUBLE_EQ(0, z);
    
    frame.setPosition(0.1, 0.2, 0.3);
    frame.getPosition(x, y, z);
    EXPECT_DOUBLE_EQ(0.1, x);
    EXPECT_DOUBLE_EQ(0.2, y);
    EXPECT_DOUBLE_EQ(0.3, z);
    
    frame.getOrientation(a, b, g);
    EXPECT_DOUBLE_EQ( M_PI / 2, a);
    EXPECT_DOUBLE_EQ(-M_PI / 2, b);
    EXPECT_DOUBLE_EQ( M_PI / 4, g);
    
    frame.setOrientation(0.1, 0.2, 0.3);
    frame.getOrientation(a, b, g);
    EXPECT_DOUBLE_EQ(0.1, a);
    EXPECT_DOUBLE_EQ(0.2, b);
    EXPECT_DOUBLE_EQ(0.3, g);
    
    frame.setPositionAndOrientation(0, 0, 0, M_PI / 2, -M_PI / 2, M_PI / 4);
    frame.getPositionAndOrientation(x, y, z, a, b, g);
    EXPECT_DOUBLE_EQ(0, x);
    EXPECT_DOUBLE_EQ(0, y);
    EXPECT_DOUBLE_EQ(0, z);
    EXPECT_DOUBLE_EQ( M_PI / 2, a);
    EXPECT_DOUBLE_EQ(-M_PI / 2, b);
    EXPECT_DOUBLE_EQ( M_PI / 4, g);
}
