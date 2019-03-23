/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#include <iostream>
#include <cr/math>
#include "gtest/gtest.h"


// Use the CoreRobotics namespace
using namespace cr;


//
// Test the unit conversion maths
//
TEST(Conversion, Units){
    
    // basic radian/degree conversions
    EXPECT_DOUBLE_EQ(180, math::Conversion::rad2deg(M_PI));
    EXPECT_DOUBLE_EQ(90,  math::Conversion::rad2deg(M_PI / 2));
    EXPECT_DOUBLE_EQ(M_PI, math::Conversion::deg2rad(180));
    EXPECT_DOUBLE_EQ(M_PI / 2,  math::Conversion::deg2rad(90));
}


//
// Test the angle wrapping maths
//
TEST(Conversion, Wrapping){
    
    // check wrapping
    EXPECT_NEAR(0, math::Conversion::wrapToPi(0), 1e-12);
    EXPECT_NEAR(0, math::Conversion::wrapToPi(2 * M_PI), 1e-12);
    EXPECT_NEAR(0, math::Conversion::wrapToPi(4 * M_PI), 1e-12);
    
    // slight offsets
    EXPECT_NEAR(0.1 - M_PI, math::Conversion::wrapToPi(M_PI + 0.1), 1e-12);
    EXPECT_NEAR(M_PI - 0.1, math::Conversion::wrapToPi(-(M_PI + 0.1)), 1e-12);
    EXPECT_NEAR(0.1, math::Conversion::wrapToPi(2 * M_PI + 0.1), 1e-12);
    
    // at pi boundary (always goes to negative)
    EXPECT_NEAR(-M_PI, math::Conversion::wrapToPi(M_PI), 1e-12);
    EXPECT_NEAR(-M_PI, math::Conversion::wrapToPi(-M_PI), 1e-12);
    EXPECT_NEAR(-M_PI, math::Conversion::wrapToPi(11 * M_PI), 1e-12);
    EXPECT_NEAR(-M_PI, math::Conversion::wrapToPi(-11 * M_PI), 1e-12);
}

