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
// Test the unit conversion maths
//
TEST(CRConversion, Units){
    
    // basic radian/degree conversions
    EXPECT_DOUBLE_EQ(180, CRConversion::rad2deg(M_PI));
    EXPECT_DOUBLE_EQ(90,  CRConversion::rad2deg(M_PI / 2));
    EXPECT_DOUBLE_EQ(M_PI, CRConversion::deg2rad(180));
    EXPECT_DOUBLE_EQ(M_PI / 2,  CRConversion::deg2rad(90));
}


//
// Test the angle wrapping maths
//
TEST(CRConversion, Wrapping){
    
    // check wrapping
    EXPECT_NEAR(0, CRConversion::wrapToPi(0), 1e-12);
    EXPECT_NEAR(0, CRConversion::wrapToPi(2 * M_PI), 1e-12);
    EXPECT_NEAR(0, CRConversion::wrapToPi(4 * M_PI), 1e-12);
    
    // slight offsets
    EXPECT_NEAR(0.1 - M_PI, CRConversion::wrapToPi(M_PI + 0.1), 1e-12);
    EXPECT_NEAR(M_PI - 0.1, CRConversion::wrapToPi(-(M_PI + 0.1)), 1e-12);
    EXPECT_NEAR(0.1, CRConversion::wrapToPi(2 * M_PI + 0.1), 1e-12);
    
    // at pi boundary (always goes to negative)
    EXPECT_NEAR(-M_PI, CRConversion::wrapToPi(M_PI), 1e-12);
    EXPECT_NEAR(-M_PI, CRConversion::wrapToPi(-M_PI), 1e-12);
    EXPECT_NEAR(-M_PI, CRConversion::wrapToPi(11 * M_PI), 1e-12);
    EXPECT_NEAR(-M_PI, CRConversion::wrapToPi(-11 * M_PI), 1e-12);
}

