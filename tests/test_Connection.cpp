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
using namespace cr::world;
using namespace cr::signal;

//
// Test signal-slot connection
//
TEST(Connection, Step){
    
    // set up 2 links
    Link* myLink1 = new Link();
    Link* myLink2 = new Link();
    
    myLink1->setDegreeOfFreedom(cr::CR_EULER_FREE_POS_X);
    myLink1->setFreeVariable(0.71);
    
    myLink2->setDegreeOfFreedom(cr::CR_EULER_FREE_POS_X);
    myLink2->setFreeVariable(0);
    
    EXPECT_DOUBLE_EQ(0.71, myLink1->getFreeVariable());
    EXPECT_DOUBLE_EQ(0, myLink2->getFreeVariable());
    
    // create the slot
    Connection<double, Link, Link> mySlot(myLink1,
                                          &Link::getFreeVariable,
                                          myLink2,
                                          &Link::setFreeVariable);
    
    // now make sure the emitters and receivers are identical
    EXPECT_EQ(myLink1, mySlot.getEmitter());
    EXPECT_EQ(myLink2, mySlot.getReceiver());
    
    // perform the step
    mySlot.step();
    
    // and check the data value
    EXPECT_DOUBLE_EQ(0.71, mySlot.request());
    EXPECT_DOUBLE_EQ(0.71, myLink2->getFreeVariable() );
}




