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
using namespace cr;


//---------------------------------------------------------------------
/*!
 Test the frame transform methods
 */
//---------------------------------------------------------------------
TEST(WorldItem, transforms){
    
    // create a world
    WorldPtr myWorld = World::create();
    
    // create 3 world items
    WorldItemPtr item1 = WorldItem::create();
    WorldItemPtr item2 = WorldItem::create();
    WorldItemPtr item3 = WorldItem::create();
    
    // define 3 unique frame transforms
    Frame f1, f2, f3;
    Eigen::Matrix3d R;
    R.setIdentity();
    Eigen::Vector3d T;
    T << 1, 0, 0;
    f1.setRotationAndTranslation(R, T);
    T << 0, 2, 0;
    f2.setRotationAndTranslation(R, T);
    T << 0, 0, 3;
    f3.setRotationAndTranslation(R, T);
    
    // assign frame transforms to each
    item1->setLocalTransform(f1);
    item2->setLocalTransform(f2);
    item3->setLocalTransform(f3);
    
    // assign a simple scene graph
    // !!! THIS IS WHERE THE SEG FAULT HAPPENS
    item1->addChild(item2);
    myWorld->addChild(item1);
    myWorld->addChild(item3);
    
    // return the frame transformation locations
    Eigen::Vector3d p;
    
    // local transform of item1
    p = item1->getLocalTransform().getPosition();
    EXPECT_DOUBLE_EQ(p(0), 1);
    EXPECT_DOUBLE_EQ(p(1), 0);
    EXPECT_DOUBLE_EQ(p(2), 0);
    
    // local transform of item2
    p = item2->getLocalTransform().getPosition();
    EXPECT_DOUBLE_EQ(p(0), 0);
    EXPECT_DOUBLE_EQ(p(1), 2);
    EXPECT_DOUBLE_EQ(p(2), 0);
    
    // local transform of item3
    p = item3->getLocalTransform().getPosition();
    EXPECT_DOUBLE_EQ(p(0), 0);
    EXPECT_DOUBLE_EQ(p(1), 0);
    EXPECT_DOUBLE_EQ(p(2), 3);
    
    
    // global transform of item1
    p = item1->getGlobalTransform().getPosition();
    EXPECT_DOUBLE_EQ(p(0), 1);
    EXPECT_DOUBLE_EQ(p(1), 0);
    EXPECT_DOUBLE_EQ(p(2), 0);
    
    // global transform of item2
    p = item2->getGlobalTransform().getPosition();
    EXPECT_DOUBLE_EQ(p(0), 1);
    EXPECT_DOUBLE_EQ(p(1), 2);
    EXPECT_DOUBLE_EQ(p(2), 0);
    
    // global transform of item3
    p = item3->getGlobalTransform().getPosition();
    EXPECT_DOUBLE_EQ(p(0), 0);
    EXPECT_DOUBLE_EQ(p(1), 0);
    EXPECT_DOUBLE_EQ(p(2), 3);
    
    
    // transform of item2 relative to item3
    p = item2->getRelativeTransform(item3).getPosition();
    EXPECT_DOUBLE_EQ(p(0), 1);
    EXPECT_DOUBLE_EQ(p(1), 2);
    EXPECT_DOUBLE_EQ(p(2), -3);
}
     





