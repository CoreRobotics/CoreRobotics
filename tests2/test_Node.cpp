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
using namespace cr::core;
using namespace cr::world;


//---------------------------------------------------------------------
/*!
 Test the frame transform methods
 */
//---------------------------------------------------------------------
TEST(Node, transforms){
    
    // create a world
    OriginPtr myOrigin = Origin::create();
    
    // create 3 world items
    NodePtr item1 = Node::create();
    NodePtr item2 = Node::create();
    NodePtr item3 = Node::create();
    
    // assign a simple scene graph
    myOrigin->addChild(item1);
    item1->addChild(item2);
    item1->addChild(item3);
    
    // define 3 unique frame transforms
    Frame f1, f2, f3;
    Eigen::Matrix3d R;
    Eigen::Vector3d T;
    R.setIdentity();
    T.setZero();
    
    // We're going to start by setting the first item at (1, 2, 3)
    T << 1, 2, 3;
    f1.setRotationAndTranslation(R, T);
    item1->setLocalTransform(f1);
    
    // The next item we're going to set +1 in the x direction
    T << 1, 0, 0;
    f2.setRotationAndTranslation(R, T);
    item2->setLocalTransform(f2);
    
    // The last item we're going to set +1 in the y direction
    T << 0, 1, 0;
    f3.setRotationAndTranslation(R, T);
    item3->setLocalTransform(f3);
    
    // Now let's test the make sure the local transformations were written
    // return the frame transformation locations
    Eigen::Vector3d p;
    Eigen::Matrix3d r;
    
    // local transform of item1
    p = item1->getLocalTransform().getTranslation();
    r = item1->getLocalTransform().getRotation();
    EXPECT_DOUBLE_EQ(p(0), 1);
    EXPECT_DOUBLE_EQ(p(1), 2);
    EXPECT_DOUBLE_EQ(p(2), 3);
    EXPECT_DOUBLE_EQ(r(0,0), 1);
    EXPECT_DOUBLE_EQ(r(0,1), 0);
    EXPECT_DOUBLE_EQ(r(0,2), 0);
    EXPECT_DOUBLE_EQ(r(1,0), 0);
    EXPECT_DOUBLE_EQ(r(1,1), 1);
    EXPECT_DOUBLE_EQ(r(1,2), 0);
    EXPECT_DOUBLE_EQ(r(2,0), 0);
    EXPECT_DOUBLE_EQ(r(2,1), 0);
    EXPECT_DOUBLE_EQ(r(2,2), 1);
    
    // local transform of item2
    p = item2->getLocalTransform().getTranslation();
    r = item2->getLocalTransform().getRotation();
    EXPECT_DOUBLE_EQ(p(0), 1);
    EXPECT_DOUBLE_EQ(p(1), 0);
    EXPECT_DOUBLE_EQ(p(2), 0);
    EXPECT_DOUBLE_EQ(r(0,0), 1);
    EXPECT_DOUBLE_EQ(r(0,1), 0);
    EXPECT_DOUBLE_EQ(r(0,2), 0);
    EXPECT_DOUBLE_EQ(r(1,0), 0);
    EXPECT_DOUBLE_EQ(r(1,1), 1);
    EXPECT_DOUBLE_EQ(r(1,2), 0);
    EXPECT_DOUBLE_EQ(r(2,0), 0);
    EXPECT_DOUBLE_EQ(r(2,1), 0);
    EXPECT_DOUBLE_EQ(r(2,2), 1);
    
    // local transform of item3
    p = item3->getLocalTransform().getTranslation();
    r = item3->getLocalTransform().getRotation();
    EXPECT_DOUBLE_EQ(p(0), 0);
    EXPECT_DOUBLE_EQ(p(1), 1);
    EXPECT_DOUBLE_EQ(p(2), 0);
    EXPECT_DOUBLE_EQ(r(0,0), 1);
    EXPECT_DOUBLE_EQ(r(0,1), 0);
    EXPECT_DOUBLE_EQ(r(0,2), 0);
    EXPECT_DOUBLE_EQ(r(1,0), 0);
    EXPECT_DOUBLE_EQ(r(1,1), 1);
    EXPECT_DOUBLE_EQ(r(1,2), 0);
    EXPECT_DOUBLE_EQ(r(2,0), 0);
    EXPECT_DOUBLE_EQ(r(2,1), 0);
    EXPECT_DOUBLE_EQ(r(2,2), 1);
    
    
    // Now let's test to make sure the global transformations work
    
    // global transform of item1
    p = item1->getGlobalTransform().getTranslation();
    r = item1->getGlobalTransform().getRotation();
    EXPECT_DOUBLE_EQ(p(0), 1);
    EXPECT_DOUBLE_EQ(p(1), 2);
    EXPECT_DOUBLE_EQ(p(2), 3);
    EXPECT_DOUBLE_EQ(r(0,0), 1);
    EXPECT_DOUBLE_EQ(r(0,1), 0);
    EXPECT_DOUBLE_EQ(r(0,2), 0);
    EXPECT_DOUBLE_EQ(r(1,0), 0);
    EXPECT_DOUBLE_EQ(r(1,1), 1);
    EXPECT_DOUBLE_EQ(r(1,2), 0);
    EXPECT_DOUBLE_EQ(r(2,0), 0);
    EXPECT_DOUBLE_EQ(r(2,1), 0);
    EXPECT_DOUBLE_EQ(r(2,2), 1);
    
    // global transform of item2
    p = item2->getGlobalTransform().getTranslation();
    r = item2->getGlobalTransform().getRotation();
    EXPECT_DOUBLE_EQ(p(0), 2);
    EXPECT_DOUBLE_EQ(p(1), 2);
    EXPECT_DOUBLE_EQ(p(2), 3);
    EXPECT_DOUBLE_EQ(r(0,0), 1);
    EXPECT_DOUBLE_EQ(r(0,1), 0);
    EXPECT_DOUBLE_EQ(r(0,2), 0);
    EXPECT_DOUBLE_EQ(r(1,0), 0);
    EXPECT_DOUBLE_EQ(r(1,1), 1);
    EXPECT_DOUBLE_EQ(r(1,2), 0);
    EXPECT_DOUBLE_EQ(r(2,0), 0);
    EXPECT_DOUBLE_EQ(r(2,1), 0);
    EXPECT_DOUBLE_EQ(r(2,2), 1);
    
    // global transform of item3
    p = item3->getGlobalTransform().getTranslation();
    r = item3->getGlobalTransform().getRotation();
    EXPECT_DOUBLE_EQ(p(0), 1);
    EXPECT_DOUBLE_EQ(p(1), 3);
    EXPECT_DOUBLE_EQ(p(2), 3);
    EXPECT_DOUBLE_EQ(r(0,0), 1);
    EXPECT_DOUBLE_EQ(r(0,1), 0);
    EXPECT_DOUBLE_EQ(r(0,2), 0);
    EXPECT_DOUBLE_EQ(r(1,0), 0);
    EXPECT_DOUBLE_EQ(r(1,1), 1);
    EXPECT_DOUBLE_EQ(r(1,2), 0);
    EXPECT_DOUBLE_EQ(r(2,0), 0);
    EXPECT_DOUBLE_EQ(r(2,1), 0);
    EXPECT_DOUBLE_EQ(r(2,2), 1);
    
    
    // Now let's test to make sure the rotations transformations work
    // let's rotate item1 by +pi/2 radians (90 degrees)
    T << 1, 2, 3;
    R << 0, -1, 0, 1, 0, 0, 0, 0, 1;
    f1.setRotationAndTranslation(R, T);
    item1->setLocalTransform(f1);
    
    // global transform of item1
    p = item1->getGlobalTransform().getTranslation();
    r = item1->getGlobalTransform().getRotation();
    EXPECT_DOUBLE_EQ(p(0), 1);
    EXPECT_DOUBLE_EQ(p(1), 2);
    EXPECT_DOUBLE_EQ(p(2), 3);
    EXPECT_DOUBLE_EQ(r(0,0), 0);
    EXPECT_DOUBLE_EQ(r(0,1), -1);
    EXPECT_DOUBLE_EQ(r(0,2), 0);
    EXPECT_DOUBLE_EQ(r(1,0), 1);
    EXPECT_DOUBLE_EQ(r(1,1), 0);
    EXPECT_DOUBLE_EQ(r(1,2), 0);
    EXPECT_DOUBLE_EQ(r(2,0), 0);
    EXPECT_DOUBLE_EQ(r(2,1), 0);
    EXPECT_DOUBLE_EQ(r(2,2), 1);
    
    // local transform of item2
    p = item2->getGlobalTransform().getTranslation();
    r = item2->getGlobalTransform().getRotation();
    EXPECT_DOUBLE_EQ(p(0), 1);
    EXPECT_DOUBLE_EQ(p(1), 3);
    EXPECT_DOUBLE_EQ(p(2), 3);
    EXPECT_DOUBLE_EQ(r(0,0), 0);
    EXPECT_DOUBLE_EQ(r(0,1), -1);
    EXPECT_DOUBLE_EQ(r(0,2), 0);
    EXPECT_DOUBLE_EQ(r(1,0), 1);
    EXPECT_DOUBLE_EQ(r(1,1), 0);
    EXPECT_DOUBLE_EQ(r(1,2), 0);
    EXPECT_DOUBLE_EQ(r(2,0), 0);
    EXPECT_DOUBLE_EQ(r(2,1), 0);
    EXPECT_DOUBLE_EQ(r(2,2), 1);
    
    // local transform of item3
    p = item3->getGlobalTransform().getTranslation();
    r = item3->getGlobalTransform().getRotation();
    EXPECT_DOUBLE_EQ(p(0), 0);
    EXPECT_DOUBLE_EQ(p(1), 2);
    EXPECT_DOUBLE_EQ(p(2), 3);
    EXPECT_DOUBLE_EQ(r(0,0), 0);
    EXPECT_DOUBLE_EQ(r(0,1), -1);
    EXPECT_DOUBLE_EQ(r(0,2), 0);
    EXPECT_DOUBLE_EQ(r(1,0), 1);
    EXPECT_DOUBLE_EQ(r(1,1), 0);
    EXPECT_DOUBLE_EQ(r(1,2), 0);
    EXPECT_DOUBLE_EQ(r(2,0), 0);
    EXPECT_DOUBLE_EQ(r(2,1), 0);
    EXPECT_DOUBLE_EQ(r(2,2), 1);
    
    
    
    // Now let's test to make sure the relative transformation works
    // The function returns item3 relative to item2 frame
    p = item3->getRelativeTransform(item2).getTranslation();
    r = item3->getRelativeTransform(item2).getRotation();
    EXPECT_DOUBLE_EQ(p(0), -1);
    EXPECT_DOUBLE_EQ(p(1), 1);
    EXPECT_DOUBLE_EQ(p(2), 0);
    EXPECT_DOUBLE_EQ(r(0,0), 1);
    EXPECT_DOUBLE_EQ(r(0,1), 0);
    EXPECT_DOUBLE_EQ(r(0,2), 0);
    EXPECT_DOUBLE_EQ(r(1,0), 0);
    EXPECT_DOUBLE_EQ(r(1,1), 1);
    EXPECT_DOUBLE_EQ(r(1,2), 0);
    EXPECT_DOUBLE_EQ(r(2,0), 0);
    EXPECT_DOUBLE_EQ(r(2,1), 0);
    EXPECT_DOUBLE_EQ(r(2,2), 1);
    
    
    /*
    // global transform of item1
    p = item1->getGlobalTransform().getTranslation();
    r = item1->getLocalTransform().getRotation();
    EXPECT_DOUBLE_EQ(p(0), 1);
    EXPECT_DOUBLE_EQ(p(1), 0);
    EXPECT_DOUBLE_EQ(p(2), 0);
    
    // global transform of item2
    p = item2->getGlobalTransform().getTranslation();
    EXPECT_DOUBLE_EQ(p(0), 1);
    EXPECT_DOUBLE_EQ(p(1), 2);
    EXPECT_DOUBLE_EQ(p(2), 0);
    
    // global transform of item3
    p = item3->getGlobalTransform().getTranslation();
    EXPECT_DOUBLE_EQ(p(0), 0);
    EXPECT_DOUBLE_EQ(p(1), 0);
    EXPECT_DOUBLE_EQ(p(2), 3);
    
    
    // transform of item2 relative to item3
    p = item2->getRelativeTransform(item3).getTranslation();
    EXPECT_DOUBLE_EQ(p(0), 1);
    EXPECT_DOUBLE_EQ(p(1), 2);
    EXPECT_DOUBLE_EQ(p(2), -3);
    */
}
     

//---------------------------------------------------------------------
/*!
 Test the frame transform methods
 */
//---------------------------------------------------------------------
TEST(Node, leaf){
    
    // create 2 world items
    NodePtr item1 = Node::create();
    NodePtr item2 = Node::create();
    
    EXPECT_EQ(item1->isLeaf(), true);
    EXPECT_EQ(item2->isLeaf(), true);
    
    item1->addChild(item2);
    
    EXPECT_EQ(item1->isLeaf(), false);
    EXPECT_EQ(item2->isLeaf(), true);
    
}



