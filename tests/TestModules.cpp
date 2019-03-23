/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#include <iostream>
#include "gtest/gtest.h"

using namespace std;

int main(int argc, char **argv) {
    
    cout << "Running the CoreRobotics test suite." << endl;
    
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
