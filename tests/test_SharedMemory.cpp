/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#include <iostream>
#include <cr/core>
#include "gtest/gtest.h"

// Use the CoreRobotics namespace
using namespace cr;
using namespace cr::core;


//
// Test memory server set/get function
//
TEST(SharedMemory, Server){
    const char* memoryName = "MyMemory";
    SharedMemory server(memoryName, CR_MANAGER_SERVER);
    Eigen::VectorXd v(2);
    v << 0.0, 0.8;
    server.addSignal("signal_1", v);
    Eigen::VectorXd v2 = server.get("signal_1");
    EXPECT_EQ(2, v2.size());
    EXPECT_DOUBLE_EQ(0, v2(0));
    EXPECT_DOUBLE_EQ(0.8, v2(1));
    
    v << 1.0, 1.2;
    server.set("signal_1", v);
    v2 = server.get("signal_1");
    EXPECT_EQ(2, v2.size());
    EXPECT_DOUBLE_EQ(1.0, v2(0));
    EXPECT_DOUBLE_EQ(1.2, v2(1));
    server.removeSignal("signal_1");
}

//
// Test memory client set/get function
//
TEST(SharedMemory, Client){
    const char* memoryName = "MyMemory";
    SharedMemory server(memoryName, CR_MANAGER_SERVER);
    Eigen::VectorXd v(1);
    v << 1.4;
    server.addSignal("signal_2", v);
    
    SharedMemory client(memoryName, CR_MANAGER_CLIENT);
    Eigen::VectorXd v2 = client.get("signal_2");
    EXPECT_EQ(1, v2.size());
    EXPECT_DOUBLE_EQ(1.4, v2(0));
    
    v << 2.4;
    client.set("signal_2", v);
    v2 = server.get("signal_2");
    EXPECT_EQ(1, v2.size());
    EXPECT_DOUBLE_EQ(2.4, v2(0));
    
    server.removeSignal("signal_2");
}

