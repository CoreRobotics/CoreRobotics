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
// Define a simple loop element.
//---------------------------------------------------------------------
class simpleLoop : public Loop
{
    
    // init
    public:
    
        // constructor
        simpleLoop(){
            reset();
        }
    
    // Primary ThreadElement functions
    public:
    
        //! Step the thread element (called on each iteration of the thread)
        virtual void step() {
            
            // this is a simple low pass system with forward euler discretiation
            x = (1 - this->getUpdateRate() / m_tau) * x;
        };
    
        //! Reset the thread element (called on Thread::start())
        virtual void reset() {
            x = x0;
        };
    
    
    // Private members
    public:
    
        //! Initial condition
        double x0 = 2.0;
    
        //! internal state
        double x = 0;
    
        //! internal parameter
        double m_tau = 0.5;
    
};



//---------------------------------------------------------------------
/*!
 Test the reset function (called on construction)
 */
//---------------------------------------------------------------------
TEST(Loop, reset){
    
    // Create the thread element smart pointer
    simpleLoop* myLoop = new simpleLoop();
    
    // Create a thread and attach the thread element
    // LoopPtr myThread(new Loop(myElemRaw));
    
    // Query the internal state to make sure the reset call happened
    EXPECT_DOUBLE_EQ(myLoop->x, myLoop->x0);
    
    // destroy memory
    delete myLoop;
}



//---------------------------------------------------------------------
/*!
 Test the thread execution functionality
 */
//---------------------------------------------------------------------
TEST(Loop, execution){
    
    // Create the thread element smart pointer
    simpleLoop* myLoop = new simpleLoop();
    
    // Set the update rate to be super slow - 20 Hz
    double dt = 0.05;
    myLoop->setUpdateRate(dt);
    EXPECT_DOUBLE_EQ(myLoop->getUpdateRate(), dt);
    
    // Create a timer
    cr::Clock timer;
    
    // Start the thread
    // std::cout << "Start() call\n";
    myLoop->start();
    
    // wait for 1 second
    timer.startTimer();
    timer.sleep(1.0);
    
    // Pause the thread
    // std::cout << "Pause() call\n";
    myLoop->pause();
    
    EXPECT_NEAR(myLoop->getCurrentTime(), 1.0, dt); // within dt
    EXPECT_NEAR(myLoop->x, 0, 0.4 * myLoop->x0);  // 1 time constant
    
    // wait for 1 second
    timer.sleep(1.0);
    
    EXPECT_NEAR(myLoop->getCurrentTime(), 1.0, dt); // within dt
    EXPECT_NEAR(myLoop->x, 0, 0.4 * myLoop->x0);  // 1 time constant
    
    // start the thread
    // std::cout << "Start() call\n";
    myLoop->start();
    
    // wait for 1 second
    timer.sleep(1.0);
    
    // Pause the thread
    // std::cout << "Pause() call\n";
    myLoop->pause();
    
    EXPECT_NEAR(myLoop->getCurrentTime(), 2.0, dt); // within dt
    EXPECT_NEAR(myLoop->x, 0, 0.28 * myLoop->x0);  // 2 time constants
    
    // wait for 1 second
    timer.sleep(1.0);
    
    // start the thread
    // std::cout << "Start() call\n";
    myLoop->start();
    
    // wait for 1 second
    timer.sleep(1.0);
    
    // stop the thread
    // std::cout << "Stop() call\n";
    myLoop->stop();
    
    EXPECT_NEAR(myLoop->getCurrentTime(), 3.0, 2 * dt); // within 2dt
    EXPECT_NEAR(myLoop->x, 0, 0.07 * myLoop->x0);  // 3 time constants
    
    // wait for 1 second
    timer.sleep(1.0);
    
    // restart the thread
    // std::cout << "Start() call\n";
    myLoop->start();
    
    // wait for 1 second
    timer.sleep(1.0);
    
    // stop the thread
    // std::cout << "Stop() call\n";
    myLoop->stop();
    
    EXPECT_NEAR(myLoop->getCurrentTime(), 1.0, 2 * dt); // within 2dt
    EXPECT_NEAR(myLoop->x, 0, 0.4 * myLoop->x0);  // 1 time constant
}

