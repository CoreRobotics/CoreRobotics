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


void callback1(void);
void callback2(void);


// Use the CoreRobotics namespace
using namespace CoreRobotics;

// define shared memory name
const char* memoryName = "MyMemory";

// boost shared memory
// using namespace boost::interprocess;

//Define an STL compatible allocator of ints that allocates from the managed_shared_memory.
//This allocator will allow placing containers in the segment
// typedef allocator<double, managed_shared_memory::segment_manager>  ShmemAllocator;

//Alias a vector that uses the previous STL-like allocator so that allocates
//its values from the segment
// typedef vector<double, ShmemAllocator> Signal;



void test_CRCore(void){
    
    std::cout << "**********************\n";
    std::cout << "Running the test_CRCore\n";
    
    double t;
    
    // Create a clock
    CRClock MyClock;
    
    // Start a timer
    MyClock.startTimer();
    MyClock.sleep(0.1);
    t = MyClock.getElapsedTime();
    
    std::cout << "t = " << t << std::endl;
    
    /*
    // init a point to shared memory
    managed_shared_memory* segment;
    
    // allocator
    const ShmemAllocator* alloc_inst;
     
    // Remove shared memory on construction and destruction
    struct shm_remove
    {
        shm_remove() { shared_memory_object::remove(memoryName); }
        ~shm_remove(){ shared_memory_object::remove(memoryName); }
    } remover;
    
    segment = new managed_shared_memory(create_only,
                                        memoryName,
                                        1024);
    
    // Initialize shared memory STL-compatible allocator
    alloc_inst = new ShmemAllocator(segment->get_segment_manager());
    */
     
    // Open a shared memory object
    CRSharedMemory mem(memoryName, CR_MANAGER_SERVER);
    // printf("test_CRCore, Line 72\n");
    
    // create 2 vectors of data
    Eigen::VectorXd v(2);
    v << 0.0, 0.8;
    
    mem.addSignal("signal_1", v);
    // Signal *myvector = segment->construct<Signal>("signal_1")(*alloc_inst);
    
    //Insert data in the vector
    // for(int i = 0; i < v.size(); ++i)
    //    myvector->push_back(v(i));
    
    

    // Create a thread
    CRThread myThread1 = CRThread(CR_PRIORITY_HIGH);
    myThread1.setCallback(*callback1);
    
    // Create another thread
    CRThread myThread2;
    myThread2.setCallback(*callback2);
    
    // start the threads
    myThread1.start();
    myThread2.start();
    
    // remove the signal
    mem.removeSignal("signal_1");
    // segment->destroy<Signal>("signal_1");
    
}



// Callback for the first thread
void callback1(void){

    // Open some shared memory
    // CRSharedMemory mem(memoryName, CR_MANAGER_CLIENT);
    /*
    // init a point to shared memory
    managed_shared_memory* segment;
    
    // allocator
    const ShmemAllocator* alloc_inst;
    
    // managed_shared_memory shm;
    // CRSharedMemory mem("MyMemory2", CR_MANAGER_CLIENT);
    segment = new managed_shared_memory(open_only,
                                        memoryName);
     */
     
    // create 2 vectors of data
    Eigen::VectorXd v(2);
    v << 0.1, 0.4;
    
    /*
    Signal *myvector = segment->find<Signal>("signal_1").first;
    int n = myvector->size();
    */
     
    // data
    /*
    for (int i = 0; i < n; i++){
        v(i) = myvector->at(i);
    }
     */
    
    
    // clock
    CRClock c;
    
    int i = 0;
    
    double dt = 0.1;
    double t  = 0.0;
    
    while(i<10){
        
        // v = mem.get("signal_1");
        /*
        for (int i = 0; i < n; i++){
            v(i) = myvector->at(i);
        }
         */
        
        c.startTimer();
        i++;
        printf("Thread 1: i = %i, signal = %+.4f, %+.4f\n",i,v(0),v(1));
        t = c.getElapsedTime();
        c.sleep(dt-t);
    }
}



// Callback for the second thread
void callback2(void){
    
    
    // Open some shared memory
    // CRSharedMemory mem(memoryName, CR_MANAGER_CLIENT);
    /*
    // init a point to shared memory
    managed_shared_memory* segment;
    
    // allocator
    const ShmemAllocator* alloc_inst;

    // Open some shared memory
    // CRSharedMemory mem("MyMemory2", CR_MANAGER_CLIENT);
    segment = new managed_shared_memory(open_only,
                                        memoryName);
     */

    // create 2 vectors of data
    Eigen::VectorXd v(2);
    v << 0.1, 0.4;
    
    // Signal *myvector = segment->find<Signal>("signal_1").first;
    // int n = myvector->size();
    
    // clock
    CRClock c;
    
    int i = 0;
    
    double dt = 0.25;
    double t  = 0.0;
    
    while(i<4){
        // printf("test_CRCore, Line 145\n");
        v = double(i) * v + v;
        
        /*
        for (int i = 0; i < n; i++){
            myvector->at(i) = v(i);
        }
         */
        
        // mem.set("signal_1", v);
        c.startTimer();
        i++;
        printf("Thread 2: i = %i, signal = %+.4f, %+.4f\n",i,v(0),v(1));
        t = c.getElapsedTime();
        c.sleep(dt-t);
    }
}
