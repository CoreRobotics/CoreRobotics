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
 \version 0.0
 
 */
//=====================================================================


#include <iostream>
#include "CoreRobotics.hpp"

// Use the CoreRobotics namespace
using namespace CoreRobotics;

void test_CRNoiseDirac(void){
    
    std::cout << "*************************************\n";
    std::cout << "Demonstration of CRNoiseDirac.\n";
    
    // define the Dirac properties
    Eigen::VectorXd point(1);
    point << 5;
    
    // initialize a noise model
    CRNoiseDirac diracNoise = CRNoiseDirac();
    diracNoise.setParameters(point);
    
    // initialize a vector to sample into
    Eigen::VectorXd v(1);
    
    const int nrolls=10000;  // number of experiments
    const int nstars=20;     // maximum number of stars to distribute
    int p[10]={};
    
    // sample the distribution
    for (int i=0; i<nrolls; ++i) {
        diracNoise.sample(v);
        if ((v(0)>=0.0)&&(v(0)<10.0)) ++p[int(v(0))];
    }
    
    // print out the result with stars to indicate density
    double prob;
    std::cout << std::fixed; std::cout.precision(1);
    for (int i=0; i<10; ++i) {
        printf("%2i - %2i | ",i,i+1);
        Eigen::VectorXd point(1);
        point << double(i);
        diracNoise.probability(point, prob);
        printf("%6.4f | ",prob);
        std::cout << std::string(p[i]*nstars/nrolls,'*') << std::endl;
    }
    
}




