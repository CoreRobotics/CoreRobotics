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

void test_CRNoiseMixture(void){
    
    std::cout << "*************************************\n";
    std::cout << "Demonstration of test_CRNoiseMixture.\n";
    
    // initialize a noise mixture model
    CRNoiseMixture mixModel = CRNoiseMixture();
    
    // define a Gaussian distribution & add it to the mixture model
    Eigen::MatrixXd cov(1,1);
    cov(0) = 1;
    Eigen::VectorXd mean(1);
    mean(0) = 3;
    CRNoiseGaussian* gaussian = new CRNoiseGaussian(cov, mean);
    mixModel.add(gaussian, 0.7);
    
    // define a uniform distribution & add it to the mixture model
    Eigen::VectorXd a(1);
    a(0) = 1;
    Eigen::VectorXd b(1);
    b(0) = 9;
    CRNoiseUniform* uniform = new CRNoiseUniform(a, b);
    mixModel.add(uniform, 0.6);
    
    // define a point mass distribution & add it to the mixture model
    Eigen::VectorXd c(1);
    c(0) = 8;
    CRNoiseDirac* dirac = new CRNoiseDirac(c);
    mixModel.add(dirac, 0.2);
    
    
    // initialize parameters for experiments
    const int nrolls=10000;  // number of experiments
    const int nstars=100;     // maximum number of stars to distribute
    int p[10]={};
    
    // sample the distribution
    for (int i=0; i<nrolls; ++i) {
        Eigen::VectorXd v = mixModel.sample();
        if ((v(0)>=0.0)&&(v(0)<10.0)) ++p[int(v(0))];
    }
    
    // print out the result with stars to indicate density
    std::cout << std::fixed; std::cout.precision(1);
    for (int i=0; i<10; ++i) {
        printf("%2i - %2i | ",i,i+1);
        Eigen::VectorXd point(1);
        point << double(i);
        double prob = mixModel.probability(point);
        printf("%6.4f | ",prob);
        std::cout << std::string(p[i]*nstars/nrolls,'*') << std::endl;
    }
}



