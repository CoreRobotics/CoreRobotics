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

// declare an inverse cumulative distribution - this is the invserse
// CDF for a triangular distribution from [0,1].
Eigen::VectorXd myInvCDF(double P){
    Eigen::VectorXd v(1);
    v(0) = sqrt(P);
    return v;
}

// declare the probability density - this is the traditional density
// defined by a distribution
double myPdensity(Eigen::VectorXd x){
    return 2*x(0);
}

//
// Sample from the distribution and check the histogram
//
TEST(NoiseModel, Sample){
    
    // initialize a noise model
    NoiseModel genericNoise = NoiseModel();
    genericNoise.setParameters(*myInvCDF, *myPdensity);
    
    // initialize parameters for experiments
    const int n=1000000; // number of experiments
    const int nintervals=10; // number of intervals
    int p[nintervals]={};
    
    // sample the distribution
    for (int i=0; i<n; ++i) {
        Eigen::VectorXd v = genericNoise.sample();
        ++p[int(nintervals*v(0))];
    }
    
    
    // compute the expected distribution for each histogram bin
    double prob[10]={};
    for (int k=0; k<nintervals; k++){
        double csum = 0;
        
        // integrate 100 steps between k and k+1
        Eigen::VectorXd point(1);
        for (int j=0; j<100; ++j){
            point << double(k) + double(j) / 100.0;
            csum += genericNoise.probability(point) / 100.0;
        }
        // compute the probability
        prob[k] = csum * (double(n) / 100.0);
    }
    
    // We expect the histogram to be close to the binned pdf (1%)
    EXPECT_NEAR(prob[0], double(p[0]), 0.01 * double(n));
    EXPECT_NEAR(prob[1], double(p[1]), 0.01 * double(n));
    EXPECT_NEAR(prob[2], double(p[2]), 0.01 * double(n));
    EXPECT_NEAR(prob[3], double(p[3]), 0.01 * double(n));
    EXPECT_NEAR(prob[4], double(p[4]), 0.01 * double(n));
    EXPECT_NEAR(prob[5], double(p[5]), 0.01 * double(n));
    EXPECT_NEAR(prob[6], double(p[6]), 0.01 * double(n));
    EXPECT_NEAR(prob[7], double(p[7]), 0.01 * double(n));
    EXPECT_NEAR(prob[8], double(p[8]), 0.01 * double(n));
    EXPECT_NEAR(prob[9], double(p[9]), 0.01 * double(n));
}
