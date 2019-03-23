/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#include <iostream>
#include <cr/noise>
#include "gtest/gtest.h"

// Use the CoreRobotics namespace
using namespace cr::noise;

//
// Sample from the distribution and check the histogram
//
TEST(NoiseGaussian, Sample){
    
    // define the Gaussian properties
    Eigen::Matrix<double, 1, 1> mean;
    Eigen::Matrix<double, 1, 1> cov;
    mean << 5;
    cov << 1;
    
    // initialize a noise model
    NoiseGaussian normalNoise = NoiseGaussian();
    normalNoise.setParameters(cov, mean);
    
    // initialize parameters for experiments
    const int n=1000000; // number of experiments
    int p[10]={};
    
    // sample the distribution
    for (int i=0; i<n; ++i) {
        Eigen::VectorXd v = normalNoise.sample();
        if ((v(0)>=0.0)&&(v(0)<10.0)) ++p[int(v(0))];
    }
    
    // compute the expected distribution for each histogram bin
    double prob[10]={};
    for (int k=0; k<10; ++k) {
        double csum = 0;
        // integrate 100 step between k and k+1
        Eigen::VectorXd point(1);
        for (int j=0; j<100; ++j){
            point << double(k) + double(j) / 100;
            csum += normalNoise.probability(point);
        }
        prob[k] = double(n) * csum / 100.0;
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
