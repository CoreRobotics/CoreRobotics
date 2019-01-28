//=====================================================================
/*
Software License Agreement (BSD-3-Clause License)
Copyright (c) 2019, CoreRobotics.
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
#include <cr/noise>
#include "gtest/gtest.h"

// Use the CoreRobotics namespace
using namespace cr::noise;

//
// Sample from the distribution and check the histogram
//
TEST(Gmm, Predict){
    
    // initialize a noise mixture model
    Gmm gmm = Gmm();
    
    // define a Gaussian distribution & add it to the mixture model
    Eigen::MatrixXd cov(4,4);   // covariance
    Eigen::VectorXd mean(4);    // mean
    
    // Add a couple Gaussians
    cov.diagonal() << 1.0, 2.0, 0.1, 4.0;
    mean << 1, 2, 3, 4;
    // NoiseGaussian* g1 = new NoiseGaussian(cov, mean);
    NoiseGaussian g1(cov, mean);
    gmm.add(g1, 0.5);
    
    cov.diagonal() << 0.1, 0.2, 4, 5;
    mean << 5, 6, 7, 8;
    // NoiseGaussian* g2 = new NoiseGaussian(cov, mean);
    NoiseGaussian g2(cov, mean);
    gmm.add(g2, 0.3);
    
    // cov.diagonal()  << 0.1, 0.2, 4, 5;
    cov << 0.1, 0, 0, 0.2,
           0, 0.2, 0, 0,
           0, 0, 4.0, -0.1,
           0.2, 0, -0.1, 5.0;
    mean << 9, 10, 11, 12;
    // NoiseGaussian* g3 = new NoiseGaussian(cov, mean);
    NoiseGaussian g3(cov, mean);
    gmm.add(g3, 0.2);
    
    // Construct the input/output booleans
    Eigen::VectorXi inputs(1);
    Eigen::VectorXi outputs(2);
    inputs << 0;
    outputs << 2, 3;
    
    // Predict
    Eigen::VectorXd x(1);
    x << 7.2;
    Eigen::VectorXd y_mu(2);
    Eigen::MatrixXd y_cov(2,2);
    gmm.regression(x, inputs, outputs, y_mu, y_cov);
    
    // We computed the above problem with an existing GMM/GMR library in MATLAB
    // and compare the evaluations here
    EXPECT_NEAR(10.701026290884815, y_mu(0), 1e-12);
    EXPECT_NEAR(8.236436140374021, y_mu(1), 1e-12);
    
    EXPECT_NEAR(3.704887227956277, y_cov(0,0), 1e-12);
    EXPECT_NEAR(-0.092618710733151, y_cov(0,1), 1e-12);
    EXPECT_NEAR(-0.092618710733151, y_cov(1,0), 1e-12);
    EXPECT_NEAR(4.265976289199881, y_cov(1,1), 1e-12);
}
