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

// Declare a probabilistic model - fcn(x,z,s,zHat,p)
void probMdlFcn(Eigen::VectorXd x,
                Eigen::VectorXd z,
                bool s,
                Eigen::VectorXd &zHat,
                double &p){
    zHat = x;  // observation
    
    // noise model
    Eigen::MatrixXd cov(1,1);
    cov << 1;
    CRNoiseGaussian noise = CRNoiseGaussian();
    noise.setParameters(cov, zHat);
    if (s){
        noise.sample(zHat);
    }
    // p = 1/sqrt(2*CoreRobotics::PI)*exp(-(z-zHat)*(z-zHat)/2);
    p = 1.0;
}

// Declare a deterministic model - fcn(x,zHat)
void detMdlFcn(Eigen::VectorXd x,
               Eigen::VectorXd &zHat){
    zHat = x;  // observation
}

void test_CRSensorModel(void){
    
    std::cout << "*************************************\n";
    std::cout << "Demonstration of CRSensorModel.\n";
    
    // initialize a state vector
    Eigen::VectorXd x0(1);
    x0 << 5;
    
    // initialize a probabilistic sensor model
    CRSensorModel myProbSensor = CRSensorModel(*detMdlFcn,x0);
    
    // initialize a sensor prediction vector
    Eigen::VectorXd zPredict(1);
    myProbSensor.measurement(false, zPredict);
    std::cout << "Predicted measurement = " << zPredict << std::endl;
    
    
    // now evaluate the likelihood
    std::cout << "Likelihood:\n";
    double p;
    Eigen::VectorXd zMeasured(1);
    for (int i = 0; i < 10; ++i){
        zMeasured << double(i);
        myProbSensor.likelihood(zMeasured, p);
        std::cout << i << ": " << p << std::endl;
    }
    
    
}
