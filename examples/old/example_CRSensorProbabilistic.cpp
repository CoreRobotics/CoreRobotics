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

// Use the CoreRobotics namespace
using namespace CoreRobotics;

// Create a global noise model
CRNoiseGaussian* measNoise;


// -------------------------------------------------------------
// Declare a probabilistic prediction model: zHat = fcn(x,sample)
Eigen::VectorXd probPredFcn(Eigen::VectorXd x,
                            bool sample){
    Eigen::VectorXd v(1);
    if (sample){
        v = measNoise->sample();
    } else {
        v << 0;
    }
    return x + v;  // observation
}

// -------------------------------------------------------------
// Declare a likelihood model: p = Pr(zObserved | zPredict)
double probLikFcn(Eigen::VectorXd zObserved,
                  Eigen::VectorXd zPredict){

    Eigen::MatrixXd cov(1,1);
    cov << 1;
    measNoise->setParameters(cov, zPredict);
    return measNoise->probability(zObserved);
}


// -------------------------------------------------------------
int main(void){
    
    std::cout << "*************************************\n";
    std::cout << "Demonstration of CRSensorProbabilistic.\n";
    
    // initialize the noise model with a mean and covariance
    Eigen::MatrixXd cov(1,1);
    cov << 1;
    Eigen::VectorXd mean(1);
    mean << 0;
    measNoise = new CRNoiseGaussian(cov, mean);
    
    
    // initialize a state vector
    Eigen::VectorXd x0(1);
    x0 << 5;
    
    
    // initialize a probabilistic sensor model
    CRSensorProbabilistic sensor = CRSensorProbabilistic(*probPredFcn,*probLikFcn,x0);
    
    
    // initialize a sensor prediction vector
    Eigen::VectorXd zPredict(1);
    zPredict = sensor.measurement(false);
    std::cout << "Predicted measurement = " << zPredict << std::endl;
    
     // init variables for storing measurement and probability
     double p;
     Eigen::VectorXd zMeasured(1);
     
     // now evaluate the likelihood for several test measurements
     std::cout << "Measurement | Likelihood\n";
     
     for (double i = 0; i <= 10; i=i+0.5){
         zMeasured << i;
         p = sensor.likelihood(zMeasured);
         printf("  %5.2f     |   %6.4f\n",i,p);
     }
}
// -------------------------------------------------------------
