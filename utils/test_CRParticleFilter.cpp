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
CRNoiseGaussian* pfSensorNoise;
CRNoiseGaussian* pfMotionNoise;

// -------------------------------------------------------------
// Declare a probabilistic prediction model: zHat = fcn(x,sample)
Eigen::VectorXd pfSensor(Eigen::VectorXd x,
                         bool sample){
    Eigen::VectorXd v(1);
    if (sample){
        v = pfSensorNoise->sample();
    } else {
        v << 0;
    }
    return x + v;  // observation
}

// -------------------------------------------------------------
// Declare a likelihood model: p = Pr(zObserved | zPredict)
double pfSensorLik(Eigen::VectorXd zObserved,
                  Eigen::VectorXd zPredict){
    
    Eigen::MatrixXd cov(1,1);
    cov << 1;
    pfSensorNoise->setParameters(cov, zPredict);
    return pfSensorNoise->probability(zObserved);
}

// -------------------------------------------------------------
// Declare a probabilistic continuous motion model - xdot = fcn(t,x,u,s)
// \dox{x} = -x + u + w
Eigen::VectorXd pfMotion(double t, Eigen::VectorXd x, Eigen::VectorXd u, bool sample){
    Eigen::VectorXd w(1);
    if (sample){
        w = pfMotionNoise->sample();
    } else {
        w << 0;
    }
    return -x + u + w;  // motion
}


// -------------------------------------------------------------
void test_CRParticleFilter(void){
    
    std::cout << "*************************************\n";
    std::cout << "Demonstration of CRParticleFilter.\n";
    
    // define an initial state
    Eigen::VectorXd x(1);
    x << 0;
    
    // define an action
    Eigen::VectorXd u(1);
    u << 0;
    
    // define an initial measurement
    Eigen::VectorXd z(1);
    z << 0;
    
    // define estimates (mean and variance)
    Eigen::VectorXd x_mu(1);
    Eigen::VectorXd x_var(1);
    x_mu << 0;
    x_var << 1;
    
    // define the noise model properties
    Eigen::MatrixXd Q(1,1); // motion cov
    Eigen::MatrixXd R(1,1); // sensor cov
    Eigen::VectorXd mu(1); // zero-mean
    Q << 0.001;
    R << 0.1;
    mu << 0;
    
    // motion noise
    pfMotionNoise = new CRNoiseGaussian();
    pfMotionNoise->setParameters(Q, mu);
    
    // sensor noise
    pfSensorNoise = new CRNoiseGaussian();
    pfSensorNoise->setParameters(R, mu);
    
    // create a new sensor & motion model
    double dt = 0.1;
    CRSensorProbabilistic* sensorModel = new CRSensorProbabilistic(*pfSensor, *pfSensorLik, x);
    CRMotionProbabilistic* motionModel = new CRMotionProbabilistic(*pfMotion, CR_MOTION_CONTINUOUS, x, dt);
    
    // create copies for simulation
    CRSensorProbabilistic* sensorSim = new CRSensorProbabilistic(*pfSensor, *pfSensorLik, x);
    CRMotionProbabilistic* motionSim = new CRMotionProbabilistic(*pfMotion, CR_MOTION_CONTINUOUS, x, dt);
    
    // intialize a set of particles (N = 1000)
    std::vector<Eigen::VectorXd> particles;
    Eigen::VectorXd xs(1);
    for (int k=0; k < 1000; k++){
        xs << pfSensorNoise->sample(); // sample from large covariance initially
        particles.push_back(xs);
    }
    
    // create a particle filter
    CRParticleFilter pf(motionModel, sensorModel, particles);
    pf.m_Nresample = 10;
    
    // loop
    double t = 0; // init a time
    printf("Time (s) | Input | State | PF Estimate | PF Covariance\n");
    while(t <= 10) {
        
        // step the particle filter
        pf.step(u, z);
        x_mu = pf.getExpectedState();
        x_var = pf.getExpectedCovariance();
        
        // output the time and state
        printf("%4.1f    | %+.3f | %+.3f | %+.3f | %+.3f \n",t,u(0),x(0),x_mu(0),x_var(0));
        
        // step at t = 2.5
        if (t >= 2.5){
            u << 1.0;
        }
        
        // get next state & time
        x = motionSim->motion(u, true);
        sensorSim->setState(x);
        z = sensorSim->measurement(true);
        t = motionSim->getTime();
    }
    printf("%4.1f    | %+.3f | %+.3f | %+.3f | %+.3f \n",t,u(0),x(0),x_mu(0),x_var(0));
    
}
// -------------------------------------------------------------
