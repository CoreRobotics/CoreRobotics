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


#include <random>
#include <iostream>
#include "CoreRobotics.hpp"


// Use the CoreRobotics namespace
using namespace CoreRobotics;


// declare the callback functions
Eigen::VectorXd cdynamics(double t, Eigen::VectorXd x, Eigen::VectorXd u);
Eigen::VectorXd ddynamics(double t, Eigen::VectorXd x, Eigen::VectorXd u);
Eigen::VectorXd obsv(Eigen::VectorXd x, Eigen::VectorXd u);
double icdf(double);

// define a time step
double dt = 0.1;


// main
void CRTestModels(void){
    
    std::cout << "**********************\n";
    std::cout << "Running the CRTestModels\n";
    
    
    // define the time (s)
    double t = 0;
    
    // Define a state vector
    Eigen::VectorXd x(2);
    x << 0, 0; // IC
    
    // Define an input vector
    Eigen::VectorXd u(1);
    u << 1;
    
    // Define a measurement vector
    Eigen::VectorXd z(1);
    z << 0;
    
    // initialize a sensor model
    CRSensorModel sensor = CRSensorModel(obsv,x);
    sensor.simulateMeasurement(u, 0, z);
    // sensor.getMeasurement(z);
    
    
    // initialize a discrete dynamic model
    CRMotionModel dModel = CRMotionModel(ddynamics,x,dt,CR_MODEL_DISCRETE);
    
    std::cout << "\nDiscrete simulation:\n";
    printf("t = %3.1f, x = (%+6.4f, %+6.4f), z= (%6.4f)\n",t,x(0),x(1),z(0));
    
    while(t<2){
        dModel.simulateMotion(u, false);
        dModel.getState(x);
        dModel.getTime(t);
        sensor.setState(x);
        sensor.simulateMeasurement(u, false, z);
        printf("t = %3.1f, x = (%+6.4f, %+6.4f), z= (%6.4f)\n",t,x(0),x(1),z(0));
    }
    
    // Reset IC
    t = 0;
    x << 0, 0;
    z << 0;
    
    // initialize a continuous dynamic model
    CRMotionModel cModel = CRMotionModel(cdynamics, x,dt,CR_MODEL_CONTINUOUS);
    
    std::cout << "\nContinuous simulation:\n";
    printf("t = %3.1f, x = (%+6.4f, %+6.4f), z= (%6.4f)\n",t,x(0),x(1),z(0));
    
    while(t<2){
        cModel.simulateMotion(u, false);
        cModel.getState(x);
        cModel.getTime(t);
        sensor.setState(x);
        sensor.simulateMeasurement(u, false, z);
        printf("t = %3.1f, x = (%+6.4f, %+6.4f), z= (%6.4f)\n",t,x(0),x(1),z(0));
    }
    
    
    
    
    // initialize a noise model
    Eigen::Matrix<double,1,1> v;
    CRNoiseModel genericNoise = CRNoiseModel();
    genericNoise.setParameters(CR_NOISE_CONTINUOUS, icdf);
    
    
    std::cout << "\nInverse CDF noise sampling:\n";
    
    const int nrolls=10000;  // number of experiments
    const int nstars=100;    // maximum number of stars to distribute
    const int nintervals=10; // number of intervals
    int p[10]={};
    
    
    for (int i=0; i<nrolls; ++i) {
        genericNoise.sample(v);
        ++p[int(nintervals*v(0))];
    }
    
    std::cout << std::fixed; std::cout.precision(1);
    for (int i=0; i<nintervals; ++i) {
        std::cout << float(i)/nintervals << " - " << float(i+1)/nintervals << ": ";
        std::cout << std::string(p[i]*nstars/nrolls,'*') << std::endl;
    }
    
    
    
    // try a Gaussian model
    Eigen::Vector2d mean;
    mean << 5, 5;
    Eigen::VectorXd v2(2);
    CRNoiseGaussian normalNoise = CRNoiseGaussian();
    normalNoise.setParameters(2*Eigen::Matrix2d::Identity(),
                              mean);
    
    std::cout << "\nGaussian noise sampling:\n";
    
    int p2[10]={};
    
    for (int i=0; i<nrolls; ++i) {
        normalNoise.sample(v2);
        if ((v2(0)>=0.0)&&(v2(0)<10.0)) ++p2[int(v2(0))];
    }
    
    
    for (int i=0; i<10; ++i) {
        std::cout << i << " - " << (i+1) << ": ";
        std::cout << std::string(p2[i]*nstars/nrolls,'*') << std::endl;
    }
    
    
    
    /*
    // from c++
    const int nrolls=10000;  // number of experiments
    const int nstars=100;    // maximum number of stars to distribute
    const int nintervals=10; // number of intervals
    
    std::cout << "\nRandom model sampling:\n";
    
    // Normal random number generator
    std::default_random_engine generator;
    std::normal_distribution<double> gaussian(5.0,2.0);
    
    int p[10]={};
    
    for (int i=0; i<nrolls; ++i) {
        double number = gaussian(generator);
        if ((number>=0.0)&&(number<10.0)) ++p[int(number)];
    }
    
    std::cout << "normal_distribution (5.0,2.0):" << std::endl;
    
    for (int i=0; i<10; ++i) {
        std::cout << i << "-" << (i+1) << ": ";
        std::cout << std::string(p[i]*nstars/nrolls,'*') << std::endl;
    }
    
    
    
    
    // Uniform random number generator
    std::uniform_real_distribution<double> unfrmreal(0.0,1.0);
    
    int p2[nintervals]={};
    
    for (int i=0; i<nrolls; ++i) {
        double number = unfrmreal(generator);
        ++p2[int(nintervals*number)];
    }
    
    std::cout << "uniform_real_distribution (0.0,1.0):" << std::endl;
    std::cout << std::fixed; std::cout.precision(1);
    
    for (int i=0; i<nintervals; ++i) {
        std::cout << float(i)/nintervals << "-" << float(i+1)/nintervals << ": ";
        std::cout << std::string(p2[i]*nstars/nrolls,'*') << std::endl;
    }
    */
    
}



// Callback for the continuous motion model
Eigen::VectorXd cdynamics(double t, Eigen::VectorXd x, Eigen::VectorXd u){
    
    Eigen::Matrix2d A;
    Eigen::Vector2d B;
    
    A << 0.0, 1.0, -10.0, -6.0;
    B << 0.0, 1.0;
    
    return A*x + B*u;
}


// Callback for the doscrete motion model
Eigen::VectorXd ddynamics(double t, Eigen::VectorXd x, Eigen::VectorXd u){
    
    Eigen::Matrix2d A;
    Eigen::Vector2d B;
    Eigen::Matrix2d I = Eigen::Matrix2d::Identity();
    
    A << 0.0, 1.0, -10.0, -6.0;
    B << 0.0, 1.0;
    
    return (I+dt*A)*x + (dt*B)*u;
}


// Callback for the sensor model
Eigen::VectorXd obsv(Eigen::VectorXd x, Eigen::VectorXd u){
    
    Eigen::Matrix<double, 1, 2> C;
    Eigen::Matrix<double, 1, 1> D;
    
    C << 1, 0;
    D << 0;
    
    return C*x + D*u;
}


// Callback for inverse CDF (triangular PDF over [0,1])
double icdf(double P){
    
    return sqrt(P);
    
}



