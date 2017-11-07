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
\author  Cameron Devine

*/
//=====================================================================

#include <iostream>
#include "CoreRobotics.hpp"
#include <random>

using namespace CoreRobotics;

void test_CRKalmanFilter(void) {

	std::cout << "*************************************\n";
	std::cout << "Demonstration of CRKalmanFilter.\n";

	// Initialize matricies
	Eigen::MatrixXd A(2, 2);
	A << 1, 1, 0, 1;

	Eigen::MatrixXd B(2, 1);
	B << 0.5, 1;

	Eigen::MatrixXd C(1, 2);
	C << 1, 0;

	Eigen::MatrixXd Q(2, 2);
	Q << 0.1, 0, 0, 0.1;

	Eigen::MatrixXd R(1, 1);
	R << 0.3;

	Eigen::VectorXd x(2);
	x << 500, 0;

	Eigen::VectorXd Sigma0(2, 2);
	Sigma0 << 0.01, 0, 0, 0.01;

	Eigen::VectorXd u(1);
	u << -9.81;

	std::cout << "---------------------------------------------\n";
    std::cout << "CASE 1: Discrete time Kalman filter.\n";

	// Initialize the Kalman filter
	CRKalmanFilter kalman = CRKalmanFilter(A, B, C, Q, R, x, Sigma0);

	Eigen::VectorXd measurement(2);

	// Initialize a random number generator
	std::default_random_engine generator;
	std::normal_distribution<double> distribution(0, 1);
	
	for(int i = 0; i < 10; i++) {
		// Take a noisy measurement
		measurement = C * x + R * (Eigen::VectorXd(1) << distribution(generator)).finished();
		// Step the system forward
		x = A * x + B * u + Q * (Eigen::VectorXd(2) << distribution(generator), distribution(generator)).finished();
		kalman.step(u, measurement);
		// Print the result
		std::cout << "At i = " << i << std::endl;
		std::cout << "The true state is " << x.transpose() << std::endl;
		std::cout << "And the estimated state is " << kalman.getState().transpose() << std::endl;
		std::cout << "With covariance\n" << kalman.getCovariance() << std::endl;
	}

	std::cout << "---------------------------------------------\n";
    std::cout << "CASE 1: Continuous time Kalman filter.\n";

	// Test the system as continuous
	x << 500, 0;
	double dt = 0.1;

	kalman = CRKalmanFilter(A, B, C, Q, R, x, Sigma0, dt);

	for(int i = 0; i < 10; i++) {
		// Create a noisy measurement
		measurement = C * x + R * (Eigen::VectorXd(1) << distribution(generator)).finished();
		// Create process noise
		Eigen::VectorXd w = Q * (Eigen::VectorXd(2) << distribution(generator), distribution(generator)).finished();
		// Step the system forward
		std::function<Eigen::VectorXd(double, Eigen::VectorXd, Eigen::VectorXd)> stateEq =
			[A, B, w] (double t, Eigen::VectorXd x, Eigen::VectorXd u) {
				return A * x + B * u + w; };
		x = CRMath::rungeKuttaStep(stateEq, i * dt, x, u, dt);
		kalman.step(u, measurement);
		// Display the result
		std::cout << "At t = " << i * dt << std::endl;
		std::cout << "The true state is " << x.transpose() << std::endl;
		std::cout << "And the estimated state is " << kalman.getState().transpose() << std::endl;
		std::cout << "With covariance\n" << kalman.getCovariance() << std::endl;
	}
}
