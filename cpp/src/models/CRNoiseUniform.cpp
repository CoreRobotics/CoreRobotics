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

#include "Eigen/Dense"
#include "CRNoiseUniform.hpp"
#include <chrono>


//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics {
    
    
//=====================================================================
/*!
 The constructor creates a noise model.\n
 
 \param[in] a - lower bound of the uniform distribution domain
 \param[in] b - upper bound of the uniform distribution domain
 \param[in] seed - seed for the random generator
 */
//---------------------------------------------------------------------
CRNoiseUniform::CRNoiseUniform(Eigen::VectorXd a,
                               Eigen::VectorXd b,
                               unsigned seed){
    this->setParameters(a,b);
    this->seed = seed;
    this->generator.seed(this->seed);
}
CRNoiseUniform::CRNoiseUniform(Eigen::VectorXd a,
                               Eigen::VectorXd b){
    this->setParameters(a,b);
}
CRNoiseUniform::CRNoiseUniform(){
    Eigen::VectorXd a(1);
    Eigen::VectorXd b(1);
    a(0) = 0;
    b(0) = 1;
    this->setParameters(a,b);
}
    
    
//=====================================================================
/*!
 This method sets the paramters of the noise model.  The Gaussian is the
 multivariate standard normal distribution with mean and covariance, as
 in http://en.wikipedia.org/wiki/Normal_distribution
 
 \param[in] a - lower bound of the uniform distribution domain
 \param[in] b - upper bound of the uniform distribution domain
 */
//---------------------------------------------------------------------
void CRNoiseUniform::setParameters(Eigen::VectorXd a,
                                   Eigen::VectorXd b)
{
    this->parameters.a = a;
    this->parameters.b = b;
}


//=====================================================================
/*!
 This method samples a random number from the specified uniform
 distribution.\n
 
 \param[out] x - sampled state
 */
//---------------------------------------------------------------------
void CRNoiseUniform::sample(Eigen::VectorXd &x)
{
    // Uniform distribution
    std::uniform_real_distribution<double> uniform(0.0,1.0);
    for (int i=0; i<this->parameters.a.size(); i++){
        x(i) = uniform(this->generator);
    }
    // linearly scale the output of the unit uniform
    Eigen::VectorXd L = parameters.b - parameters.a;
    x = L.asDiagonal()*x + this->parameters.a;
}


//=====================================================================
// End namespace
}
