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
#include "CRNoiseGaussian.hpp"
#include <chrono>


//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics {
    
    
//=====================================================================
/*!
 The constructor creates a noise model.\n
 
 \param[in] cov - covariance
 \param[in] mean - mean
 \param[in] seed - seed for the random generator
 */
//---------------------------------------------------------------------
CRNoiseGaussian::CRNoiseGaussian(Eigen::MatrixXd cov,
                                 Eigen::VectorXd mean,
                                 unsigned seed){
    this->setParameters(cov,mean);
    this->seed = seed;
    this->generator.seed(this->seed);
}
CRNoiseGaussian::CRNoiseGaussian(Eigen::MatrixXd cov,
                                 Eigen::VectorXd mean){
    this->setParameters(cov,mean);
}
CRNoiseGaussian::CRNoiseGaussian(){
    Eigen::MatrixXd cov(1,1);
    cov(0) = 1;
    Eigen::VectorXd mean(1);
    mean(0) = 0;
    this->setParameters(cov,mean);
}
    
    
//=====================================================================
/*!
 This method sets the paramters of the noise model.  The Gaussian is the
 multivariate standard normal distribution with mean and covariance, as
 in http://en.wikipedia.org/wiki/Normal_distribution
 
 \param[in] cov - covariance matrix of the Gaussian distribution
 \param[in] mean - mean vector of the Gaussian distribution
 */
//---------------------------------------------------------------------
void CRNoiseGaussian::setParameters(Eigen::MatrixXd cov,
                                    Eigen::VectorXd mean)
{
    this->parameters.cov = cov;
    this->parameters.mean = mean;
}


//=====================================================================
/*!
 This method samples a random number from the Gaussian distribution.\n
 
 \param[out] x - sampled state
 */
//---------------------------------------------------------------------
void CRNoiseGaussian::sample(Eigen::VectorXd &x)
{
    // Normal distribution
    std::normal_distribution<double> gaussian(0.0,1.0);
    for (int i=0; i<this->parameters.mean.size(); i++){
        x(i) = gaussian(this->generator);
    }
    // compute the Cholesky decomposition of A
    Eigen::LLT<Eigen::MatrixXd> lltOfCov(this->parameters.cov);
    Eigen::MatrixXd L = lltOfCov.matrixL();
    x = L*x + this->parameters.mean;
}


//=====================================================================
// End namespace
}
