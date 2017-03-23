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
#include "CRMath.hpp"
#include <chrono>
#include <math.h>


//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics {
    
    
//=====================================================================
/*!
 The constructor creates a noise model.\n
 
 \param[in] in_cov - covariance
 \param[in] in_mean - mean
 \param[in] in_seed - seed for the random generator
 */
//---------------------------------------------------------------------
CRNoiseGaussian::CRNoiseGaussian(Eigen::MatrixXd in_cov,
                                 Eigen::VectorXd in_mean,
                                 unsigned in_seed){
    this->setParameters(in_cov,in_mean);
    this->seed = in_seed;
    this->generator.seed(this->seed);
}
CRNoiseGaussian::CRNoiseGaussian(Eigen::MatrixXd in_cov,
                                 Eigen::VectorXd in_mean){
    this->setParameters(in_cov,in_mean);
    this->randomSeed();
}
CRNoiseGaussian::CRNoiseGaussian(){
    Eigen::MatrixXd cov(1,1);
    cov(0) = 1;
    Eigen::VectorXd mean(1);
    mean(0) = 0;
    this->setParameters(cov,mean);
    this->randomSeed();
}
    
    
//=====================================================================
/*!
 This method sets the paramters of the noise model.  The Gaussian is the
 multivariate standard normal distribution with mean and covariance, as
 in http://en.wikipedia.org/wiki/Normal_distribution
 
 \param[in] in_cov - covariance matrix of the Gaussian distribution
 \param[in] in_mean - mean vector of the Gaussian distribution
 */
//---------------------------------------------------------------------
void CRNoiseGaussian::setParameters(Eigen::MatrixXd in_cov,
                                    Eigen::VectorXd in_mean)
{
    this->parameters.cov = in_cov;
    this->parameters.covInv = in_cov.inverse();
    this->parameters.mean = in_mean;
}


//=====================================================================
/*!
 This method samples a random number from the Gaussian distribution.\n
 
 \param[out] out_x - sampled state
 */
//---------------------------------------------------------------------
void CRNoiseGaussian::sample(Eigen::VectorXd &out_x)
{
    // Normal distribution
    std::normal_distribution<double> gaussian(0.0,1.0);
    for (int i=0; i<this->parameters.mean.size(); i++){
        out_x(i) = gaussian(this->generator);
    }
    // compute the Cholesky decomposition of A
    Eigen::LLT<Eigen::MatrixXd> lltOfCov(this->parameters.cov);
    Eigen::MatrixXd L = lltOfCov.matrixL();
    out_x = L*out_x + this->parameters.mean;
}
    
    
//=====================================================================
/*!
 This method returns the probability of x.\n
 
 \param[in] in_x - state to evaluate
 \param[out] out_p - probability of in_x
 */
//---------------------------------------------------------------------
void CRNoiseGaussian::probability(Eigen::VectorXd in_x, double &out_p)
{
    // out_p = 1.2;
    Eigen::MatrixXd cov2pi = 2*CoreRobotics::PI*this->parameters.cov;
    Eigen::VectorXd error = in_x - this->parameters.mean;
    double k = 1/sqrt(cov2pi.determinant());
    double arg = -0.5*error.transpose()*this->parameters.covInv*error;
    out_p = k*exp(arg);
    /*   THINK THIS IS BROKEN!!! - fix first
    Eigen::MatrixXd cov2pi = 2*CoreRobotics::PI*this->parameters.cov;
    Eigen::VectorXd error = in_x - this->parameters.mean;
    double k = 1/sqrt(cov2pi.determinant());
    double arg = -0.5*error.transpose()*this->parameters.cov.inverse()*error;
    out_p = k*exp(arg);
     */
}


//=====================================================================
// End namespace
}
