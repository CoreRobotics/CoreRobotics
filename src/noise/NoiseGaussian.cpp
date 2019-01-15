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

#define _USE_MATH_DEFINES
#include <cmath>
#include "Eigen/Dense"
#include "NoiseGaussian.hpp"
#include "math/Matrix.hpp"
#include <chrono>


//=====================================================================
// CoreRobotics namespace
namespace cr {
    
    
//=====================================================================
/*!
 The constructor creates a noise model.\n
 
 \param[in] i_cov - covariance
 \param[in] i_mean - mean
 \param[in] i_seed - seed for the random generator
 */
//---------------------------------------------------------------------
NoiseGaussian::NoiseGaussian(Eigen::MatrixXd i_cov,
                                 Eigen::VectorXd i_mean,
                                 unsigned i_seed){
    this->setParameters(i_cov,i_mean);
    this->m_seed = i_seed;
    this->m_generator.seed(this->m_seed);
}
NoiseGaussian::NoiseGaussian(Eigen::MatrixXd i_cov,
                                 Eigen::VectorXd i_mean){
    this->setParameters(i_cov,i_mean);
    this->randomSeed();
}
NoiseGaussian::NoiseGaussian(){
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
 
 \param[in] i_cov - covariance matrix of the Gaussian distribution
 \param[in] i_mean - mean vector of the Gaussian distribution
 */
//---------------------------------------------------------------------
void NoiseGaussian::setParameters(Eigen::MatrixXd i_cov,
                                    Eigen::VectorXd i_mean)
{
    this->m_parameters.cov = i_cov;
    this->m_parameters.covInv = i_cov.inverse();
    this->m_parameters.mean = i_mean;
}


//=====================================================================
/*!
 This method samples a random number from the Gaussian distribution.\n
 
 \return - sampled state
 */
//---------------------------------------------------------------------
Eigen::VectorXd NoiseGaussian::sample(void)
{
    // initialize the sampled state and set to zero
    Eigen::VectorXd x = this->m_parameters.mean;
    x.setZero();
    
    // sample from standard normal distribution
    std::normal_distribution<double> gaussian(0.0,1.0);
    for (int i=0; i<this->m_parameters.mean.size(); i++){
        x(i) = gaussian(this->m_generator);
    }
    
    // compute the Cholesky decomposition of A & project x onto
    // the defined distribution
    Eigen::LLT<Eigen::MatrixXd> lltOfCov(this->m_parameters.cov);
    Eigen::MatrixXd L = lltOfCov.matrixL();
    return L*x + this->m_parameters.mean;
}
    
    
//=====================================================================
/*!
 This method returns the probability of x.\n
 
 \param[in] i_x - state to evaluate
 \return - probability of i_x
 */
//---------------------------------------------------------------------
double NoiseGaussian::probability(Eigen::VectorXd i_x)
{
    // compute subarguments
    Eigen::MatrixXd cov2pi = 2*M_PI*this->m_parameters.cov;
    Eigen::VectorXd error = i_x - this->m_parameters.mean;
    
    // define the arguments (gain k and arg of exponent)
    double k = 1/sqrt(cov2pi.determinant());
    double arg = -0.5*error.transpose()*this->m_parameters.covInv*error;
    
    // return the probability
    return k*exp(arg);
}


//=====================================================================
// End namespace
}
