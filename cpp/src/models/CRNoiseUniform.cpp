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
 
 \param[in] in_a - lower bound of the uniform distribution domain
 \param[in] in_b - upper bound of the uniform distribution domain
 \param[in] in_seed - seed for the random generator
 */
//---------------------------------------------------------------------
CRNoiseUniform::CRNoiseUniform(Eigen::VectorXd in_a,
                               Eigen::VectorXd in_b,
                               unsigned in_seed){
    
    this->setParameters(in_a,in_b);
    this->m_seed = in_seed;
    this->m_generator.seed(this->m_seed);
    
}
    
    
CRNoiseUniform::CRNoiseUniform(Eigen::VectorXd in_a,
                               Eigen::VectorXd in_b){
    
    this->setParameters(in_a,in_b);
    this->randomSeed();
    
}
    
    
CRNoiseUniform::CRNoiseUniform(){
    
    Eigen::VectorXd a(1);
    Eigen::VectorXd b(1);
    a(0) = 0;
    b(0) = 1;
    this->setParameters(a,b);
    this->randomSeed();
    
}
    
    
//=====================================================================
/*!
 This method sets the paramters of the noise model.  The Gaussian is the
 multivariate standard normal distribution with mean and covariance, as
 in http://en.wikipedia.org/wiki/Normal_distribution
 
 \param[in] in_a - lower bound of the uniform distribution domain
 \param[in] in_b - upper bound of the uniform distribution domain
 */
//---------------------------------------------------------------------
void CRNoiseUniform::setParameters(Eigen::VectorXd in_a,
                                   Eigen::VectorXd in_b)
{
    this->m_parameters.a = in_a;
    this->m_parameters.b = in_b;
}


//=====================================================================
/*!
 This method samples a random number from the specified uniform
 distribution.\n
 
 \param[out] out_x - sampled state
 */
//---------------------------------------------------------------------
void CRNoiseUniform::sample(Eigen::VectorXd &out_x)
{
    
    // Uniform distribution
    std::uniform_real_distribution<double> uniform(0.0,1.0);
    
    for (int i=0; i<this->m_parameters.a.size(); i++){
        out_x(i) = uniform(this->m_generator);
    }
    
    
    // linearly scale the output of the unit uniform
    Eigen::VectorXd L = m_parameters.b - m_parameters.a;
    
    out_x = L.asDiagonal()*out_x + this->m_parameters.a;
}
    
    
//=====================================================================
/*!
 This method returns the probability of x.\n
 
 \param[in] in_x - state to evaluate
 \param[out] out_p - probability of in_x
 */
//---------------------------------------------------------------------
void CRNoiseUniform::probability(Eigen::VectorXd in_x, double &out_p)
{
    
    Eigen::VectorXd e = (this->m_parameters.b-this->m_parameters.a);
    out_p = 1/e.prod();
    
    
    for(int i = 0; i < in_x.size(); i++){
        if ((in_x(i) > this->m_parameters.b(i)) || (in_x(i) < this->m_parameters.a(i))){
            out_p = 0.0;
        }
    }
    
}


//=====================================================================
// End namespace
}
