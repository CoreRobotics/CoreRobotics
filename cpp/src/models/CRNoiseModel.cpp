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
#include "CRNoiseModel.hpp"
#include <chrono>


//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics {
    
    
//=====================================================================
/*!
 The constructor creates a noise model.\n
 
 \param[in] in_seed - seed for the random generator
 */
//---------------------------------------------------------------------
CRNoiseModel::CRNoiseModel(unsigned in_seed) {
    this->m_seed = in_seed;
    this->m_generator.seed(this->m_seed);
}
CRNoiseModel::CRNoiseModel() {
    this->randomSeed();
}
    
    
//=====================================================================
/*!
 This method sets the paramters of the noise model. The icd function is
 an inverse cumulative distribution of the form:
 
 \f$ v = F^{-1}(P) \f$
 
 where \f$v\f$ is the noise and \f$P\f$ is the cumulative probability
 [0,1].  The function must take a double between 0,1 and output a
 double.  See: https://en.wikipedia.org/wiki/Inverse_transform_sampling
 
 \param[in] in_icd - inverse CDF of the distribution.  This function
                   is sampled with a uniform distribution over [0,1]
 */
//---------------------------------------------------------------------
void CRNoiseModel::setParameters(Eigen::VectorXd(in_icd)(double))
{
    this->m_parameters.icdFunction = in_icd;
}


//=====================================================================
/*!
 This method samples the distribution and returns the sample x.\n
 
 \param[out] out_x - sampled state
 */
//---------------------------------------------------------------------
void CRNoiseModel::sample(Eigen::VectorXd &out_x)
{
    // Uniform real distribution
    std::uniform_real_distribution<double> uniform(0.0,1.0);
    out_x = (this->m_parameters.icdFunction)(uniform(this->m_generator));
}
   

//=====================================================================
/*!
 This method computes the probability of x from the density.\n
 
 \param[in] in_x - random vector to be evaluated
 \param[out] out_p - probability of in_x
 */
//---------------------------------------------------------------------
void CRNoiseModel::probability(Eigen::VectorXd in_x, double &out_p)
{
    // TODO - this is gonna be tricky
    // 1.  Find the p* = F(x) (optimization)
    // 2a. Get x+ <- F^{-1}(p*+)
    // 2b. Get x- <- F^{-1}(p*-)
    // 3.  Central difference: p = (p*+ - p*-)/(x+ - x-)
    out_p = 1.0;
}

    
//=====================================================================
/*!
 This method randomizes the seed of the NoiseModel using clock.\n
 */
//---------------------------------------------------------------------
void CRNoiseModel::randomSeed()
{
    // get a seed
    typedef std::chrono::steady_clock clock;
    clock::time_point t0 = clock::now();
    for(int i=0; i < 1000000; i++){
        clock::now();
    }
    clock::duration d = clock::now() - t0;
    this->m_seed = unsigned(10000*d.count());
    
    // set the seed
    this->m_generator.seed(this->m_seed);
}


//=====================================================================
// End namespace
}
