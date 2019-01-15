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

#include "Eigen/Dense"
#include "NoiseModel.hpp"
#include <chrono>


//=====================================================================
// CoreRobotics namespace
namespace cr {
    
    
//=====================================================================
/*!
 The constructor creates a noise model.\n
 
 \param[in] i_seed - seed for the random generator
 */
//---------------------------------------------------------------------
NoiseModel::NoiseModel(unsigned i_seed) {
    this->m_seed = i_seed;
    this->m_generator.seed(this->m_seed);
}
NoiseModel::NoiseModel() {
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
 
 The probability density function returns the probability of x for the 
 distribution, i.e.:
 
 \f$ p = f(x) \f$
 
 \param[in] i_icd - inverse CDF of the distribution.  This function
                   is sampled with a uniform distribution over [0,1]
 \param[in] i_prob - probability density function that returns p(x)
 */
//---------------------------------------------------------------------
void NoiseModel::setParameters(Eigen::VectorXd(i_icd)(double),
                                 double(*i_prob)(Eigen::VectorXd))
{
    this->m_parameters.icdFunction = i_icd;
    this->m_parameters.probFunction = i_prob;
}


//=====================================================================
/*!
 This method samples the distribution and returns the sample x.\n
 
 \return - sampled state
 */
//---------------------------------------------------------------------
Eigen::VectorXd NoiseModel::sample(void)
{
    // Uniform real distribution
    std::uniform_real_distribution<double> uniform(0.0,1.0);
    return (this->m_parameters.icdFunction)(uniform(this->m_generator));
}
   

//=====================================================================
/*!
 This method computes the probability of x from the density.\n
 
 \param[in] i_x - random vector to be evaluated
 \return - probability of i_x
 */
//---------------------------------------------------------------------
double NoiseModel::probability(Eigen::VectorXd i_x)
{
    return (this->m_parameters.probFunction)(i_x);
}

    
//=====================================================================
/*!
 This method randomizes the seed of the NoiseModel using clock.\n
 */
//---------------------------------------------------------------------
void NoiseModel::randomSeed(void)
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
