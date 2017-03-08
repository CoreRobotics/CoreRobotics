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
 
 \param[in] - seed specifies the seed to use for the generator.  If not
 specified, a random seed is chosen.
 */
//---------------------------------------------------------------------
CRNoiseModel::CRNoiseModel(unsigned seed) {
    this->seed = seed;
    this->generator.seed(this->seed);
}
CRNoiseModel::CRNoiseModel() {
    
    // get a seed
    typedef std::chrono::steady_clock clock;
    clock::time_point t0 = clock::now();
    for(int i=0; i < 1000000; i++){
        clock::now();
    }
    clock::duration d = clock::now() - t0;
    this->seed = unsigned(10000*d.count());
    
    // set the seed
    this->generator.seed(this->seed);
}
    
    
//=====================================================================
/*!
 This method sets the paramters of the noise model.  The icdFunction is
 an inverse cumulative distribution of the form:
 
 \f$ v = cdf^{-1}(P) \f$
 
 where \f$v\f$ is the noise and \f$P\f$ is the cumulative probability
 [0,1].  The function must take a double between 0,1 and output a
 double.  See: `https://en.wikipedia.org/wiki/Inverse_transform_sampling
 
 \param[in] type - enumerator declaring distribution type, see 
                   CoreRobotics::CRNoiseType
 \param[in] icdFunction - inverse CDF of the distribution.  This function
                   is sampled with a uniform distribution over [0,1]
 */
//---------------------------------------------------------------------
void CRNoiseModel::setParameters(CRNoiseType type,
                                 double(icdFunction)(double))
{
    this->parameters.type = type;
    this->parameters.icdFunction = icdFunction;
}


//=====================================================================
/*!
 This method samples a random number using the inverse sampling method. 
 This method uniformly draws a number from [0,1] and applies it to the 
 supplied inverse cumulative distribution (ICD) function.\n
 
 \param[out] x - sampled state
 */
//---------------------------------------------------------------------
void CRNoiseModel::sample(Eigen::VectorXd &x)
{
    // Uniform real distribution
    std::uniform_real_distribution<double> uniform(0.0,1.0);
    x << (this->parameters.icdFunction)(uniform(this->generator));
}


//=====================================================================
// End namespace
}
