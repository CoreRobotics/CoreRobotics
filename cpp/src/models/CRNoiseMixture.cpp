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
#include "CRNoiseMixture.hpp"
#include <chrono>


//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics {
    
    
//=====================================================================
/*!
 The constructor creates a noise model.\n
 
 \param[in] in_seed - seed for the random generator.
 */
//---------------------------------------------------------------------
CRNoiseMixture::CRNoiseMixture(unsigned in_seed) {
    this->seed = in_seed;
    this->generator.seed(this->seed);
}
CRNoiseMixture::CRNoiseMixture() {
    this->randomSeed();
}
    
    
//=====================================================================
/*!
 This method adds a distribution to the mixture model.
 
 \param[in] in_model - a CRNoiseModel distribution.
 \param[in] in_weight - the corresponding weight of the added distribution.
 */
//---------------------------------------------------------------------
void CRNoiseMixture::add(CRNoiseModel* in_model, double in_weight)
{
    this->parameters.models.push_back(in_model);
    this->parameters.weights.push_back(in_weight);
}


//=====================================================================
/*!
 This method samples a random number from the mixture model.\n
 
 \param[out] out_x - sampled state.
 */
//---------------------------------------------------------------------
void CRNoiseMixture::sample(Eigen::VectorXd &out_x)
{
    
    // return the sum of the weights
    double sum_of_weights = 0;
    for (size_t i = 0; i < parameters.weights.size(); i++) {
        sum_of_weights += parameters.weights[i];
    }
    
    // now push into a cdf vector
    std::vector<double> cdf;
    cdf.resize(parameters.weights.size());
    double wPrev = 0;
    for (size_t i = 0; i < parameters.weights.size(); i++) {
        cdf[i] = parameters.weights[i]/sum_of_weights + wPrev;
        wPrev = cdf[i];
    }
    
    // set up a uniform sample generator \in [0,1]
    std::uniform_real_distribution<double> uniform(0.0,1.0);
    double s = uniform(this->generator);
    
    // Now iterate through the cdf and get the index (inverse CDF discrete sampling)
    int index = 0;
    while(s > cdf[index]){
        index++;
    }
    
    // Finally sample from the distribution specified by index
    this->parameters.models[index]->sample(out_x);
    
}
    
    
//=====================================================================
/*!
 This method returns the probability of x.\n
 
 \param[in] in_x - state to evaluate
 \param[out] out_p - probability of in_x
 */
//---------------------------------------------------------------------
void CRNoiseMixture::probability(Eigen::VectorXd in_x, double &out_p)
{
    out_p = 0.0;
    double p = 0.0;
    for (size_t i = 0; i < parameters.weights.size(); i++) {
        this->parameters.models[i]->probability(in_x, p);
        out_p += this->parameters.weights[i]*p;
    }
}


//=====================================================================
// End namespace
}
