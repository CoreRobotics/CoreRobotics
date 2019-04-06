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

#include "CRNoiseMixture.hpp"
#include "Eigen/Dense"
#include <chrono>

//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics {

//=====================================================================
/*!
 The constructor creates a noise model.\n

 \param[in] i_seed - seed for the random generator.
 */
//---------------------------------------------------------------------
CRNoiseMixture::CRNoiseMixture(unsigned i_seed) {

  this->m_seed = i_seed;
  this->m_generator.seed(this->m_seed);
}

CRNoiseMixture::CRNoiseMixture() { this->randomSeed(); }

//=====================================================================
/*!
 This method adds a distribution to the mixture model.

 \param[in] i_model - a CRNoiseModel distribution.
 \param[in] i_weight - the corresponding weight of the added distribution.
 */
//---------------------------------------------------------------------
void CRNoiseMixture::add(CRNoiseModel *i_model, double i_weight) {

  this->m_parameters.models.push_back(i_model);
  this->m_parameters.weights.push_back(i_weight);
}

//=====================================================================
/*!
 This method samples a random number from the mixture model.\n

 \return - sampled state.
 */
//---------------------------------------------------------------------
Eigen::VectorXd CRNoiseMixture::sample(void) {

  // return the sum of the weights
  double sum_of_weights = 0;

  for (size_t i = 0; i < m_parameters.weights.size(); i++) {
    sum_of_weights += m_parameters.weights[i];
  }

  // now push into a cdf vector
  std::vector<double> cdf;

  cdf.resize(m_parameters.weights.size());

  double wPrev = 0;

  for (size_t i = 0; i < m_parameters.weights.size(); i++) {
    cdf[i] = m_parameters.weights[i] / sum_of_weights + wPrev;
    wPrev = cdf[i];
  }

  // set up a uniform sample generator \in [0,1]
  std::uniform_real_distribution<double> uniform(0.0, 1.0);
  double s = uniform(this->m_generator);

  // Now iterate through the cdf and get the index (inverse CDF discrete
  // sampling)
  int index = 0;

  while (s > cdf[index]) {
    index++;
  }

  // Finally sample from the distribution specified by index
  return this->m_parameters.models[index]->sample();
}

//=====================================================================
/*!
 This method returns the probability of x.\n

 \param[in] i_x - random number to evaluate
 \return - probability of i_x
 */
//---------------------------------------------------------------------
double CRNoiseMixture::probability(Eigen::VectorXd i_x) {

  // return the sum of the weights
  double sum_of_weights = 0;

  for (size_t i = 0; i < m_parameters.weights.size(); i++) {
    sum_of_weights += m_parameters.weights[i];
  }

  // get the probability of the observation for each dist in the
  // mixture multiplied by the dist weight
  double p = 0.0;

  for (size_t i = 0; i < m_parameters.weights.size(); i++) {
    double weight = this->m_parameters.weights[i] / sum_of_weights;
    p += weight * this->m_parameters.models[i]->probability(i_x);
  }

  return p;
}

//=====================================================================
// End namespace
}
