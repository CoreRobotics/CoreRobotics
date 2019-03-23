/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#include "NoiseMixture.hpp"
#include "Eigen/Dense"
#include <chrono>

namespace cr {
namespace noise {

//------------------------------------------------------------------------------
/*!
 The constructor creates a noise model.\n

 \param[in] i_seed - seed for the random generator.
 */
//------------------------------------------------------------------------------
NoiseMixture::NoiseMixture(unsigned i_seed) {

  this->m_seed = i_seed;
  this->m_generator.seed(this->m_seed);
}

NoiseMixture::NoiseMixture() { this->randomSeed(); }

//------------------------------------------------------------------------------
/*!
 This method adds a distribution to the mixture model.

 \param[in] i_model - a NoiseModel distribution.
 \param[in] i_weight - the corresponding weight of the added distribution.
 */
//------------------------------------------------------------------------------
void NoiseMixture::add(NoiseModel *i_model, double i_weight) {

  this->m_parameters.models.push_back(i_model);
  this->m_parameters.weights.push_back(i_weight);
}

//------------------------------------------------------------------------------
/*!
 This method samples a random number from the mixture model.\n

 \return - sampled state.
 */
//------------------------------------------------------------------------------
Eigen::VectorXd NoiseMixture::sample(void) {

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

//------------------------------------------------------------------------------
/*!
 This method returns the probability of x.\n

 \param[in] i_x - random number to evaluate
 \return - probability of i_x
 */
//------------------------------------------------------------------------------
double NoiseMixture::probability(Eigen::VectorXd i_x) {

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

} // namespace noise
} // namespace cr
