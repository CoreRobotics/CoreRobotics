/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#include "Discrete.hpp"

namespace cr {
namespace noise {

//------------------------------------------------------------------------------
/*!
 This method samples a random index from the discrete model.\n

 \return - sampled state.
 */
//------------------------------------------------------------------------------
unsigned Discrete::sample() {

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
  double s = m_uniform(m_generator);

  // Now iterate through the cdf and get the index (inverse CDF discrete
  // sampling)
  unsigned index = 0;

  while (s > cdf[index]) {
    index++;
  }
  return index;
}

//------------------------------------------------------------------------------
/*!
 This method returns the probability of x.\n

 \param[in] i_x - random number to evaluate
 \return - probability of i_x
 */
//------------------------------------------------------------------------------
double Discrete::probability(const unsigned &i_x) {

  // return the sum of the weights
  double sum_of_weights = 0;

  for (size_t i = 0; i < m_parameters.weights.size(); i++) {
    sum_of_weights += m_parameters.weights[i];
  }

  return m_parameters.weights[i_x] / sum_of_weights;
}

} // namespace noise
} // namespace cr
