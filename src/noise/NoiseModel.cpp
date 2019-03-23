/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#include "NoiseModel.hpp"
#include "Eigen/Dense"
#include <chrono>

namespace cr {
namespace noise {

//------------------------------------------------------------------------------
/*!
 The constructor creates a noise model.\n

 \param[in] i_seed - seed for the random generator
 */
//------------------------------------------------------------------------------
NoiseModel::NoiseModel(unsigned i_seed) {
  this->m_seed = i_seed;
  this->m_generator.seed(this->m_seed);
}
NoiseModel::NoiseModel() { this->randomSeed(); }

//------------------------------------------------------------------------------
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
//------------------------------------------------------------------------------
void NoiseModel::setParameters(Eigen::VectorXd(i_icd)(double),
                               double (*i_prob)(Eigen::VectorXd)) {
  this->m_parameters.icdFunction = i_icd;
  this->m_parameters.probFunction = i_prob;
}

//------------------------------------------------------------------------------
/*!
 This method samples the distribution and returns the sample x.\n

 \return - sampled state
 */
//------------------------------------------------------------------------------
Eigen::VectorXd NoiseModel::sample(void) {
  // Uniform real distribution
  std::uniform_real_distribution<double> uniform(0.0, 1.0);
  return (this->m_parameters.icdFunction)(uniform(this->m_generator));
}

//------------------------------------------------------------------------------
/*!
 This method computes the probability of x from the density.\n

 \param[in] i_x - random vector to be evaluated
 \return - probability of i_x
 */
//------------------------------------------------------------------------------
double NoiseModel::probability(Eigen::VectorXd i_x) {
  return (this->m_parameters.probFunction)(i_x);
}

//------------------------------------------------------------------------------
/*!
 This method randomizes the seed of the NoiseModel using clock.\n
 */
//------------------------------------------------------------------------------
void NoiseModel::randomSeed(void) {
  // get a seed
  typedef std::chrono::steady_clock clock;
  clock::time_point t0 = clock::now();
  for (int i = 0; i < 1000000; i++) {
    clock::now();
  }
  clock::duration d = clock::now() - t0;
  this->m_seed = unsigned(10000 * d.count());

  // set the seed
  this->m_generator.seed(this->m_seed);
}

} // namespace noise
} // namespace cr
