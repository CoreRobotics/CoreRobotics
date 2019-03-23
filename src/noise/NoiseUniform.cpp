/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#include "NoiseUniform.hpp"
#include "Eigen/Dense"
#include <chrono>

namespace cr {
namespace noise {

//------------------------------------------------------------------------------
/*!
 The constructor creates a noise model.\n

 \param[in] i_a - lower bound of the uniform distribution domain
 \param[in] i_b - upper bound of the uniform distribution domain
 \param[in] i_seed - seed for the random generator
 */
//------------------------------------------------------------------------------
NoiseUniform::NoiseUniform(Eigen::VectorXd i_a, Eigen::VectorXd i_b,
                           unsigned i_seed) {

  this->setParameters(i_a, i_b);
  this->m_seed = i_seed;
  this->m_generator.seed(this->m_seed);
}

NoiseUniform::NoiseUniform(Eigen::VectorXd i_a, Eigen::VectorXd i_b) {

  this->setParameters(i_a, i_b);
  this->randomSeed();
}

NoiseUniform::NoiseUniform() {

  Eigen::VectorXd a(1);
  Eigen::VectorXd b(1);
  a(0) = 0;
  b(0) = 1;
  this->setParameters(a, b);
  this->randomSeed();
}

//------------------------------------------------------------------------------
/*!
 This method sets the paramters of the noise model.  The Gaussian is the
 multivariate standard normal distribution with mean and covariance, as
 in http://en.wikipedia.org/wiki/Normal_distribution

 \param[in] i_a - lower bound of the uniform distribution domain
 \param[in] i_b - upper bound of the uniform distribution domain
 */
//------------------------------------------------------------------------------
void NoiseUniform::setParameters(Eigen::VectorXd i_a, Eigen::VectorXd i_b) {
  this->m_parameters.a = i_a;
  this->m_parameters.b = i_b;
}

//------------------------------------------------------------------------------
/*!
 This method samples a random number from the specified uniform
 distribution.\n

 \return - sampled state
 */
//------------------------------------------------------------------------------
Eigen::VectorXd NoiseUniform::sample(void) {

  // Initialize the sampled vector and set to zero
  Eigen::VectorXd x = this->m_parameters.a;
  x.setZero();

  // Uniform distribution
  std::uniform_real_distribution<double> uniform(0.0, 1.0);

  for (int i = 0; i < this->m_parameters.a.size(); i++) {
    x(i) = uniform(this->m_generator);
  }

  // linearly scale the output of the unit uniform
  Eigen::VectorXd L = m_parameters.b - m_parameters.a;

  // return the sampled vector
  return L.asDiagonal() * x + this->m_parameters.a;
}

//------------------------------------------------------------------------------
/*!
 This method returns the probability of x.\n

 \param[in] i_x - state to evaluate
 \return - probability of i_x
 */
//------------------------------------------------------------------------------
double NoiseUniform::probability(Eigen::VectorXd i_x) {

  Eigen::VectorXd e = (this->m_parameters.b - this->m_parameters.a);

  // compute the probability of sampling over the continuous domain
  double p = 1 / e.prod();

  // check if x is in the domain of the distribution
  for (int i = 0; i < i_x.size(); i++) {
    if ((i_x(i) > this->m_parameters.b(i)) ||
        (i_x(i) < this->m_parameters.a(i))) {
      p = 0.0;
    }
  }

  return p;
}

} // namespace noise
} // namespace cr
