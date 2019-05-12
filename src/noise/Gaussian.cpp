/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#define _USE_MATH_DEFINES
#include "Gaussian.hpp"
#include "math/Probability.hpp"
#include <cmath>

namespace cr {
namespace noise {

//------------------------------------------------------------------------------
/*!
 This method samples a random number from the Gaussian distribution.\n

 \return - sampled state
 */
//------------------------------------------------------------------------------
Eigen::VectorXd Gaussian::sample() {
  // initialize the sampled state and set to zero
  Eigen::VectorXd x = m_parameters.mean();
  x.setZero();

  // sample from standard normal distribution
  for (int i = 0; i < m_parameters.mean().size(); i++) {
    x(i) = m_gaussian(m_generator);
  }

  // compute the Cholesky decomposition of A & project x onto
  // the defined distribution
  Eigen::LLT<Eigen::MatrixXd> lltOfCov(m_parameters.cov());
  Eigen::MatrixXd L = lltOfCov.matrixL();
  return L * x + m_parameters.mean();
}

//------------------------------------------------------------------------------
/*!
 This method returns the probability of x.\n

 \param[in] i_x - state to evaluate
 \return - probability of i_x
 */
//------------------------------------------------------------------------------
double Gaussian::probability(const Eigen::VectorXd &i_x) {
  return math::Probability::mvnpdf(i_x, m_parameters.mean(), 
    m_parameters.cov());
}

} // namespace noise
} // namespace cr
