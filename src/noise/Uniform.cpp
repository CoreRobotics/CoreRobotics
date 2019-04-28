/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#include "Uniform.hpp"

namespace cr {
namespace noise {

//------------------------------------------------------------------------------
/*!
 This method samples a random number from the distribution.\n

 \return - sampled state
 */
//------------------------------------------------------------------------------
Eigen::VectorXd Uniform::sample() {
  // Initialize the sampled vector and set to zero
  Eigen::VectorXd x = this->m_parameters.a;
  x.setZero();
  for (int i = 0; i < this->m_parameters.a.size(); i++) {
    x(i) = m_uniform(m_generator);
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
double Uniform::probability(const Eigen::VectorXd &i_x) {
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
