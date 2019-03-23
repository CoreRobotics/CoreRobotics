/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#ifndef CR_NOISEUNIFORM_HPP_
#define CR_NOISEUNIFORM_HPP_

#include "Eigen/Dense"
#include "NoiseModel.hpp"
#include <random>

namespace cr {
namespace noise {

//! Uniform paramter structure
struct ParamNoiseUniform {
  Eigen::VectorXd a;
  Eigen::VectorXd b;
};

//------------------------------------------------------------------------------
/*!
 \class NoiseUniform
 \ingroup noise

 \brief Implements a class for modeling uniform noise.

 \details
 ## Description
 NoiseUniform implements methods for modeling and sampling uniform
 noise.  The uniform distribution is defined by a lower bound \f$a\f$
 and upper bound \f$b\f$ on the range of the sampled state [3].  Every
 state in this range has equal probability of being sampled.

 - NoiseUniform::setParameters sets the parameters of the noise model
 - NoiseUniform::sample samples from the noise model
 - NoiseUniform::probability evaluates the probability

 ## Example
 This example demonstrates use of the NoiseUniform class.

 \include example_NoiseUniform.cpp

 ## References
 [1] J. Crassidis and J. Junkins, "Optimal Estimation of Dynamic Systems",
 Ed. 2, CRC Press, 2012. \n\n

 [2] S. Thrun, W. Burgard, and D. Fox, "Probabilistic Robotics", MIT Press,
 2006.
 \n\n

 [3] en.wikipedia.org/wiki/Uniform_distribution_(continuous)
 \n\n
 */
//------------------------------------------------------------------------------
class NoiseUniform : public NoiseModel {

  // Constructor and Destructor
public:
  //! Class constructor
  NoiseUniform(Eigen::VectorXd i_a, Eigen::VectorXd i_b, unsigned i_seed);
  NoiseUniform(Eigen::VectorXd i_a, Eigen::VectorXd i_b);
  NoiseUniform();

  // Get/Set Methods
public:
  //! Set the parameters that describe the distribution
  using NoiseModel::setParameters;
  void setParameters(Eigen::VectorXd i_a, Eigen::VectorXd i_b);

  // Public Methods
public:
  //! Sample a noise vector from the density
  using NoiseModel::sample;
  Eigen::VectorXd sample(void);

  //! Evaluate the probability from the density
  using NoiseModel::probability;
  double probability(Eigen::VectorXd i_x);

  // Public Members
public:
  //! Noise model parameters
  ParamNoiseUniform m_parameters;
};

} // namespace noise
} // namespace cr

#endif
