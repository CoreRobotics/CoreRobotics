/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#ifndef CR_NOISEGAUSSIAN_HPP_
#define CR_NOISEGAUSSIAN_HPP_

#include "Eigen/Dense"
#include "NoiseModel.hpp"
#include <random>

namespace cr {
namespace noise {

//! Gaussian paramter structure
struct ParamNoiseGaussian {
  Eigen::MatrixXd cov;
  Eigen::MatrixXd covInv;
  Eigen::VectorXd mean;
};

//------------------------------------------------------------------------------
/*!
 \class NoiseGaussian
 \ingroup noise

 \brief Implements a class for modeling Gaussian noise.

 \details
 ## Description
 NoiseGaussian implements methods for sampling from and modeling
 multivariate Gaussian noise (see [1-3]). The Gaussian is completely
 defined by a mean \f$\mu\f$ and covariance \f$\Sigma\f$.

 - NoiseGaussian::setParameters sets the parameters of the noise model
 - NoiseGaussian::sample samples from the noise model
 - NoiseGaussian::probability evaluates the probability

 ## Example
 This example demonstrates use of the NoiseGaussian class.

 \include example_NoiseGaussian.cpp

 ## References
 [1] J. Crassidis and J. Junkins, "Optimal Estimation of Dynamic Systems",
 Ed. 2, CRC Press, 2012. \n\n

 [2] S. Thrun, W. Burgard, and D. Fox, "Probabilistic Robotics", MIT Press,
 2006. \n\n

 [3] en.wikipedia.org/wiki/Multivariate_normal_distribution
 */
//------------------------------------------------------------------------------
class NoiseGaussian : public NoiseModel {

  // Constructor and Destructor
public:
  //! Class constructor
  NoiseGaussian(Eigen::MatrixXd i_cov, Eigen::VectorXd i_mean, unsigned i_seed);
  NoiseGaussian(Eigen::MatrixXd i_cov, Eigen::VectorXd i_mean);
  NoiseGaussian();
  ~NoiseGaussian(){};

  // Get/Set Methods
public:
  //! Set the parameters that describe the distribution
  using NoiseModel::setParameters;
  void setParameters(Eigen::MatrixXd i_cov, Eigen::VectorXd i_mean);

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
  ParamNoiseGaussian m_parameters;
};

} // namespace noise
} // namespace cr

#endif
