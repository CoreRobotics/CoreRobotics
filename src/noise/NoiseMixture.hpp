/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#ifndef CR_NOISEMIXTURE_HPP_
#define CR_NOISEMIXTURE_HPP_

#include "Eigen/Dense"
#include "NoiseModel.hpp"
#include <random>
#include <vector>

namespace cr {
namespace noise {

//! Mixture paramter structure
struct ParamNoiseMixture {
  std::vector<NoiseModel *> models;
  std::vector<double> weights;
};

//------------------------------------------------------------------------------
/*!
 \class NoiseMixture
 \ingroup noise

 \brief Implements a class for modeling noise as a mixture of distributions.

 \details
 ## Description
 NoiseMixture implements methods for sampling and modeling noise as a
 mixture of probability distributions [2-3].  Each specified noise model
 is also accompanied by a weight, indicating the probability of that
 distribution being selected for sampling of the noise.

 - NoiseMixture::add adds a noise model to the mixture
 - NoiseMixture::sample samples from the noise model
 - NoiseMixture::probability evaluates the probability

 ## Example
 This example demonstrates use of the NoiseMixture class.

 \include example_NoiseMixture.cpp

 ## References
 [1] J. Crassidis and J. Junkins, "Optimal Estimation of Dynamic Systems",
 Ed. 2, CRC Press, 2012. \n\n

 [2] S. Thrun, W. Burgard, and D. Fox, "Probabilistic Robotics", MIT Press,
 2006. \n\n

 [3] en.wikipedia.org/wiki/Mixture_model
 */
//------------------------------------------------------------------------------
class NoiseMixture : public NoiseModel {

  // Constructor and Destructor
public:
  //! Class constructor
  NoiseMixture(unsigned i_seed);
  NoiseMixture();

  // Add models to the mixture
public:
  //! Add a distribution to the mixture model
  void add(NoiseModel *i_model, double i_weight);

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
  ParamNoiseMixture m_parameters;

private:
  //! hide the setParameters method and make it do nothing
  using NoiseModel::setParameters;
  void setParameters(void){};
};

} // namespace noise
} // namespace cr

#endif
