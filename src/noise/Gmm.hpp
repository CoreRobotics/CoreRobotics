/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#ifndef CR_GMM_HPP_
#define CR_GMM_HPP_

#include "Eigen/Dense"
#include "NoiseGaussian.hpp"
#include "NoiseMixture.hpp"
#include "core/CRTypes.hpp"
#include <random>
#include <vector>

namespace cr {
namespace noise {

//! GMM paramter structure
struct ParamGaussianMixture {
  std::vector<NoiseGaussian> models;
  std::vector<double> weights;
};

//------------------------------------------------------------------------------
/*!
 \class Gmm
 \ingroup noise

 \brief Implements a class for Gaussian mixture models.

 \details
 ## Description
 Gmm implements methods for sampling and modeling noise as a
 mixture of Gaussian distributions [2-3].  Each specified Gaussian
 is also accompanied by a weight, indicating the probability of that
 distribution being selected for sampling of the noise.

 - CRGmm::add adds a noise model to the mixture
 - CRGmm::sample samples from the noise model
 - CRGmm::probability evaluates the probability
 - CRGmm::regression performs Gaussian mixture regression

 ## Example
 This example demonstrates use of the CRGmm class.

 \include example_CRGmm.cpp

 ## References
 [1] J. Crassidis and J. Junkins, "Optimal Estimation of Dynamic Systems",
 Ed. 2, CRC Press, 2012. \n\n

 [2] S. Thrun, W. Burgard, and D. Fox, "Probabilistic Robotics", MIT Press,
 2006. \n\n

 [3] H. Sung, "Gaussian Mixture Regression and Classification", PhD Thesis,
 Rice University, 2004. \n\n
 */
//------------------------------------------------------------------------------
class Gmm : public NoiseMixture {

  // Constructor and Destructor
public:
  //! Class constructor
  Gmm(unsigned i_seed);
  Gmm();

  //! Class destructor
  ~Gmm();

  // Add models to the mixture
public:
  //! Add a distribution to the mixture model
  void add(NoiseGaussian i_model, double i_weight);

  // Public Methods
public:
  //! Sample a noise vector from the density
  using NoiseMixture::sample;
  Eigen::VectorXd sample(void);

  //! Evaluate the probability from the density
  using NoiseMixture::probability;
  double probability(Eigen::VectorXd i_x);

  //! Perform Gaussian Mixture Regression (GMR)
  void regression(Eigen::VectorXd i_x, Eigen::VectorXi i_inputIndices,
                  Eigen::VectorXi i_outputIndices, Eigen::VectorXd &o_mean,
                  Eigen::MatrixXd &o_covariance);

  // Public Members
public:
  //! Noise model parameters
  ParamGaussianMixture m_parameters;

private:
  //! Evaluate the multivariate normal dist
  double mvnpdf(Eigen::VectorXd i_x, Eigen::VectorXd i_mean,
                Eigen::MatrixXd i_covariance);
};

} // namespace noise
} // namespace cr

#endif
