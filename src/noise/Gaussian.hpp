/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#ifndef CR_NOISE_GAUSSIAN_HPP_
#define CR_NOISE_GAUSSIAN_HPP_

#include "Distribution.hpp"
#include "Eigen/Dense"

namespace cr {
namespace noise {

//! Gaussian paramters
struct GaussianParameters {
  GaussianParameters() = delete;

  //! Initializes the multivariate standard normal
  GaussianParameters(std::size_t i_n)
      : cov(Eigen::MatrixXd::Identity(i_n, i_n)),
        mean(Eigen::VectorXd::Zero(i_n)){};

  Eigen::MatrixXd cov;
  Eigen::VectorXd mean;
};

//------------------------------------------------------------------------------
/*!
 \class Gaussian
 \ingroup noise

 \brief Implements a class for modeling Gaussian noise.

 \details
 ## Description
 Gaussian implements methods for sampling from and modeling multivariate
 Gaussian noise (see [1-3]). The Gaussian is completely defined by a mean
 \f$\mu\f$ and covariance \f$\Sigma\f$.

 ## References
 [1] J. Crassidis and J. Junkins, "Optimal Estimation of Dynamic Systems",
 Ed. 2, CRC Press, 2012. \n\n

 [2] S. Thrun, W. Burgard, and D. Fox, "Probabilistic Robotics", MIT Press,
 2006. \n\n

 [3] en.wikipedia.org/wiki/Multivariate_normal_distribution
 */
//------------------------------------------------------------------------------
class Gaussian : public Distribution<Eigen::VectorXd, GaussianParameters> {

public:
  //! Constructor
  Gaussian() = default;
  Gaussian(GaussianParameters i_parameters,
           unsigned i_seed = DistributionBase<Eigen::VectorXd>::randomSeed())
      : Distribution<Eigen::VectorXd, GaussianParameters>(i_parameters,
                                                          i_seed){};

  //! The sample function must be implemented.
  Eigen::VectorXd sample() override;

  //! The probability (pdf: continuous, pmf: discrete) must be implemented.
  double probability(const Eigen::VectorXd &i_x) override;

protected:
  //! Normal distribution generator
  std::normal_distribution<double> m_gaussian{0.0, 1.0};
};

} // namespace noise
} // namespace cr

#endif
