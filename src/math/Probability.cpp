/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#include "Probability.hpp"
#include "Eigen/Dense"
#include "Eigen/SVD"
#include "Matrix.hpp"

namespace cr {
namespace math {

//------------------------------------------------------------------------------
/*!
 This method computes the multivariate normal probability density.

 \f[
 \mathcal{N}(x | \mu, \Sigma)
 \f]

 \param [in] i_x - the state to evaluate
 \param [in] i_mean - the Gaussian mean vector
 \param [in] i_covariance - the Gaussian covariance matrix
 \return the multivariare pdf

 */
//------------------------------------------------------------------------------
double Probability::mvnpdf(Eigen::VectorXd i_x, Eigen::VectorXd i_mean,
                           Eigen::MatrixXd i_covariance) {
  // compute subarguments
  Eigen::MatrixXd cov2pi = 2 * M_PI * i_covariance;
  Eigen::VectorXd error = i_x - i_mean;

  Eigen::MatrixXd SigInv;
  if (math::Matrix::svdInverse(i_covariance, 1e-8, SigInv) ==
      core::CR_RESULT_SINGULAR) {
    return 0;
  } else {
    // define the arguments (gain k and arg of exponent)
    double k = 1 / sqrt(cov2pi.determinant());
    double arg = -0.5 * error.transpose() * SigInv * error;
    return k * exp(arg);
  }
}

} // namespace math
} // namespace cr
