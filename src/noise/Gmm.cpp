/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#define _USE_MATH_DEFINES
#include "Gmm.hpp"
#include "Eigen/Dense"
#include "math/Matrix.hpp"
#include "math/Probability.hpp"
#include <cmath>

namespace cr {
namespace noise {

//------------------------------------------------------------------------------
/*!
 This method performs the regression y = f(x) + e using by conditioning
 on the learned Gaussian Mixture Model.\n

 \param[in] i_x               - the input vector
 \param[in] i_inputIndices    - a vector of indices of the GMM means that
 defines the input.  Note the size of i_inputIndices must match the size
 of i_x.
 \param[in] i_outputIndices   - a vector of indices of the GMM means that
 defines the output.  Note the size of i_outputIndices will correspond to
 the number of rows and columns in the output mean and covariance.
 \param[out] o_mean           - the predicted mean
 \param[out] o_covariance     - the predicted covariance
 */
//------------------------------------------------------------------------------
void Gmm::regression(Eigen::VectorXd i_x, Eigen::VectorXi i_inputIndices,
                     Eigen::VectorXi i_outputIndices, Eigen::VectorXd &o_mean,
                     Eigen::MatrixXd &o_covariance) {
  // Define the smallest number of double precision
  double realmin = sqrt(2.225073858507202e-308);

  // Get cluster/input/output dimensions
  int NK = m_parameters.models.size();

  // Zero out the outputs
  o_mean.setZero(i_outputIndices.size());
  o_covariance.setZero(i_outputIndices.size(), i_outputIndices.size());

  // Init the intermediate variables
  double cweight =
      realmin; // initialize to smallest double so we don't ever divide by zero
  double weight = 0;
  Eigen::MatrixXd c1;
  c1.setZero(i_outputIndices.size(), i_outputIndices.size());

  // For each Gaussian dist
  for (int k = 0; k < NK; k++) {

    // Get matrices we need
    Eigen::MatrixXd SigmaYY = math::Matrix::reducedMatrix(
        m_parameters.models.at(k)->getParameters().cov, i_outputIndices,
        i_outputIndices);
    Eigen::MatrixXd SigmaYX = math::Matrix::reducedMatrix(
        m_parameters.models.at(k)->getParameters().cov, i_outputIndices,
        i_inputIndices);
    Eigen::MatrixXd SigmaXY = math::Matrix::reducedMatrix(
        m_parameters.models.at(k)->getParameters().cov, i_inputIndices,
        i_outputIndices);
    Eigen::MatrixXd SigmaXX = math::Matrix::reducedMatrix(
        m_parameters.models.at(k)->getParameters().cov, i_inputIndices,
        i_inputIndices);
    Eigen::MatrixXd SigmaXXInv = SigmaXX.inverse();
    // Eigen::MatrixXd SigmaXXInv;
    // CRMatrix::svdInverse(SigmaXX, 1e-8, SigmaXXInv);
    /*
    if (CRMatrix::svdInverse(SigmaXX, 1e-8, SigmaXXInv) == CR_RESULT_SINGULAR) {
        printf("\n\nSingular SigmaXX inverse!!!!\n\n");
    }
    */
    Eigen::VectorXd MuY = math::Matrix::reducedVector(
        m_parameters.models.at(k)->getParameters().mean, i_outputIndices);
    Eigen::VectorXd MuX = math::Matrix::reducedVector(
        m_parameters.models.at(k)->getParameters().mean, i_inputIndices);

    // Return the mean, covariance, and weight
    Eigen::VectorXd Mu = MuY + SigmaYX * SigmaXXInv * (i_x - MuX);
    Eigen::MatrixXd Sigma = SigmaYY - SigmaYX * SigmaXXInv * SigmaXY;
    weight = m_parameters.discrete.getParameters().weights.at(k) *
             math::Probability::mvnpdf(i_x, MuX, SigmaXX);
    cweight += weight;

    // compute the mean & covariance
    o_mean += weight * Mu;
    o_covariance += pow(weight, 2.0) * Sigma;

    // Eigen::VectorXd m2 = Mu.array().square();
    // Eigen::MatrixXd M2 = m2.asDiagonal();
    // c1 += pow(weight, 2.0) * (M2 + Sigma);
  }

  // Normalize
  o_mean = o_mean / cweight;
  o_covariance = o_covariance / pow(cweight, 2.0);
}

} // namespace noise
} // namespace cr
