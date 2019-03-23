/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#define _USE_MATH_DEFINES
#include "Gmm.hpp"
#include "Eigen/Dense"
#include "math/Matrix.hpp"
#include <chrono>
#include <cmath>
#include <iostream>

namespace cr {
namespace noise {

//------------------------------------------------------------------------------
/*!
 The constructor creates a noise model.\n

 \param[in] i_seed - seed for the random generator.
 */
//------------------------------------------------------------------------------
Gmm::Gmm(unsigned i_seed) {

  this->m_seed = i_seed;
  this->m_generator.seed(this->m_seed);
}

Gmm::Gmm() { this->randomSeed(); }

//------------------------------------------------------------------------------
/*!
Destructor.\n
*/
//------------------------------------------------------------------------------
Gmm::~Gmm() {
  for (unsigned k = 0; k < m_parameters.models.size(); k++) {
    // delete m_parameters.models.at(k); // this causes malloc errors
  }
  m_parameters.models.clear();
  m_parameters.weights.clear();
}

//------------------------------------------------------------------------------
/*!
 This method adds a distribution to the mixture model.

 \param[in] i_model - a NoiseGaussian distribution.
 \param[in] i_weight - the corresponding weight of the added distribution.
 */
//------------------------------------------------------------------------------
void Gmm::add(NoiseGaussian i_model, double i_weight) {

  this->m_parameters.models.push_back(i_model);
  this->m_parameters.weights.push_back(i_weight);
}

//------------------------------------------------------------------------------
/*!
 This method samples a random number from the mixture model.\n

 \return - sampled state.
 */
//------------------------------------------------------------------------------
Eigen::VectorXd Gmm::sample(void) {

  // return the sum of the weights
  double sum_of_weights = 0;

  for (size_t i = 0; i < m_parameters.weights.size(); i++) {
    sum_of_weights += m_parameters.weights[i];
  }

  // now push into a cdf vector
  std::vector<double> cdf;

  cdf.resize(m_parameters.weights.size());

  double wPrev = 0;

  for (size_t i = 0; i < m_parameters.weights.size(); i++) {
    cdf[i] = m_parameters.weights[i] / sum_of_weights + wPrev;
    wPrev = cdf[i];
  }

  // set up a uniform sample generator \in [0,1]
  std::uniform_real_distribution<double> uniform(0.0, 1.0);
  double s = uniform(this->m_generator);

  // Now iterate through the cdf and get the index (inverse CDF discrete
  // sampling)
  int index = 0;

  while (s > cdf[index]) {
    index++;
  }

  // Finally sample from the distribution specified by index
  return this->m_parameters.models[index].sample();
}

//------------------------------------------------------------------------------
/*!
 This method returns the probability of x.\n

 \param[in] i_x - random number to evaluate
 \return - probability of i_x
 */
//------------------------------------------------------------------------------
double Gmm::probability(Eigen::VectorXd i_x) {

  // return the sum of the weights
  double sum_of_weights = 0;

  for (size_t i = 0; i < m_parameters.weights.size(); i++) {
    sum_of_weights += m_parameters.weights[i];
  }

  // get the probability of the observation for each dist in the
  // mixture multiplied by the dist weight
  double p = 0.0;

  for (size_t i = 0; i < m_parameters.weights.size(); i++) {
    double weight = this->m_parameters.weights[i] / sum_of_weights;
    p += weight * this->m_parameters.models[i].probability(i_x);
  }

  return p;
}

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
    Eigen::MatrixXd SigmaYY =
        math::Matrix::reducedMatrix(m_parameters.models.at(k).m_parameters.cov,
                                    i_outputIndices, i_outputIndices);
    Eigen::MatrixXd SigmaYX =
        math::Matrix::reducedMatrix(m_parameters.models.at(k).m_parameters.cov,
                                    i_outputIndices, i_inputIndices);
    Eigen::MatrixXd SigmaXY =
        math::Matrix::reducedMatrix(m_parameters.models.at(k).m_parameters.cov,
                                    i_inputIndices, i_outputIndices);
    Eigen::MatrixXd SigmaXX =
        math::Matrix::reducedMatrix(m_parameters.models.at(k).m_parameters.cov,
                                    i_inputIndices, i_inputIndices);
    Eigen::MatrixXd SigmaXXInv = SigmaXX.inverse();
    // Eigen::MatrixXd SigmaXXInv;
    // CRMatrix::svdInverse(SigmaXX, 1e-8, SigmaXXInv);
    /*
    if (CRMatrix::svdInverse(SigmaXX, 1e-8, SigmaXXInv) == CR_RESULT_SINGULAR) {
        printf("\n\nSingular SigmaXX inverse!!!!\n\n");
    }
    */
    Eigen::VectorXd MuY = math::Matrix::reducedVector(
        m_parameters.models.at(k).m_parameters.mean, i_outputIndices);
    Eigen::VectorXd MuX = math::Matrix::reducedVector(
        m_parameters.models.at(k).m_parameters.mean, i_inputIndices);

    // Return the mean, covariance, and weight
    Eigen::VectorXd Mu = MuY + SigmaYX * SigmaXXInv * (i_x - MuX);
    Eigen::MatrixXd Sigma = SigmaYY - SigmaYX * SigmaXXInv * SigmaXY;
    weight = m_parameters.weights.at(k) * mvnpdf(i_x, MuX, SigmaXX);
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

// evaluation of the multivariate normal pdf
double Gmm::mvnpdf(Eigen::VectorXd i_x, Eigen::VectorXd i_mean,
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
    double arg = -0.5 * error.transpose() * i_covariance.inverse() * error;
    return k * exp(arg);
  }
}

} // namespace noise
} // namespace cr
