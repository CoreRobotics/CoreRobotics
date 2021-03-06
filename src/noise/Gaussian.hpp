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
class Gaussian : public Distribution<Eigen::VectorXd> {

public:
  //! Paramters
  class Parameters {
  public:
    Parameters() = default;
    Parameters(std::size_t i_dim)
        : m_cov(Eigen::MatrixXd::Identity(i_dim, i_dim)),
          m_mean(Eigen::VectorXd::Zero(i_dim)) {
      m_covInv = m_cov.inverse();
    }
    virtual ~Parameters() = default;
    const Eigen::MatrixXd& cov() const { return m_cov; }
    const Eigen::MatrixXd& covInv() const { return m_covInv; }
    const Eigen::VectorXd& mean() const { return m_mean; }
    void setCov(const Eigen::MatrixXd& i_covariance ) { 
      m_cov = i_covariance; 
      m_covInv = m_cov.inverse();
      }
    void setMean(const Eigen::VectorXd& i_mu ) { m_mean = i_mu; }
  private:
    Eigen::MatrixXd m_cov;
    Eigen::MatrixXd m_covInv;
    Eigen::VectorXd m_mean;
  };

  //! Constructor
  Gaussian() = default;
  Gaussian(const Parameters& i_parameters,
           unsigned i_seed = Distribution<Eigen::VectorXd>::randomSeed())
      : Distribution<Eigen::VectorXd>(i_seed), m_parameters(i_parameters) {};

  //! Destructor
  virtual ~Gaussian() = default;

  CR_ASPECT_PARAMETER_MUTABLE(Gaussian::Parameters);

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
