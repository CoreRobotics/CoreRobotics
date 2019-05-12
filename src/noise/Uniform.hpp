/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#ifndef CR_NOISE_UNIFORM_HPP_
#define CR_NOISE_UNIFORM_HPP_

#include "Distribution.hpp"
#include "Eigen/Dense"

namespace cr {
namespace noise {

//------------------------------------------------------------------------------
/*!
 \class Uniform
 \ingroup noise

 \brief Implements a class for modeling uniform noise.

 \details
 ## Description
 Uniform implements methods for modeling and sampling uniform noise.  The
 uniform distribution is defined by a lower bound \f$a\f$ and upper bound
 \f$b\f$ on the range of the sampled state [3].  Every state in this range has
 equal probability of being sampled.

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
class Uniform : public Distribution<Eigen::VectorXd> {

public:
  //! Parameters
  class Parameters {
  public:
    Parameters() = default;
    Parameters(std::size_t i_dim)
      : m_a(Eigen::VectorXd::Zero(i_dim)),
        m_b(Eigen::VectorXd::Zero(i_dim)) {}
    virtual ~Parameters() = default;
    const Eigen::VectorXd& a() const { return m_a; }
    const Eigen::VectorXd& b() const { return m_b; }
    const double a(unsigned i) const { return m_a(i); }
    const double b(unsigned i) const { return m_b(i); }
    void setA(const Eigen::VectorXd& i_domainMin) { m_a = i_domainMin; } 
    void setB(const Eigen::VectorXd& i_domainMax) { m_b = i_domainMax; } 
  private:
    Eigen::VectorXd m_a;
    Eigen::VectorXd m_b;
  };

  //! Constructor
  Uniform() = default;
  Uniform(Parameters i_parameters,
          unsigned i_seed = Distribution<Eigen::VectorXd>::randomSeed())
      : Distribution<Eigen::VectorXd>(i_seed), m_parameters(i_parameters){};

  //! Destructor
  virtual ~Uniform() = default;

  CR_ASPECT_PARAMETER_MUTABLE(Uniform::Parameters);

  //! The sample function must be implemented.
  Eigen::VectorXd sample() override;

  //! The probability (pdf: continuous, pmf: discrete) must be implemented.
  double probability(const Eigen::VectorXd &i_x) override;

protected:
  //! Normal distribution generator
  std::uniform_real_distribution<double> m_uniform{0.0, 1.0};
};

} // namespace noise
} // namespace cr

#endif
