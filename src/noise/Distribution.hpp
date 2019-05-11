/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#ifndef CR_NOISE_DISTRIBUTION_HPP_
#define CR_NOISE_DISTRIBUTION_HPP_

#include <chrono>
#include <random>

namespace cr {
namespace noise {

//------------------------------------------------------------------------------
/*!
 \class DistributionBase
 \ingroup noise

 \brief A class template for an abstract Distribution model.

 \details
 ## Description
 DistributionBase defines prototype methods for sampling from a distribution and
 evaluating probability.  It serves as a base class to specfic distributions.

 ## References
 [1] J. Crassidis and J. Junkins, "Optimal Estimation of Dynamic Systems",
 Ed. 2, CRC Press, 2012. \n\n

 [2] S. Thrun, W. Burgard, and D. Fox, "Probabilistic Robotics", MIT Press,
 2006. \n\n

 [3] en.wikipedia.org/wiki/Inverse_transform_sampling
 */
//------------------------------------------------------------------------------
template <typename DomainType> class DistributionBase {

public:
  //! Class constructor
  DistributionBase(unsigned i_seed = randomSeed()) : m_seed(i_seed) {
    m_generator.seed(m_seed);
  }

  //! Class destructor
  virtual ~DistributionBase() = default;

public:
  //! The sample function must be implemented.
  virtual DomainType sample() = 0;

  //! The probability (pdf: continuous, pmf: discrete) must be implemented.
  virtual double probability(const DomainType &i_x) = 0;

  //! Random seed generator
  static unsigned randomSeed() {
    auto t0 = std::chrono::system_clock::now();
    auto epoch = t0.time_since_epoch();
    return unsigned(epoch.count());
  }

protected:
  //! Seed value
  unsigned m_seed;

  //! Random number generator
  std::default_random_engine m_generator;
};

//------------------------------------------------------------------------------
/*!
 \class Distribution
 \ingroup noise

 \brief A class template for an abstract Distribution model with parameters.

 \details
 ## Description
 DistributionBase defines prototype methods for sampling from a distribution and
 evaluating probability.  It serves as a base class to specfic distributions,
 with parameter specification.

 ## References
 [1] J. Crassidis and J. Junkins, "Optimal Estimation of Dynamic Systems",
 Ed. 2, CRC Press, 2012. \n\n

 [2] S. Thrun, W. Burgard, and D. Fox, "Probabilistic Robotics", MIT Press,
 2006. \n\n

 [3] en.wikipedia.org/wiki/Inverse_transform_sampling
 */
//------------------------------------------------------------------------------
template <typename DomainType, typename ParameterType>
class Distribution : public DistributionBase<DomainType> {

public:
  //! Class constructor
  Distribution() = default;
  Distribution(ParameterType i_parameters,
               unsigned i_seed = DistributionBase<DomainType>::randomSeed())
      : DistributionBase<DomainType>(i_seed), m_parameters(i_parameters) {};

  //! Class destructor
  virtual ~Distribution() = default;

public:
  //! Set the parameters that describe the distribution.
  virtual void setParameters(const ParameterType &i_parameters) {
    m_parameters = i_parameters;
  }

  //! Get the parameters that descrive the distribution.
  virtual const ParameterType &getParameters() { return m_parameters; }

protected:
  //! Noise model parameters
  ParameterType m_parameters;
};

} // namespace noise
} // namespace cr

#endif
