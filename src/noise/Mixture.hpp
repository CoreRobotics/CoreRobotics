/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#ifndef CR_NOISE_MIXTURE_HPP_
#define CR_NOISE_MIXTURE_HPP_

#include "Discrete.hpp"
#include "Distribution.hpp"
#include <vector>

namespace cr {
namespace noise {

//! Mixture paramter structure
template <typename DistributionType> struct MixtureParameters {
  virtual ~MixtureParameters() = default;

  //! Add a distribution to the model
  void add(DistributionType *i_model, double i_weight) {
    models.emplace_back(i_model);
    auto dp = discrete.getParameters();
    dp.weights.emplace_back(i_weight);
    discrete.setParameters(dp);
  }

  Discrete discrete;
  std::vector<DistributionType *> models;
};

//------------------------------------------------------------------------------
/*!
 \class Mixture
 \ingroup noise

 \brief A class template for modeling noise as a mixture of distributions.

 \details
 ## Description
 Mixture implements methods for sampling and modeling noise as a mixture of
 probability distributions [2-3].  Each specified noise model is also
 accompanied by a weight, indicating the probability of that distribution being
 selected for sampling of the noise.

 ## References
 [1] J. Crassidis and J. Junkins, "Optimal Estimation of Dynamic Systems",
 Ed. 2, CRC Press, 2012. \n\n

 [2] S. Thrun, W. Burgard, and D. Fox, "Probabilistic Robotics", MIT Press,
 2006. \n\n

 [3] en.wikipedia.org/wiki/Mixture_model
 */
//------------------------------------------------------------------------------
template<typename DomainType, 
  typename DistributionType = DistributionBase<DomainType>>
class Mixture
  : public Distribution<DomainType, MixtureParameters<DistributionType>> {

public:
  //! Constructor
  Mixture() = default;
  Mixture(MixtureParameters<DistributionType> i_parameters,
          unsigned i_seed = DistributionBase<DomainType>::randomSeed())
      : Distribution<DomainType, MixtureParameters<DistributionType>>(
            i_parameters, i_seed) {};

  //! Destructor
  virtual ~Mixture() = default;

  //! The sample function must be implemented.
  DomainType sample() override {
    unsigned index = this->m_parameters.discrete.sample();
    return this->m_parameters.models[index]->sample();
  }

  //! The probability (pdf: continuous, pmf: discrete) must be implemented.
  double probability(const DomainType &i_x) override {
    double p = 0.0;
    for (size_t i = 0; i < this->m_parameters.models.size(); i++) {
      double weight = this->m_parameters.discrete.probability(i);
      p += weight * this->m_parameters.models[i]->probability(i_x);
    }
    return p;
  }

protected:
  //! Uniform distribution generator
  std::uniform_real_distribution<double> m_uniform{0.0, 1.0};
};

} // namespace noise
} // namespace cr

#endif
