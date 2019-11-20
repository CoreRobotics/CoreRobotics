/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#ifndef CR_NOISE_PARTICLE_HPP_
#define CR_NOISE_PARTICLE_HPP_

#include "Discrete.hpp"
#include "Distribution.hpp"
#include <vector>

namespace cr {
namespace noise {

//! Particle paramter structure
template <typename DomainType> struct ParticleParameters {
  virtual ~ParticleParameters() = default;

  //! Add a distribution to the model
  void add(const DomainType &i_particle, double i_weight) {
    particles.emplace_back(i_particle);
    auto dp = discrete.getParameters();
    dp.weights.emplace_back(i_weight);
    discrete.setParameters(dp);
  }

  Discrete discrete;
  std::vector<const DomainType &> particles;
};

//------------------------------------------------------------------------------
/*!
 \class Particle
 \ingroup noise

 \brief Implements a class for modeling noise as set of particles and weights.

 \details
 ## Description
 Particle implements methods for sampling and modeling noise as a set of states
 and weights [2-3].  Each state (particle) is also accompanied by a weight,
 indicating the probability of that distribution being selected for sampling of
 the noise.

 ## References
 [1] J. Crassidis and J. Junkins, "Optimal Estimation of Dynamic Systems",
 Ed. 2, CRC Press, 2012. \n\n

 [2] S. Thrun, W. Burgard, and D. Fox, "Probabilistic Robotics", MIT Press,
 2006. \n\n

 [3] en.wikipedia.org/wiki/Mixture_model
 */
//------------------------------------------------------------------------------
template <typename DomainType>
class Particle
    : public Distribution<DomainType, ParticleParameters<DomainType>> {

public:
  //! Constructor
  Particle() = default;
  Particle(ParticleParameters<DomainType> i_parameters,
           unsigned i_seed = DistributionBase<DomainType>::randomSeed())
      : Distribution<DomainType, ParticleParameters<DomainType>>>
        (i_parameters, i_seed){};

  //! Destructor
  virtual ~Particle() = default;

  //! The sample function must be implemented.
  DomainType sample() override {
    unsigned index = this->m_parameters.discrete.sample();
    return this->m_parameters.particles[index];
  }

  //! The probability (pdf: continuous, pmf: discrete) must be implemented.
  double probability(const DomainType &i_x) override{
      //  TODO: compute the probability
  };
};

} // namespace noise
} // namespace cr

#endif
