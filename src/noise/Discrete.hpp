/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#ifndef CR_NOISE_DISCRETE_HPP_
#define CR_NOISE_DISCRETE_HPP_

#include "Distribution.hpp"
#include <vector>

namespace cr {
namespace noise {

//! Mixture paramter structure
struct DiscreteParameters {
  DiscreteParameters() = default;
  virtual ~DiscreteParameters() = default;

  //! The discrete weights.  At sample time, these are normalized to sum to 1
  std::vector<double> weights;
};

//------------------------------------------------------------------------------
/*!
 \class Discrete
 \ingroup noise

 \brief Implements a class for modeling noise as a discrete distribution.

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
class Discrete : public Distribution<unsigned, DiscreteParameters> {

public:
  //! Constructor
  Discrete() = default;
  Discrete(DiscreteParameters i_parameters,
           unsigned i_seed = DistributionBase<unsigned>::randomSeed())
      : Distribution<unsigned, DiscreteParameters>(i_parameters, i_seed){};
    
  //! Destructor
  virtual ~Discrete() = default;

  //! The sample function must be implemented.
  unsigned sample() override;

  //! The probability (pdf: continuous, pmf: discrete) must be implemented.
  double probability(const unsigned &i_x) override;

protected:
  //! Uniform distribution generator
  std::uniform_real_distribution<double> m_uniform{0.0, 1.0};
};

} // namespace noise
} // namespace cr

#endif
