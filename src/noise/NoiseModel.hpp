/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#ifndef CR_NOISEMODEL_HPP_
#define CR_NOISEMODEL_HPP_

#include "Eigen/Dense"
#include <random>

namespace cr {
namespace noise {

//! ICDF Paramter structure
struct CRParamNoiseGeneric {
  Eigen::VectorXd (*icdFunction)(double);
  double (*probFunction)(Eigen::VectorXd);
};

//------------------------------------------------------------------------------
/*!
 \class NoiseModel
 \ingroup noise

 \brief This class implements a noise model.

 \details
 ## Description
 NoiseModel implements methods for sampling from a distribution and
 serves as a base class to specfic distributions.  NoiseModel uses
 inverse transform sampling [3] to generate a state.  This requires the
 user to define an inverse cumulative density according to

 \f$ x = F^{-1}(P) \f$

 where \f$F^{-1}\f$ is the inverse cumulative density.

 Additionally, the NoiseModel uses a probability density function to
 return the probability of a state x.  Thus the user must also specify
 a density function of the type

 \f$ p = f(x) \f$.

 - NoiseModel::setParameters sets the noise model callback
 - NoiseModel::sample samples from the noise model
 - NoiseModel::probability evaluates the probability

 ## Example
 This example demonstrates use of the NoiseModel class.

 \include example_NoiseModel.cpp

 ## References
 [1] J. Crassidis and J. Junkins, "Optimal Estimation of Dynamic Systems",
 Ed. 2, CRC Press, 2012. \n\n

 [2] S. Thrun, W. Burgard, and D. Fox, "Probabilistic Robotics", MIT Press,
 2006. \n\n

 [3] en.wikipedia.org/wiki/Inverse_transform_sampling
 */
//------------------------------------------------------------------------------
class NoiseModel {

  // Constructor and Destructor
public:
  //! Class constructor
  NoiseModel(unsigned i_seed);
  NoiseModel();

  // Get/Set Methods
public:
  //! Set the parameters that describe the distribution
  virtual void setParameters(Eigen::VectorXd (*i_icd)(double),
                             double (*i_prob)(Eigen::VectorXd));

  // Public Methods
public:
  //! Sample a vector from the density
  virtual Eigen::VectorXd sample(void);

  //! Evaluate the probability from the density
  virtual double probability(Eigen::VectorXd i_x);

  // Protected Methods
protected:
  //! Random seed generator
  void randomSeed(void);

  // Public Members
public:
  //! Noise model parameters
  CRParamNoiseGeneric m_parameters;

  // Protected Members
protected:
  //! Seed value
  unsigned m_seed;

  //! Random number generator
  std::default_random_engine m_generator;
};

} // namespace noise
} // namespace cr

#endif
