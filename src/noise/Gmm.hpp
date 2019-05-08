/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#ifndef CR_NOISE_GMM_HPP_
#define CR_NOISE_GMM_HPP_

#include "Eigen/Dense"
#include "Gaussian.hpp"
#include "Mixture.hpp"
#include <vector>

namespace cr {
namespace noise {

//! GMM paramter structure
typedef MixtureParameters<Gaussian> GmmParameters;

//------------------------------------------------------------------------------
/*!
 \class Gmm
 \ingroup noise

 \brief Implements a class for Gaussian mixture models.

 \details
 ## Description
 Gmm implements methods for sampling and modeling noise as a mixture of Gaussian
 distributions [2-3].  Each specified Gaussian is also accompanied by a weight,
 indicating the probability of that distribution being selected for sampling of
 the noise.

 ## References
 [1] J. Crassidis and J. Junkins, "Optimal Estimation of Dynamic Systems",
 Ed. 2, CRC Press, 2012. \n\n

 [2] S. Thrun, W. Burgard, and D. Fox, "Probabilistic Robotics", MIT Press,
 2006. \n\n

 [3] H. Sung, "Gaussian Mixture Regression and Classification", PhD Thesis,
 Rice University, 2004. \n\n
 */
//------------------------------------------------------------------------------
class Gmm : public Mixture<Eigen::VectorXd, Gaussian> {

public:
  //! Constructor
  Gmm() = default;
  Gmm(GmmParameters i_parameters,
      unsigned i_seed = DistributionBase<Eigen::VectorXd>::randomSeed())
      : Mixture<Eigen::VectorXd, Gaussian>(i_parameters, i_seed){};
  
  //! Destructor
  virtual ~Gmm() = default;

  //! Perform Gaussian Mixture Regression (GMR)
  void regression(Eigen::VectorXd i_x, Eigen::VectorXi i_inputIndices,
                  Eigen::VectorXi i_outputIndices, Eigen::VectorXd &o_mean,
                  Eigen::MatrixXd &o_covariance);
};

} // namespace noise
} // namespace cr

#endif
