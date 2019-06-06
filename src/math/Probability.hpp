/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#ifndef CR_MATH_PROBABILITY_HPP_
#define CR_MATH_PROBABILITY_HPP_

#include "Eigen/Dense"
#include "core/Types.hpp"

namespace cr {
namespace math {

//------------------------------------------------------------------------------
/*!
\class Probability
\ingroup math

\brief This class implements statistical methods.

## References
[1] Kreyszig, E., Advanced Engineering Mathematics, Ed.9,
John Wiley & Sons, 2011.

*/
//------------------------------------------------------------------------------
class Probability {

public:
  //! Evaluate the multivariate normal dist p(x | mu, \Sigma)
  static double mvnpdf(Eigen::VectorXd i_x, Eigen::VectorXd i_mean,
                       Eigen::MatrixXd i_covariance);
};

} // namepsace math
} // namepsace cr

#endif
