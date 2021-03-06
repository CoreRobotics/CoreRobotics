/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#ifndef CR_MATH_INTEGRATION_HPP_
#define CR_MATH_INTEGRATION_HPP_

#include "Eigen/Dense"
#include "core/Types.hpp"

namespace cr {
namespace math {

//------------------------------------------------------------------------------
/*!
\class Integration
\ingroup math

\brief This class implements numerical integration methods.

- Integration::forwardEulerStep performs forward Euler integration.
- Integration::rungeKuttaStep performs 4th order Runga-Kutta integration.

## References
[1] Kreyszig, E., Advanced Engineering Mathematics, Ed.9,
John Wiley & Sons, 2011.

*/
//------------------------------------------------------------------------------
class Integration {

public:
  //! Forward euler integration
  static Eigen::VectorXd
  forwardEulerStep(Eigen::VectorXd(i_dynamicSystem)(double, Eigen::VectorXd,
                                                    Eigen::VectorXd),
                   double i_t, Eigen::VectorXd i_x, Eigen::VectorXd i_u,
                   double i_dt);

  static Eigen::VectorXd forwardEulerStep(
      std::function<Eigen::VectorXd(double, Eigen::VectorXd, Eigen::VectorXd)>
          i_dynamicSystem,
      double i_t, Eigen::VectorXd i_x, Eigen::VectorXd i_u, double i_dt);

  //! Runge-Kutta 4th order integration
  static Eigen::VectorXd
  rungeKuttaStep(Eigen::VectorXd(i_dynamicSystem)(double, Eigen::VectorXd,
                                                  Eigen::VectorXd),
                 double i_t, Eigen::VectorXd i_x, Eigen::VectorXd i_u,
                 double i_dt);

  static Eigen::VectorXd rungeKuttaStep(
      std::function<Eigen::VectorXd(double, Eigen::VectorXd, Eigen::VectorXd)>
          i_dynamicSystem,
      double i_t, Eigen::VectorXd i_x, Eigen::VectorXd i_u, double i_dt);
};

} // namepsace math
} // namepsace cr

#endif
