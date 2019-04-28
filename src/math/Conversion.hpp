/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#ifndef CR_CONVERSIONS_HPP_
#define CR_CONVERSIONS_HPP_

#include "Eigen/Dense"
#include "core/Types.hpp"

namespace cr {
namespace math {

//------------------------------------------------------------------------------
/*!
\class Conversion
\ingroup math

\brief This class implements math conversions.

\details
## Description
This class implements vmath conversions.

- Conversion::deg2rad converts degress to radians
- Conversion::rad2deg converts radians to degrees.
- Conversion::wrapToPi wraps angles to +/- \f$\pi\f$ radians.

## Example
This example shows usage of the math functions.
\include example_CRMath.cpp

## References
[1] Kreyszig, E., Advanced Engineering Mathematics, Ed.9,
John Wiley & Sons, 2011.

*/
//------------------------------------------------------------------------------
class Conversion {

public:
  //! Convert angles in degrees to radians
  static double deg2rad(const double i_deg) { return M_PI * i_deg / 180.0; }

  //! Convert angles in radians to degrees
  static double rad2deg(const double i_rad) { return 180.0 * i_rad / M_PI; }

  //! Wrap angle (rad) to +/- pi
  static double wrapToPi(const double angle) {
    double y = std::fmod(angle + M_PI, 2 * M_PI);
    if (angle < 0) {
      y += 2 * M_PI;
    }
    return y - M_PI;
  }

  //! Wrap angle (rad) to +/- pi
  static Eigen::VectorXd wrapToPi(const Eigen::VectorXd &angle) {
    Eigen::VectorXd y(angle.size());
    for (int i = 0; i < angle.size(); i++) {
      y(i) = wrapToPi(angle(i));
    }
    return y;
  }
};

} // namepsace math
} // namepsace cr

#endif
