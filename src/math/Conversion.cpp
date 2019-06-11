/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#include "Conversion.hpp"

namespace cr {
namespace math {

double Conversion::deg2rad(const double i_deg) {
  return M_PI * i_deg / 180.0;
}

double Conversion::rad2deg(const double i_rad) {
  return 180.0 * i_rad / M_PI;
}

double Conversion::wrapToPi(const double angle) {
  double y = std::fmod(angle + M_PI, 2 * M_PI);
  if (angle < 0) {
    y += 2 * M_PI;
  }
  return y - M_PI;
}

Eigen::VectorXd Conversion::wrapToPi(const Eigen::VectorXd &angle) {
  Eigen::VectorXd y(angle.size());
  for (int i = 0; i < angle.size(); i++) {
    y(i) = wrapToPi(angle(i));
  }
  return y;
}

} // namepsace math
} // namepsace cr
